/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file CollisionPrevention.hpp
 * @author Tanja Baumann <tanja@auterion.com>
 *
 * CollisionPrevention controller.
 *
 */

#pragma once

#include <commander/px4_custom_mode.h>
#include <drivers/drv_hrt.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/module_params.h>
#include <systemlib/mavlink_log.h>
#include <uORB/topics/collision_constraints.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>

#include <matrix/matrix/math.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

using namespace time_literals;

class CollisionPrevention : public ModuleParams
{
   public:
    CollisionPrevention(ModuleParams *parent);
    ~CollisionPrevention() override = default;

    /**
     * Returns true if Collision Prevention is running
     */
    bool is_active();

    /**
     * Computes collision free setpoints
     * @param original_setpoint, setpoint before collision prevention intervention
     * @param max_speed, maximum xy speed
     * @param curr_pos, current vehicle position
     * @param curr_vel, current vehicle velocity
     */
    void modifySetpoint(matrix::Vector2f &original_setpoint, const float max_speed,
                        const matrix::Vector2f &curr_pos, const matrix::Vector2f &curr_vel,
                        float &z_setpoint);

    // lc add
    void _ConstrainSetpoint_ZDown(float &z_setpoint, float stick);
    void _ConstrainSetpoint_ZUp(float &z_setpoint, float stick);

   protected:
    obstacle_distance_s _obstacle_map_body_frame{};
    bool _data_fov[sizeof(_obstacle_map_body_frame.distances) /
                   sizeof(_obstacle_map_body_frame.distances[0])];
    uint64_t _data_timestamps[sizeof(_obstacle_map_body_frame.distances) /
                              sizeof(_obstacle_map_body_frame.distances[0])];
    uint16_t _data_maxranges[sizeof(_obstacle_map_body_frame.distances) /
                             sizeof(_obstacle_map_body_frame.distances[0])]; /**< in cm */

    void _addDistanceSensorData(distance_sensor_s &distance_sensor,
                                const matrix::Quatf &vehicle_attitude);

    /**
     * Updates obstacle distance message with measurement from offboard
     * @param obstacle, obstacle_distance message to be updated
     */
    void _addObstacleSensorData(const obstacle_distance_s &obstacle,
                                const matrix::Quatf &vehicle_attitude);

    /**
     * Computes an adaption to the setpoint direction to guide towards free space
     * @param setpoint_dir, setpoint direction before collision prevention intervention
     * @param setpoint_index, index of the setpoint in the internal obstacle map
     * @param vehicle_yaw_angle_rad, vehicle orientation
     */
    void _adaptSetpointDirection(matrix::Vector2f &setpoint_dir, int &setpoint_index,
                                 float vehicle_yaw_angle_rad);

    /**
     * Determines whether a new sensor measurement is used
     * @param map_index, index of the bin in the internal map the measurement belongs in
     * @param sensor_range, max range of the sensor in meters
     * @param sensor_reading, distance measurement in meters
     */
    bool _enterData(int map_index, float sensor_range, float sensor_reading);

    // Timing functions. Necessary to mock time in the tests
    virtual hrt_abstime getTime();
    virtual hrt_abstime getElapsedTime(const hrt_abstime *ptr);

   private:
    bool _interfering{
        false}; /**< states if the collision prevention interferes with the user input */
    bool _was_active{
        false}; /**< states if the collision prevention interferes with the user input */

    orb_advert_t _mavlink_log_pub{nullptr}; /**< Mavlink log uORB handle */

    uORB::Publication<collision_constraints_s> _constraints_pub{
        ORB_ID(collision_constraints)}; /**< constraints publication */
    uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{
        ORB_ID(obstacle_distance_fused)}; /**< obstacle_distance publication */
    uORB::Publication<vehicle_command_s> _vehicle_command_pub{
        ORB_ID(vehicle_command)}; /**< vehicle command do publication */

    uORB::SubscriptionData<obstacle_distance_s> _sub_obstacle_distance{
        ORB_ID(obstacle_distance)}; /**< obstacle distances received form a range sensor */
    uORB::SubscriptionData<vehicle_attitude_s> _sub_vehicle_attitude{ORB_ID(vehicle_attitude)};
    uORB::SubscriptionMultiArray<distance_sensor_s> _distance_sensor_subs{ORB_ID::distance_sensor};

    static constexpr uint64_t RANGE_STREAM_TIMEOUT_US{500_ms};
    static constexpr uint64_t TIMEOUT_HOLD_US{5_s};

    hrt_abstime _last_timeout_warning{0};
    hrt_abstime _time_activated{0};

    DEFINE_PARAMETERS(
        (ParamFloat<px4::params::CP_DIST>)
            _param_cp_dist, /**< collision prevention keep minimum distance */
        (ParamFloat<px4::params::CP_DELAY>)
            _param_cp_delay, /**< delay of the range measurement data*/
        (ParamFloat<px4::params::CP_GUIDE_ANG>)
            _param_cp_guide_ang, /**< collision prevention change setpoint angle */
        (ParamBool<px4::params::CP_GO_NO_DATA>)
            _param_cp_go_nodata,                            /**< movement allowed where no data*/
        (ParamFloat<px4::params::MPC_XY_P>)_param_mpc_xy_p, /**< p gain from position controller*/
        (ParamFloat<px4::params::MPC_JERK_MAX>)_param_mpc_jerk_max, /**< vehicle maximum jerk*/
        (ParamFloat<px4::params::MPC_ACC_HOR>)
            _param_mpc_acc_hor, /**< vehicle maximum horizontal acceleration*/
        (ParamBool<px4::params::CP_MODE>)_param_cp_mode, /**< collision prevention mode */
        (ParamFloat<px4::params::CP_DOWN_GATE1>)_param_cp_down_gate1, /**< Brake Mode */
        (ParamFloat<px4::params::CP_DOWN_GATE2>)_param_cp_down_gate2, /**< Brake Mode */
        (ParamFloat<px4::params::CP_DOWN_DECAY>)_param_cp_down_decay, /**< Brake Mode */
        (ParamFloat<px4::params::CP_UP_GATE1>)_param_cp_up_gate1,     /**< Brake Mode */
        (ParamFloat<px4::params::CP_UP_GATE2>)_param_cp_up_gate2,     /**< Brake Mode */
        (ParamFloat<px4::params::CP_UP_DECAY>)_param_cp_up_decay,     /**< Brake Mode */
        (ParamFloat<px4::params::CP_DECEL_DIS>)_param_cp_decel_dis,   /**< Bypass Mode */
        (ParamFloat<px4::params::CP_BYPASS_DIS>)_param_cp_bypass_dis, /**< Bypass Mode */
        (ParamFloat<px4::params::CP_BYPASS_VEL>)_param_cp_bypass_vel, /**< Bypass Mode */
        (ParamInt<px4::params::CP_NEI_BINS>)_param_cp_nei_bins,       /**< Bypass Mode */
        (ParamFloat<px4::params::CP_ALIGN_GAIN>)_param_cp_align_gain, /**< Bypass Mode */
        (ParamFloat<px4::params::CP_DIS_GAIN>)_param_cp_dis_gain,     /**< Bypass Mode */
        (ParamFloat<px4::params::CP_HOR_DENSE>)_param_cp_hor_dense,   /**< Bypass Mode */
        (ParamFloat<px4::params::CP_VER_GATE>)_param_cp_ver_gate      /**< Bypass Mode */
    )

    /**
     * Transforms the sensor orientation into a yaw in the local frame
     * @param distance_sensor, distance sensor message
     * @param angle_offset, sensor body frame offset
     */
    float _sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor,
                                        float angle_offset) const;

    /**
     * Computes collision free setpoints
     * @param setpoint, setpoint before collision prevention intervention
     * @param curr_pos, current vehicle position
     * @param curr_vel, current vehicle velocity
     */
    void _calculateConstrainedSetpoint(matrix::Vector2f &setpoint, const matrix::Vector2f &curr_pos,
                                       const matrix::Vector2f &curr_vel);

    /**
     * Publishes collision_constraints message
     * @param original_setpoint, setpoint before collision prevention intervention
     * @param adapted_setpoint, collision prevention adaped setpoint
     */
    void _publishConstrainedSetpoint(const matrix::Vector2f &original_setpoint,
                                     const matrix::Vector2f &adapted_setpoint);

    /**
     * Publishes obstacle_distance message with fused data from offboard and from distance sensors
     * @param obstacle, obstacle_distance message to be publsihed
     */
    void _publishObstacleDistance(obstacle_distance_s &obstacle);

    /**
     * Aggregates the sensor data into a internal obstacle map in body frame
     */
    void _updateObstacleMap();

    /**
     * Publishes vehicle command.
     */
    void _publishVehicleCmdDoLoiter();

    // lc add
    // BYPASS MODE
    enum BP_State
    {
        MANUAL,
        SLOWING_DOWN,
        AVOIDING,
        RECOVERING,
    };

    // 参数配置
    BP_State bp_state_ = MANUAL;
    matrix::Vector2f _original_setpoint_xy;        // 保存的用户原始指令
    float _original_setpoint_z;                    // 保存的用户原始指令
    hrt_abstime _recovery_start_xy;                // 恢复阶段开始时间
    matrix::Vector2f _last_avoidance_cmd;          // 上一次避障指令
    static constexpr double RECOVERY_TIME = 0.5f;  // 恢复时间 (s)
    bool BP_XY = false;
    bool BP_ZUP = false;

    // 避障核心逻辑
    void applyAvoidance(matrix::Vector2f &setpoint, float &setpointz);
    matrix::Vector2f _calculateAvoidanceCommand(bool xyorz);
    // bool _checkCollisionRisk(matrix::Vector2f& setpoint);
    matrix::Vector2f _getMinimumForwardDistance(const matrix::Vector2f &setpoint);
    float _getMinimumVerticalDistance();
    template <typename T>
    T calculateSlowdown(const T &original, float min_dist) const;
    matrix::Vector2f _blendCommands(const matrix::Vector2f &user_cmd,
                                    const matrix::Vector2f &avoid_cmd, float ratio);
};
