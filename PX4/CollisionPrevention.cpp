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
 * @file CollisionPrevention.cpp
 * CollisionPrevention controller.
 *
 */

#include "CollisionPrevention.hpp"

#include <px4_platform_common/events.h>
#include <uORB/topics/debug_key_value.h>
#include <uORB/topics/debug_vect.h>
// #include <vector>
// #include <algorithm>

using namespace matrix;
using namespace time_literals;

namespace
{
// lc add
// xy平面前后左右分辨率6 共60数据
// 72位数组 前六十存储前后左右，后十二存储上下
static constexpr int INTERNAL_MAP_UPDOWN_BLOCK = 12;
static constexpr int INTERNAL_MAP_INCREMENT_DEG =
    6;  // cannot be lower than 5 degrees, should divide 360 evenly
static constexpr int INTERNAL_MAP_USED_BINS = 360 / INTERNAL_MAP_INCREMENT_DEG;

static float wrap_360(float f) { return wrap(f, 0.f, 360.f); }

static int wrap_bin(int i)
{
    i = i % INTERNAL_MAP_USED_BINS;

    while (i < 0)
    {
        i += INTERNAL_MAP_USED_BINS;
    }

    return i;
}

}  // namespace

CollisionPrevention::CollisionPrevention(ModuleParams *parent) : ModuleParams(parent)
{
    static_assert(INTERNAL_MAP_INCREMENT_DEG >= 5,
                  "INTERNAL_MAP_INCREMENT_DEG needs to be at least 5");
    static_assert(360 % INTERNAL_MAP_INCREMENT_DEG == 0,
                  "INTERNAL_MAP_INCREMENT_DEG should divide 360 evenly");

    // initialize internal obstacle map
    _obstacle_map_body_frame.timestamp = getTime();
    _obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
    _obstacle_map_body_frame.increment = INTERNAL_MAP_INCREMENT_DEG;
    _obstacle_map_body_frame.min_distance = UINT16_MAX;
    _obstacle_map_body_frame.max_distance = 0;
    _obstacle_map_body_frame.angle_offset = 0.f;
    uint32_t internal_bins =
        sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0]);
    uint64_t current_time = getTime();

    for (uint32_t i = 0; i < internal_bins; i++)
    {
        _data_timestamps[i] = current_time;
        _data_maxranges[i] = 0;
        _data_fov[i] = 0;
        _obstacle_map_body_frame.distances[i] = UINT16_MAX;
    }
}

hrt_abstime CollisionPrevention::getTime() { return hrt_absolute_time(); }

hrt_abstime CollisionPrevention::getElapsedTime(const hrt_abstime *ptr)
{
    return hrt_absolute_time() - *ptr;
}

bool CollisionPrevention::is_active()
{
    bool activated = _param_cp_dist.get() > 0;

    if (activated && !_was_active)
    {
        _time_activated = getTime();
    }

    _was_active = activated;
    return activated;
}

void CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle,
                                                 const matrix::Quatf &vehicle_attitude)
{
    int msg_index = 0;
    float vehicle_orientation_deg = math::degrees(Eulerf(vehicle_attitude).psi());
    float increment_factor = 1.f / obstacle.increment;

    if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL ||
        obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED)
    {
        // Obstacle message arrives in local_origin frame (north aligned)
        // corresponding data index (convert to world frame and shift by msg offset)
        for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++)
        {
            float bin_angle_deg =
                (float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset;
            msg_index =
                ceil(wrap_360(vehicle_orientation_deg + bin_angle_deg - obstacle.angle_offset) *
                     increment_factor);
            // lc add 确保数组不越界
            msg_index %= INTERNAL_MAP_USED_BINS;

            // add all data points inside to FOV
            if (obstacle.distances[msg_index] != UINT16_MAX)
            {
                if (_enterData(i, obstacle.max_distance * 0.01f,
                               obstacle.distances[msg_index] * 0.01f))
                {
                    _obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
                    _data_timestamps[i] = _obstacle_map_body_frame.timestamp;
                    _data_maxranges[i] = obstacle.max_distance;
                    _data_fov[i] = 1;
                }
            }
        }

        // lc add
        // todo transform to body frame
        // 前6位为下方数据 后6位为上方数据
        for (int i = INTERNAL_MAP_USED_BINS; i < INTERNAL_MAP_USED_BINS + INTERNAL_MAP_UPDOWN_BLOCK;
             i++)
        {
            _obstacle_map_body_frame.distances[i] = obstacle.distances[i];
        }
    }
    else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD)
    {
        // Obstacle message arrives in body frame (front aligned)
        // corresponding data index (shift by msg offset)
        for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++)
        {
            float bin_angle_deg =
                (float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset;
            msg_index = ceil(wrap_360(bin_angle_deg - obstacle.angle_offset) * increment_factor);

            // add all data points inside to FOV
            if (obstacle.distances[msg_index] != UINT16_MAX)
            {
                if (_enterData(i, obstacle.max_distance * 0.01f,
                               obstacle.distances[msg_index] * 0.01f))
                {
                    _obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
                    _data_timestamps[i] = _obstacle_map_body_frame.timestamp;
                    _data_maxranges[i] = obstacle.max_distance;
                    _data_fov[i] = 1;
                }
            }
        }
    }
    else
    {
        mavlink_log_critical(&_mavlink_log_pub,
                             "Obstacle message received in unsupported frame %i\t", obstacle.frame);
        events::send<uint8_t>(events::ID("col_prev_unsup_frame"), events::Log::Error,
                              "Obstacle message received in unsupported frame {1}", obstacle.frame);
    }
}

bool CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
    // use data from this sensor if:
    // 1. this sensor data is in range, the bin contains already valid data and this data is coming
    // from the same or less range sensor
    // 2. this sensor data is in range, and the last reading was out of range
    // 3. this sensor data is out of range, the last reading was as well and this is the sensor with
    // longest range
    // 4. this sensor data is out of range, the last reading was valid and coming from the same
    // sensor

    uint16_t sensor_range_cm = static_cast<uint16_t>(100.0f * sensor_range + 0.5f);  // convert to
                                                                                     // cm

    if (sensor_reading < sensor_range)
    {
        if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index] &&
             sensor_range_cm <= _data_maxranges[map_index]) ||
            _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index])
        {
            return true;
        }
    }
    else
    {
        if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index] &&
             sensor_range_cm >= _data_maxranges[map_index]) ||
            (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index] &&
             sensor_range_cm == _data_maxranges[map_index]))
        {
            return true;
        }
    }

    return false;
}

void CollisionPrevention::_updateObstacleMap()
{
    _sub_vehicle_attitude.update();

    // add distance sensor data
    for (auto &dist_sens_sub : _distance_sensor_subs)
    {
        distance_sensor_s distance_sensor;

        if (dist_sens_sub.update(&distance_sensor))
        {
            // consider only instances with valid data and orientations useful for collision
            // prevention
            if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
                (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
                (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING))
            {
                // update message description
                _obstacle_map_body_frame.timestamp =
                    math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
                _obstacle_map_body_frame.max_distance =
                    math::max(_obstacle_map_body_frame.max_distance,
                              (uint16_t)(distance_sensor.max_distance * 100.0f));
                _obstacle_map_body_frame.min_distance =
                    math::min(_obstacle_map_body_frame.min_distance,
                              (uint16_t)(distance_sensor.min_distance * 100.0f));

                _addDistanceSensorData(distance_sensor, Quatf(_sub_vehicle_attitude.get().q));
            }
        }
    }

    // add obstacle distance data
    if (_sub_obstacle_distance.update())
    {
        const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

        // Update map with obstacle data if the data is not stale
        if (getElapsedTime(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US &&
            obstacle_distance.increment > 0.f)
        {
            // update message description
            _obstacle_map_body_frame.timestamp =
                math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
            _obstacle_map_body_frame.max_distance =
                math::max(_obstacle_map_body_frame.max_distance, obstacle_distance.max_distance);
            _obstacle_map_body_frame.min_distance =
                math::min(_obstacle_map_body_frame.min_distance, obstacle_distance.min_distance);
            _addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
        }
    }

    // publish fused obtacle distance message with data from offboard obstacle_distance and distance
    // sensor
    _obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor,
                                                 const matrix::Quatf &vehicle_attitude)
{
    // clamp at maximum sensor range
    float distance_reading =
        math::min(distance_sensor.current_distance, distance_sensor.max_distance);

    // discard values below min range
    if ((distance_reading > distance_sensor.min_distance))
    {
        float sensor_yaw_body_rad =
            _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
        float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

        // calculate the field of view boundary bin indices
        int lower_bound =
            (int)floor((sensor_yaw_body_deg - math::degrees(distance_sensor.h_fov / 2.0f)) /
                       INTERNAL_MAP_INCREMENT_DEG);
        int upper_bound =
            (int)floor((sensor_yaw_body_deg + math::degrees(distance_sensor.h_fov / 2.0f)) /
                       INTERNAL_MAP_INCREMENT_DEG);

        // floor values above zero, ceil values below zero
        if (lower_bound < 0)
        {
            lower_bound++;
        }

        if (upper_bound < 0)
        {
            upper_bound++;
        }

        // rotate vehicle attitude into the sensor body frame
        matrix::Quatf attitude_sensor_frame = vehicle_attitude;
        attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
        float sensor_dist_scale = cosf(Eulerf(attitude_sensor_frame).theta());

        if (distance_reading < distance_sensor.max_distance)
        {
            distance_reading = distance_reading * sensor_dist_scale;
        }

        uint16_t sensor_range =
            static_cast<uint16_t>(100.0f * distance_sensor.max_distance + 0.5f);  // convert to cm

        for (int bin = lower_bound; bin <= upper_bound; ++bin)
        {
            int wrapped_bin = wrap_bin(bin);

            if (_enterData(wrapped_bin, distance_sensor.max_distance, distance_reading))
            {
                _obstacle_map_body_frame.distances[wrapped_bin] =
                    static_cast<uint16_t>(100.0f * distance_reading + 0.5f);
                _data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
                _data_maxranges[wrapped_bin] = sensor_range;
                _data_fov[wrapped_bin] = 1;
            }
        }
    }
}

void CollisionPrevention::_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index,
                                                  float vehicle_yaw_angle_rad)
{
    const float col_prev_d = _param_cp_dist.get();
    const int guidance_bins = floor(_param_cp_guide_ang.get() / INTERNAL_MAP_INCREMENT_DEG);
    const int sp_index_original = setpoint_index;
    float best_cost = 9999.f;
    int new_sp_index = setpoint_index;

    for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++)
    {
        // apply moving average filter to the distance array to be able to center in larger gaps
        const int filter_size = 1;
        float mean_dist = 0;

        for (int j = i - filter_size; j <= i + filter_size; j++)
        {
            int bin = wrap_bin(j);

            if (_obstacle_map_body_frame.distances[bin] == UINT16_MAX)
            {
                mean_dist += col_prev_d * 100.f;
            }
            else
            {
                mean_dist += _obstacle_map_body_frame.distances[bin];
            }
        }

        const int bin = wrap_bin(i);
        mean_dist = mean_dist / (2.f * filter_size + 1.f);
        const float deviation_cost = col_prev_d * 50.f * abs(i - sp_index_original);
        const float bin_cost = deviation_cost - mean_dist - _obstacle_map_body_frame.distances[bin];

        if (bin_cost < best_cost && _obstacle_map_body_frame.distances[bin] != UINT16_MAX)
        {
            best_cost = bin_cost;
            new_sp_index = bin;
        }
    }

    // only change setpoint direction if it was moved to a different bin
    if (new_sp_index != setpoint_index)
    {
        float angle = math::radians((float)new_sp_index * INTERNAL_MAP_INCREMENT_DEG +
                                    _obstacle_map_body_frame.angle_offset);
        angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
        setpoint_dir = {cosf(angle), sinf(angle)};
        setpoint_index = new_sp_index;
    }
}

float CollisionPrevention::_sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor,
                                                         float angle_offset) const
{
    float offset = angle_offset > 0.0f ? math::radians(angle_offset) : 0.0f;

    switch (distance_sensor.orientation)
    {
        case distance_sensor_s::ROTATION_YAW_0:
            offset = 0.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_45:
            offset = M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_90:
            offset = M_PI_F / 2.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_135:
            offset = 3.0f * M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_180:
            offset = M_PI_F;
            break;

        case distance_sensor_s::ROTATION_YAW_225:
            offset = -3.0f * M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_270:
            offset = -M_PI_F / 2.0f;
            break;

        case distance_sensor_s::ROTATION_YAW_315:
            offset = -M_PI_F / 4.0f;
            break;

        case distance_sensor_s::ROTATION_CUSTOM:
            offset = matrix::Eulerf(matrix::Quatf(distance_sensor.q)).psi();
            break;
    }

    return offset;
}

// lc add
void CollisionPrevention::_ConstrainSetpoint_ZDown(float &setpointz, float stick)
{
    /* 参数说明:
     * stick > 0 向下运动 | stick < 0 向上运动 | 距离单位: 厘米/---
     * STOP_GAP_PHASE1: 第一阶段警戒距离 //m
     * STOP_GAP_PHASE2: 第二阶段危险距离 //m
     * DECEL_EXPONENT:  指数衰减系数(值越大减速越剧烈)
     */
    // static constexpr float STOP_GAP_PHASE1 = 250.0f;
    // static constexpr float STOP_GAP_PHASE2 = 120.0f;
    // static constexpr float DECEL_EXPONENT = 5.0f;  // 指数衰减强度系数
    const float STOP_GAP_PHASE1 = _param_cp_down_gate1.get();
    const float STOP_GAP_PHASE2 = _param_cp_down_gate2.get();
    const float DECEL_EXPONENT = _param_cp_down_decay.get();  // 指数衰减强度系数
    static constexpr int CLIP_MAX_PHASE1 = 100;               // 第一阶段需持续推动次数
    static constexpr int CLIP_MAX_PHASE2 = 150;               // 第二阶段需持续推动次数

    // 状态变量
    static int phase1_counter = 0;  // 第一阶段操作计数器
    static int phase2_counter = 0;  // 第二阶段操作计数器

    // 初始化最小距离为最大可测距离
    float min_dist = _obstacle_map_body_frame.max_distance * 0.01f;
    const hrt_abstime current_time = getTime();

    // 公共处理逻辑
    auto handlePhase = [&](int &counter, int counter_max, float stop_gap)
    {
        if (stick > 0.0f)
        {  // 仅处理向下运动
            if (counter <= counter_max)
            {
                // 安全模式：强制停止并累积操作计数
                setpointz = 0.0f;
                // 摇杆强度>50%时累积，否则重置
                counter += (stick > 0.5f) ? 1 : -counter;
            }
            else
            {
                // 解除限制后：根据距离指数衰减速度
                // setpointz = stick * expf(-DECEL_EXPONENT * math::constrain((stop_gap - min_dist)
                // / stop_gap, 0.0f, 1.0f));
                float distance_ratio = (stop_gap - min_dist) / stop_gap;  // 危险程度[0,1]
                distance_ratio = math::constrain(distance_ratio, 0.0f, 1.0f);
                float speed_decay = expf(-DECEL_EXPONENT * distance_ratio);  // 指数衰减
                setpointz = stick * speed_decay;                             // 应用衰减后的速度
            }
        }
        else
        {  // 摇杆释放时重置状态
            setpointz = stick;
            counter = 0;
        }
    };

    // 更新障碍物地图数据
    _updateObstacleMap();

    // 检查传感器数据有效性(300ms超时)
    if ((current_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US)
    {
        // 遍历指定区域寻找最小障碍物距离
        for (int i = INTERNAL_MAP_USED_BINS;
             i < INTERNAL_MAP_USED_BINS + (INTERNAL_MAP_UPDOWN_BLOCK / 2); ++i)
        {
            if (_obstacle_map_body_frame.distances[i] * 0.01f < min_dist)
                min_dist = _obstacle_map_body_frame.distances[i] * 0.01f;
        }

        // 分级安全处理
        if (min_dist < STOP_GAP_PHASE2)
        {  // 进入危险距离
            handlePhase(phase2_counter, CLIP_MAX_PHASE2, STOP_GAP_PHASE2);
            phase1_counter = 0;  // 重置第一阶段计数器
        }
        else if (min_dist < STOP_GAP_PHASE1)
        {  // 进入警戒距离
            handlePhase(phase1_counter, CLIP_MAX_PHASE1, STOP_GAP_PHASE1);
            phase2_counter = 0;  // 重置第二阶段计数器
        }
        else
        {  // 安全距离内
            setpointz = stick;
            phase1_counter = phase2_counter = 0;
        }
    }
    else
    {  // 传感器数据超时处理
        setpointz = 0.0f;
        phase1_counter = phase2_counter = 0;
    }

    // 调试数据发布
    // struct debug_key_value_s dbg{};
    // strncpy(dbg.key, "min_dist", sizeof(dbg.key));
    // dbg.value = min_dist;
    // dbg.timestamp = current_time;
    // orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
    // orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
}

void CollisionPrevention::_ConstrainSetpoint_ZUp(float &setpointz, float stick)
{
    // static constexpr float DECEL_EXPONENT = 4.0f;  // 指数衰减强度系数
    // const constexpr float stop_gap = 200.0f;
    // const constexpr float slow_gap = 350.0f;
    const float slow_gap = _param_cp_up_gate1.get();
    const float stop_gap = _param_cp_up_gate2.get();
    const float DECEL_EXPONENT = _param_cp_up_decay.get();  // 指数衰减强度系数
    const hrt_abstime current_time = getTime();
    // 初始化最小距离为最大可测距离
    float min_dist = _obstacle_map_body_frame.max_distance * 0.01f;

    // 更新障碍物地图数据
    _updateObstacleMap();

    // 检查传感器数据有效性(300ms超时)
    if ((current_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US)
    {
        if (stick < 0.0f)
        {
            // 遍历指定区域寻找最小障碍物距离
            for (int i = INTERNAL_MAP_USED_BINS + (INTERNAL_MAP_UPDOWN_BLOCK / 2);
                 i < INTERNAL_MAP_USED_BINS + INTERNAL_MAP_UPDOWN_BLOCK; ++i)
            {
                if (_obstacle_map_body_frame.distances[i] * 0.01f < min_dist)
                    min_dist = _obstacle_map_body_frame.distances[i] * 0.01f;
            }

            if (min_dist < stop_gap)
                setpointz = 0.0f;

            else if (min_dist < slow_gap)
            {
                // 计算减速系数：越接近停止距离，衰减越强
                const float decel_range = slow_gap - stop_gap;
                const float distance_ratio = (min_dist - stop_gap) / decel_range;

                // 使用三次曲线实现平滑过渡（可替换为其他缓动函数）
                const float eased_ratio =
                    distance_ratio * distance_ratio * (3.0f - 2.0f * distance_ratio);
                const float speed_decay = expf(-DECEL_EXPONENT * (1.0f - eased_ratio));

                setpointz = stick * speed_decay;
            }
            else
                return;
        }

        else
            return;
    }

    else
    {
        setpointz = 0.0f;
    }
}

// xy平面刹停逻辑
void CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint,
                                                        const Vector2f &curr_pos,
                                                        const Vector2f &curr_vel)
{
    // 更新obstacle_distance
    _updateObstacleMap();

    // read parameters
    // 最小安全距离
    const float col_prev_d = _param_cp_dist.get();
    // 距离延时时间 附加
    const float col_prev_dly = _param_cp_delay.get();
    // 无数据时是否允许运动
    const bool move_no_data = _param_cp_go_nodata.get();
    // 运动学参数获取
    const float xy_p = _param_mpc_xy_p.get();
    const float max_jerk = _param_mpc_jerk_max.get();
    const float max_accel = _param_mpc_acc_hor.get();
    const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
    const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

    // 设定点模长
    const float setpoint_length = setpoint.norm();

    const hrt_abstime constrain_time = getTime();
    // 记录有效扇区数据个数
    int num_fov_bins = 0;

    // 调试数据发布
    // static orb_advert_t dbg_angle_pub = nullptr;

    // 数据有效 进入避障逻辑
    if ((constrain_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US)
    {
        // 用户已打杆
        if (setpoint_length > 0.001f)
        {
            // lc add 检测范围正负18度
            const float rad_threshold = math::radians(18.0f);
            // xy平面 setpoint_dir设定值归一化 归一化为单位向量
            Vector2f setpoint_dir = setpoint / setpoint_length;
            // 最大速度值 设置为模长
            float vel_max = setpoint_length;
            // 最小安全距离
            const float min_dist_to_keep =
                math::max(_obstacle_map_body_frame.min_distance / 100.0f, col_prev_d);

            // lc add 世界系设定弧度
            const float sp_rad_local = wrap_2pi((atan2f(setpoint_dir(1), setpoint_dir(0))));

            // atan2f(setpoint_dir(1), setpoint_dir(0))为世界系下设定速度 减去机体yaw角得到
            // 机体系下设定速度方向
            const float sp_angle_body_frame =
                atan2f(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw_angle_rad;
            // 缩放到0-360
            const float sp_angle_with_offset_deg = wrap_360(math::degrees(sp_angle_body_frame) -
                                                            _obstacle_map_body_frame.angle_offset);
            // 设定值索引
            int sp_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);

            //_obstacle_map_body_frame.distances[i]注意障碍物信息数组 为机体系下

            // change setpoint direction slightly (max by _param_cp_guide_ang degrees) to help guide
            // through narrow gaps 角度允许情况下实现绕飞
            //_adaptSetpointDirection(setpoint_dir, sp_index, vehicle_yaw_angle_rad);

            // limit speed for safe flight
            // 遍历每一个扇区
            for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++)
            {  // disregard unused bins at the end of the message

                // delete stale values
                // 保留计算数据时间
                const hrt_abstime data_age = constrain_time - _data_timestamps[i];

                // 超时则将所有数据置换为无效
                if (data_age > RANGE_STREAM_TIMEOUT_US)
                {
                    _obstacle_map_body_frame.distances[i] = UINT16_MAX;
                }

                // 转换单位为米 m
                const float distance =
                    _obstacle_map_body_frame.distances[i] * 0.01f;   // convert to meters
                const float max_range = _data_maxranges[i] * 0.01f;  // convert to meters

                // 当前循环中的扇区的对应弧度
                float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG +
                                            _obstacle_map_body_frame.angle_offset);

                // convert from body to local frame in the range [0, 2*pi]
                // 机体系下弧度 + 当前yaw角 转换为世界系下弧度
                angle = wrap_2pi(vehicle_yaw_angle_rad + angle);

                // struct debug_key_value_s dbg_angle{};
                // strncpy(dbg_angle.key, "angle", sizeof(dbg_angle.key));
                // dbg_angle.value = angle;
                // dbg_angle.timestamp = constrain_time;

                // if (dbg_angle_pub == nullptr) {
                // 	dbg_angle_pub = orb_advertise(ORB_ID(debug_key_value), &dbg_angle);
                // } else {
                // 	orb_publish(ORB_ID(debug_key_value), dbg_angle_pub, &dbg_angle);
                // }

                // get direction of current bin
                // 当前扇区值角度转换为xy方向向量
                // 转换为世界系 统一后方便与setpoint_dir比较
                const Vector2f bin_direction = {cosf(angle), sinf(angle)};

                // count number of bins in the field of valid_new
                // 如果数据有效 记录有效扇区数据个数
                if (_obstacle_map_body_frame.distances[i] < UINT16_MAX)
                {
                    num_fov_bins++;
                }

                // 数据有效
                if (_obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance &&
                    _obstacle_map_body_frame.distances[i] < UINT16_MAX)
                {
                    // 不在检测范围内 直接跳过循环
                    float rad_diff = wrap_pi(angle - sp_rad_local);
                    if (fabsf(rad_diff) > rad_threshold)
                    {
                        continue;
                    }

                    // 在范围内时进行避障逻辑

                    // calculate max allowed velocity with a P-controller (same gain as in the
                    // position controller)
                    // 当前速度在该方向的投影
                    const float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));
                    // 延迟导致的位移
                    float delay_distance = curr_vel_parallel * col_prev_dly;

                    if (distance < max_range)
                    {
                        // 数据老化补偿
                        delay_distance += curr_vel_parallel * (data_age * 1e-6f);
                    }

                    // 安全距离为识别到的障碍距离-最小安全距离-延迟位移
                    // 若stop_distance为0 则输出vel_max_posctrl直接为0
                    const float stop_distance =
                        math::max(0.f, distance - min_dist_to_keep - delay_distance);
                    // 位置控制器速度限制；距离越近 速度被限制得越小
                    const float vel_max_posctrl = xy_p * stop_distance;

                    // 调用函数光滑化目标设定值  运动学平滑限制
                    const float vel_max_smooth = math::trajectory::computeMaxSpeedFromDistance(
                        max_jerk, max_accel, stop_distance, 0.f);
                    // 投影值（projection）越大（方向越一致）d
                    const float projection = bin_direction.dot(setpoint_dir);
                    float vel_max_bin = vel_max;

                    if (projection > 0.01f)
                    {
                        // 综合限制  projection越大方向越一致，即分母越大，vel_max_bin
                        // 越小，速度限制越严格
                        // 结合位置控制器（响应快）和运动学模型（平滑性）生成速度限制
                        vel_max_bin = math::min(vel_max_posctrl, vel_max_smooth) / projection;
                    }

                    // constrain the velocity
                    if (vel_max_bin >= 0)
                    {
                        vel_max = math::min(vel_max, vel_max_bin);
                    }

                    // 如果目标位置没有传感器数据覆盖，则根据move_no_data决定是否冻结速度
                }
                else if (_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == sp_index)
                {
                    if (!move_no_data || (move_no_data && _data_fov[i]))
                    {
                        vel_max = 0.f;
                    }
                }
            }

            // if the sensor field of view is zero, never allow to move (even if move_no_data=1)
            // 如果传感器完全失去有效数据，即一个有效扇区数据都没有
            if (num_fov_bins == 0)
            {
                vel_max = 0.f;
            }
            // 调整后最终值为单位方向向量*速度
            // 该函数主要通过遍历并计算邻近扇区（点乘>0）的障碍物信息来调整速度值
            //  调整方向主要在_adaptSetpointDirection 函数中
            setpoint = setpoint_dir * vel_max;
        }
    }
    else
    {
        // 如果传感器数据超时 则禁止机动
        // allow no movement
        float vel_max = 0.f;
        setpoint = setpoint * vel_max;

        // if distance data is stale, switch to Loiter
        // 当数据丢失超过一定阈值后 传感器可能发生故障 触发悬停
        if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s)
        {
            if ((constrain_time - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US &&
                getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US)
            {
                // 发布进入悬停模式
                _publishVehicleCmdDoLoiter();
            }

            _last_timeout_warning = getTime();
        }
    }
}

void CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed,
                                         const Vector2f &curr_pos, const Vector2f &curr_vel,
                                         float &setpointz)
{
    const bool cp_mode = _param_cp_mode.get();
    // calculate movement constraints based on range data
    Vector2f new_setpoint = original_setpoint;

    if (!cp_mode)
    {
        _calculateConstrainedSetpoint(new_setpoint, curr_pos, curr_vel);
        float sp_zd = setpointz;
        _ConstrainSetpoint_ZDown(setpointz, sp_zd);
        float sp_zu = setpointz;
        _ConstrainSetpoint_ZUp(setpointz, sp_zu);
    }

    else
    {
        applyAvoidance(new_setpoint, setpointz);
    }

    // warn user if collision prevention starts to interfere
    bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed ||
                                  new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed ||
                                  new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed ||
                                  new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

    _interfering = currently_interfering;

    // publish constraints
    collision_constraints_s constraints{};
    constraints.timestamp = getTime();
    original_setpoint.copyTo(constraints.original_setpoint);
    new_setpoint.copyTo(constraints.adapted_setpoint);
    _constraints_pub.publish(constraints);

    original_setpoint = new_setpoint;
}

void CollisionPrevention::_publishVehicleCmdDoLoiter()
{
    vehicle_command_s command{};
    command.timestamp = getTime();
    command.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    command.param1 = (float)1;  // base mode
    command.param3 = (float)0;  // sub mode
    command.target_system = 1;
    command.target_component = 1;
    command.source_system = 1;
    command.source_component = 1;
    command.confirmation = false;
    command.from_external = false;
    command.param2 = (float)PX4_CUSTOM_MAIN_MODE_AUTO;
    command.param3 = (float)PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

    // publish the vehicle command
    _vehicle_command_pub.publish(command);
}

void CollisionPrevention::applyAvoidance(Vector2f &setpoint_xy, float &setpointz)
{
    // const bool ooa_mode = _param_cp_mode.get();
    const float DECEL_DIS = _param_cp_decel_dis.get();
    const float BP_DIS = _param_cp_bypass_dis.get();
    // const float VER_GATE = _param_cp_ver_gate.get();
    // const float HOR_DENSE = _param_cp_hor_dense.get();
    // 水平拖动向上下避障
    // bool vertical_bypass = false;

    // 用户未打杆时 不检测
    if (setpoint_xy.norm() < 0.05f && setpointz < 0.1f && setpointz > -0.1f)
        return;

    // 更新障碍物距离数据
    _updateObstacleMap();

    if (setpoint_xy.norm() > 0.05f)
    {
        BP_XY = true;
        // 初始速度设定值
        _original_setpoint_xy = setpoint_xy;
    }
    else
    {
        BP_XY = false;
    }

    if (setpointz < -0.1f)
    {
        // 初始速度设定值
        _original_setpoint_z = setpointz;
        BP_ZUP = true;
    }
    else if (setpointz > 0.1f)
    {
        // 下方障碍物 不绕行 参考刹停时处理逻辑
        BP_ZUP = false;
        // 初始速度设定值
        _original_setpoint_z = setpointz;
        _ConstrainSetpoint_ZDown(setpointz, _original_setpoint_z);
    }
    else
    {
        BP_ZUP = false;
    }

    // 获取垂直方向 上 的最小障碍物距离
    float min_verup_dist = _getMinimumVerticalDistance();
    Vector2f dist = _getMinimumForwardDistance(setpoint_xy);
    // 获取目标范围内(+-24 degree)最小障碍物距离
    float min_hor_dist = dist(0);
    // 水平全向平均距离
    // float hor_aver_dist = dist(1);

    // if(min_verup_dist < 1.0f || min_hor_dist < 1.0f){
    // 	setpointz = 0.0f;
    // 	setpoint_xy = Vector2f(0.0f, 0.0f);
    // 	return;
    // }

    // 状态切换逻辑
    switch (bp_state_)
    {
            // 纯摇杆映射
        case MANUAL:
            if (BP_XY == true)
            {
                // 障碍物距离未触发绕行 到达减速阈值 切换到减速状态
                if (min_hor_dist < DECEL_DIS && min_hor_dist >= BP_DIS)
                {
                    bp_state_ = SLOWING_DOWN;
                    // 可能是移动yaw 障碍物距离到达绕行阈值 切换到绕行状态
                }
                else if (min_hor_dist < BP_DIS)
                {
                    bp_state_ = AVOIDING;
                }
            }

            if (BP_ZUP == true)
            {
                // 障碍物距离未触发绕行 到达减速阈值 切换到减速状态
                if (min_verup_dist < DECEL_DIS && min_verup_dist >= BP_DIS)
                {
                    bp_state_ = SLOWING_DOWN;
                }
                else if (min_verup_dist < BP_DIS)
                {
                    bp_state_ = AVOIDING;
                }
            }
            break;

        case SLOWING_DOWN:
            if (BP_XY == true)
            {
                // 判读为安全 切换回摇杆映射模式
                if (min_hor_dist >= DECEL_DIS)
                {
                    bp_state_ = MANUAL;
                    // 减速后达到绕行阈值后切换到绕行状态
                }
                else if (min_hor_dist < BP_DIS)
                {
                    bp_state_ = AVOIDING;
                }
            }

            if (BP_ZUP == true)
            {
                // 判读为安全 切换回摇杆映射模式
                if (min_verup_dist >= DECEL_DIS)
                {
                    bp_state_ = MANUAL;
                }
                else if (min_verup_dist < BP_DIS)
                {
                    bp_state_ = AVOIDING;
                }
            }
            break;

        case AVOIDING:
            if (BP_XY == true)
            {
                // 绕行完成 切换到恢复状态
                if (min_hor_dist >= BP_DIS && !BP_ZUP)
                {
                    bp_state_ = RECOVERING;
                    _recovery_start_xy = hrt_absolute_time();
                }
            }

            if (BP_ZUP == true)
            {
                if (min_verup_dist >= DECEL_DIS)
                {
                    bp_state_ = MANUAL;
                }
            }

            break;

        case RECOVERING:
            if (BP_XY == true)
            {
                // 恢复完成 切换回摇杆映射模式
                if (min_hor_dist >= DECEL_DIS)
                {
                    if (hrt_elapsed_time(&_recovery_start_xy) > RECOVERY_TIME * 1e6)
                    {
                        bp_state_ = MANUAL;
                    }
                }
                // 若突然又识别到障碍物 切换到绕行状态
                else if (min_hor_dist < DECEL_DIS && min_hor_dist > BP_DIS)
                {
                    bp_state_ = SLOWING_DOWN;
                }

                else if (min_hor_dist <= BP_DIS)
                {
                    bp_state_ = AVOIDING;
                }
            }

            if (BP_ZUP == true)
            {
                if (min_verup_dist < DECEL_DIS && min_verup_dist > BP_DIS)
                    bp_state_ = SLOWING_DOWN;

                if (min_verup_dist <= BP_DIS)
                    bp_state_ = AVOIDING;
            }

            break;
    }

    // 输出控制逻辑
    switch (bp_state_)
    {
        case MANUAL:
            // 原样输出
            break;

        case SLOWING_DOWN:
            if (BP_XY == true)
            {
                // 减速平滑输出
                setpoint_xy = calculateSlowdown(_original_setpoint_xy, min_hor_dist);
            }
            if (BP_ZUP == true)
            {
                setpointz = calculateSlowdown(_original_setpoint_z, min_verup_dist);
            }
            break;

        case AVOIDING:
            if (BP_XY == true)
            {
                // 绕行输出 保存最后一次绕行命令 以备恢复使用
                setpoint_xy = _calculateAvoidanceCommand(true);
                _last_avoidance_cmd = setpoint_xy;
            }
            if (BP_ZUP == true)
            {
                setpointz = -0.0f;
                setpoint_xy = _calculateAvoidanceCommand(false);
            }

            break;

        case RECOVERING:
            // 恢复输出 混合绕行命令和原命令 以平滑过渡
            float t = hrt_elapsed_time(&_recovery_start_xy) / (RECOVERY_TIME * 1e6);
            t = math::constrain(t, 0.0f, 1.0f);
            setpoint_xy = _blendCommands(_original_setpoint_xy, _last_avoidance_cmd, 1.0f - t);
            break;
    }

    // 调试使用
    static orb_advert_t dbg_vect_pub = nullptr;
    struct debug_vect_s dbg_vect{};
    strncpy(dbg_vect.name, "bp_state_", sizeof(dbg_vect.name));
    dbg_vect.timestamp = hrt_absolute_time();
    dbg_vect.x = bp_state_;
    dbg_vect.y = min_hor_dist;
    dbg_vect.z = BP_ZUP;

    if (dbg_vect_pub == nullptr)
    {
        dbg_vect_pub = orb_advertise(ORB_ID(debug_vect), &dbg_vect);
    }
    else
    {
        orb_publish(ORB_ID(debug_vect), dbg_vect_pub, &dbg_vect);
    }
}

Vector2f CollisionPrevention::_blendCommands(const Vector2f &user_cmd, const Vector2f &avoid_cmd,
                                             float ratio)
{
    // 使用三次贝塞尔曲线平滑过渡
    float t = ratio * ratio * (3.0f - 2.0f * ratio);
    return user_cmd * (1.0f - t) + avoid_cmd * t;
}

template <typename T>
T CollisionPrevention::calculateSlowdown(const T &original, float min_dist) const
{
    const float MIN_RATIO = 0.3f;
    const float DECEL_DIS = _param_cp_decel_dis.get();
    const float BP_DIS = _param_cp_bypass_dis.get();
    float ratio = math::constrain((min_dist - BP_DIS) / (DECEL_DIS - BP_DIS), MIN_RATIO, 1.0f);
    return original * ratio;
}

Vector2f CollisionPrevention::_getMinimumForwardDistance(const Vector2f &setpoint)
{
    const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
    const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();
    // 前进方向检测
    const float forward_threshold = math::radians(24.0f);
    // 上下绕飞时全向检测
    const float around_threshold = math::radians(180.0f);
    const float setpoint_length = setpoint.norm();
    // 计算水平方向障碍物密度
    float aver_dist = 0.f;
    int use_bins = 0;

    // xy平面未打杆时 不检测
    // if (setpoint_length < 0.01f) return Vector2f{INFINITY, INFINITY};

    Vector2f setpoint_dir = setpoint / setpoint_length;
    const float sp_rad_local = wrap_2pi(atan2f(setpoint_dir(1), setpoint_dir(0)));

    float min_dist = INFINITY;

    for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++)
    {
        float raw_dist = _obstacle_map_body_frame.distances[i];
        if (raw_dist <= _obstacle_map_body_frame.min_distance || raw_dist >= UINT16_MAX)
            continue;

        float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG +
                                    _obstacle_map_body_frame.angle_offset);
        angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
        float rad_diff = wrap_pi(angle - sp_rad_local);

        // 障碍物密度计算只评估指定运动方向的前180度
        if (fabsf(rad_diff) > around_threshold)
            continue;
        aver_dist += raw_dist * 0.01f;
        use_bins++;

        // 跳过偏离目标方向太多角度的扇区
        // 计算运动方向上最短距离，不需要遍历过多
        if (fabsf(rad_diff) > forward_threshold)
            continue;

        float dist_m = raw_dist * 0.01f;
        if (dist_m < min_dist)
            min_dist = dist_m;
    }

    aver_dist /= use_bins;
    return Vector2f(min_dist, aver_dist);
}

float CollisionPrevention::_getMinimumVerticalDistance()
{
    float min_up_dist = INFINITY;
    // float min_down_dist = INFINITY;

    // 遍历指定区域寻找最小障碍物距离 down
    // for(int i = INTERNAL_MAP_USED_BINS; i < INTERNAL_MAP_USED_BINS + (INTERNAL_MAP_UPDOWN_BLOCK /
    // 2); ++i){ 	if(_obstacle_map_body_frame.distances[i] * 0.01f < min_down_dist) min_down_dist
    // = _obstacle_map_body_frame.distances[i] * 0.01f;
    // }
    // 遍历指定区域寻找最小障碍物距离 up
    for (int i = INTERNAL_MAP_USED_BINS + (INTERNAL_MAP_UPDOWN_BLOCK / 2);
         i < INTERNAL_MAP_USED_BINS + INTERNAL_MAP_UPDOWN_BLOCK; ++i)
    {
        if (_obstacle_map_body_frame.distances[i] * 0.01f < min_up_dist)
            min_up_dist = _obstacle_map_body_frame.distances[i] * 0.01f;
    }

    return min_up_dist;
}

Vector2f CollisionPrevention::_calculateAvoidanceCommand(bool xyorz)
{
    // const float DECEL_DIS = _param_cp_decel_dis.get();
    const float BP_DIS = _param_cp_bypass_dis.get();
    const float MAX_AVOID_SPEED = _param_cp_bypass_vel.get();
    // 计算距离评分时考虑临近扇区 避免绕行时过于贴近障碍物
    const int NEIGHBOR_BINS = _param_cp_nei_bins.get();
    const float ALIGNMENT_GAIN = _param_cp_align_gain.get();
    const float DISTANCE_GAIN = _param_cp_dis_gain.get();
    Vector2f best_direction(0, 0);
    float max_safety = -INFINITY;
    const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
    const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();
    const float SAFE_DISTANCE = _obstacle_map_body_frame.max_distance * 0.01f;

    const float sp_rad_local =
        xyorz ? wrap_2pi(atan2f(_original_setpoint_xy(1), _original_setpoint_xy(0)))
              : math::radians(90.0f);

    const float angle_limit_sp = xyorz ? 90.0f : 180.0f;

    // 遍历所有扇区寻找最安全方向
    for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++)
    {
        if (_obstacle_map_body_frame.distances[i] <= _obstacle_map_body_frame.min_distance ||
            _obstacle_map_body_frame.distances[i] >= UINT16_MAX)
            continue;
        // 当前循环中的扇区的对应弧度

        float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG +
                                    _obstacle_map_body_frame.angle_offset);

        // convert from body to local frame in the range [0, 2*pi]
        // 机体系下弧度 + 当前yaw角 转换为世界系下弧度
        angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
        float rad_diff = wrap_pi(angle - sp_rad_local);

        // 不考虑往目标方向的反方向绕行  此情况全向均需要考虑
        if (fabsf(rad_diff) > math::radians(angle_limit_sp))
            continue;

        float total_dist = 0.f;

        // 计算临近扇区的平均距离
        for (int j = -NEIGHBOR_BINS; j <= NEIGHBOR_BINS; ++j)
        {
            int index = (i + j + INTERNAL_MAP_USED_BINS) % INTERNAL_MAP_USED_BINS;

            float dist = _obstacle_map_body_frame.distances[index] * 0.01f;
            if (dist > BP_DIS)
            {
                total_dist += dist;
            }
            // 如果有障碍物距离小于BP_DIS 则跳出循环 保证安全
            else
            {
                total_dist = -INFINITY;
                break;
            }
        }

        // 安全评分 = 距离评分 + 方向对齐评分
        float distance_score = total_dist / (2 * NEIGHBOR_BINS + 1) / SAFE_DISTANCE;  //[0,1]
        float alignment_score = 0.5f * (cosf(rad_diff) + 1.0f);                       // [0,1]

        float total_score = DISTANCE_GAIN * distance_score + ALIGNMENT_GAIN * alignment_score;

        if (total_score > max_safety)
        {
            max_safety = total_score;
            best_direction = Vector2f(cosf(angle), sinf(angle));
        }
    }

    return best_direction * MAX_AVOID_SPEED;
}
