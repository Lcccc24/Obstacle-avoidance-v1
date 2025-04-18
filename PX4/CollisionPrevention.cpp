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

using namespace matrix;
using namespace time_literals;

namespace
{
//lc add
//xy平面前后左右分辨率6 共60数据
//72位数组 前六十存储前后左右，后十二存储上下
static constexpr int INTERNAL_MAP_UPDOWN_BLOCK = 12;
static constexpr int INTERNAL_MAP_INCREMENT_DEG = 6; //cannot be lower than 5 degrees, should divide 360 evenly
static constexpr int INTERNAL_MAP_USED_BINS = 360 / INTERNAL_MAP_INCREMENT_DEG;

static float wrap_360(float f)
{
	return wrap(f, 0.f, 360.f);
}

static int wrap_bin(int i)
{
	i = i % INTERNAL_MAP_USED_BINS;

	while (i < 0) {
		i += INTERNAL_MAP_USED_BINS;
	}

	return i;
}

} // namespace

CollisionPrevention::CollisionPrevention(ModuleParams *parent) :
	ModuleParams(parent)
{
	static_assert(INTERNAL_MAP_INCREMENT_DEG >= 5, "INTERNAL_MAP_INCREMENT_DEG needs to be at least 5");
	static_assert(360 % INTERNAL_MAP_INCREMENT_DEG == 0, "INTERNAL_MAP_INCREMENT_DEG should divide 360 evenly");

	// initialize internal obstacle map
	_obstacle_map_body_frame.timestamp = getTime();
	_obstacle_map_body_frame.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_map_body_frame.increment = INTERNAL_MAP_INCREMENT_DEG;
	_obstacle_map_body_frame.min_distance = UINT16_MAX;
	_obstacle_map_body_frame.max_distance = 0;
	_obstacle_map_body_frame.angle_offset = 0.f;
	uint32_t internal_bins = sizeof(_obstacle_map_body_frame.distances) / sizeof(_obstacle_map_body_frame.distances[0]);
	uint64_t current_time = getTime();

	for (uint32_t i = 0 ; i < internal_bins; i++) {
		_data_timestamps[i] = current_time;
		_data_maxranges[i] = 0;
		_data_fov[i] = 0;
		_obstacle_map_body_frame.distances[i] = UINT16_MAX;
	}
}

hrt_abstime CollisionPrevention::getTime()
{
	return hrt_absolute_time();
}

hrt_abstime CollisionPrevention::getElapsedTime(const hrt_abstime *ptr)
{
	return hrt_absolute_time() - *ptr;
}

bool CollisionPrevention::is_active()
{
	bool activated = _param_cp_dist.get() > 0;

	if (activated && !_was_active) {
		_time_activated = getTime();
	}

	_was_active = activated;
	return activated;
}

void
CollisionPrevention::_addObstacleSensorData(const obstacle_distance_s &obstacle, const matrix::Quatf &vehicle_attitude)
{
	int msg_index = 0;
	float vehicle_orientation_deg = math::degrees(Eulerf(vehicle_attitude).psi());
	float increment_factor = 1.f / obstacle.increment;

	if (obstacle.frame == obstacle.MAV_FRAME_GLOBAL || obstacle.frame == obstacle.MAV_FRAME_LOCAL_NED) {
		// Obstacle message arrives in local_origin frame (north aligned)
		// corresponding data index (convert to world frame and shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(vehicle_orientation_deg + bin_angle_deg - obstacle.angle_offset) * increment_factor);
			//lc add 确保数组不越界
			msg_index %= INTERNAL_MAP_USED_BINS;

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {
				if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[msg_index] * 0.01f)) {
					_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
					_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
					_data_maxranges[i] = obstacle.max_distance;
					_data_fov[i] = 1;
				}
			}
		}

		//lc add
		//todo transform to body frame
		//前6位为下方数据 后6位为上方数据
		for(int i = INTERNAL_MAP_USED_BINS; i < INTERNAL_MAP_USED_BINS + INTERNAL_MAP_UPDOWN_BLOCK; i++){
			_obstacle_map_body_frame.distances[i] = obstacle.distances[i];
		}

	} else if (obstacle.frame == obstacle.MAV_FRAME_BODY_FRD) {
		// Obstacle message arrives in body frame (front aligned)
		// corresponding data index (shift by msg offset)
		for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) {
			float bin_angle_deg = (float)i * INTERNAL_MAP_INCREMENT_DEG +
					      _obstacle_map_body_frame.angle_offset;
			msg_index = ceil(wrap_360(bin_angle_deg - obstacle.angle_offset) * increment_factor);

			//add all data points inside to FOV
			if (obstacle.distances[msg_index] != UINT16_MAX) {

				if (_enterData(i, obstacle.max_distance * 0.01f, obstacle.distances[msg_index] * 0.01f)) {
					_obstacle_map_body_frame.distances[i] = obstacle.distances[msg_index];
					_data_timestamps[i] = _obstacle_map_body_frame.timestamp;
					_data_maxranges[i] = obstacle.max_distance;
					_data_fov[i] = 1;
				}
			}
		}

	} else {
		mavlink_log_critical(&_mavlink_log_pub, "Obstacle message received in unsupported frame %i\t",
				     obstacle.frame);
		events::send<uint8_t>(events::ID("col_prev_unsup_frame"), events::Log::Error,
				      "Obstacle message received in unsupported frame {1}", obstacle.frame);
	}
}

bool
CollisionPrevention::_enterData(int map_index, float sensor_range, float sensor_reading)
{
	//use data from this sensor if:
	//1. this sensor data is in range, the bin contains already valid data and this data is coming from the same or less range sensor
	//2. this sensor data is in range, and the last reading was out of range
	//3. this sensor data is out of range, the last reading was as well and this is the sensor with longest range
	//4. this sensor data is out of range, the last reading was valid and coming from the same sensor

	uint16_t sensor_range_cm = static_cast<uint16_t>(100.0f * sensor_range + 0.5f); //convert to cm

	if (sensor_reading < sensor_range) {
		if ((_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
		     && sensor_range_cm <= _data_maxranges[map_index])
		    || _obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]) {

			return true;
		}

	} else {
		if ((_obstacle_map_body_frame.distances[map_index] >= _data_maxranges[map_index]
		     && sensor_range_cm >= _data_maxranges[map_index])
		    || (_obstacle_map_body_frame.distances[map_index] < _data_maxranges[map_index]
			&& sensor_range_cm == _data_maxranges[map_index])) {

			return true;
		}
	}

	return false;
}

void
CollisionPrevention::_updateObstacleMap()
{
	_sub_vehicle_attitude.update();

	// add distance sensor data
	for (auto &dist_sens_sub : _distance_sensor_subs) {
		distance_sensor_s distance_sensor;

		if (dist_sens_sub.update(&distance_sensor)) {
			// consider only instances with valid data and orientations useful for collision prevention
			if ((getElapsedTime(&distance_sensor.timestamp) < RANGE_STREAM_TIMEOUT_US) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) &&
			    (distance_sensor.orientation != distance_sensor_s::ROTATION_UPWARD_FACING)) {

				// update message description
				_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, distance_sensor.timestamp);
				_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
									(uint16_t)(distance_sensor.max_distance * 100.0f));
				_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
									(uint16_t)(distance_sensor.min_distance * 100.0f));

				_addDistanceSensorData(distance_sensor, Quatf(_sub_vehicle_attitude.get().q));
			}
		}
	}

	// add obstacle distance data
	if (_sub_obstacle_distance.update()) {
		const obstacle_distance_s &obstacle_distance = _sub_obstacle_distance.get();

		// Update map with obstacle data if the data is not stale
		if (getElapsedTime(&obstacle_distance.timestamp) < RANGE_STREAM_TIMEOUT_US && obstacle_distance.increment > 0.f) {
			//update message description
			_obstacle_map_body_frame.timestamp = math::max(_obstacle_map_body_frame.timestamp, obstacle_distance.timestamp);
			_obstacle_map_body_frame.max_distance = math::max(_obstacle_map_body_frame.max_distance,
								obstacle_distance.max_distance);
			_obstacle_map_body_frame.min_distance = math::min(_obstacle_map_body_frame.min_distance,
								obstacle_distance.min_distance);
			_addObstacleSensorData(obstacle_distance, Quatf(_sub_vehicle_attitude.get().q));
		}
	}

	// publish fused obtacle distance message with data from offboard obstacle_distance and distance sensor
	_obstacle_distance_pub.publish(_obstacle_map_body_frame);
}

void
CollisionPrevention::_addDistanceSensorData(distance_sensor_s &distance_sensor, const matrix::Quatf &vehicle_attitude)
{
	// clamp at maximum sensor range
	float distance_reading = math::min(distance_sensor.current_distance, distance_sensor.max_distance);

	// discard values below min range
	if ((distance_reading > distance_sensor.min_distance)) {

		float sensor_yaw_body_rad = _sensorOrientationToYawOffset(distance_sensor, _obstacle_map_body_frame.angle_offset);
		float sensor_yaw_body_deg = math::degrees(wrap_2pi(sensor_yaw_body_rad));

		// calculate the field of view boundary bin indices
		int lower_bound = (int)floor((sensor_yaw_body_deg  - math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);
		int upper_bound = (int)floor((sensor_yaw_body_deg  + math::degrees(distance_sensor.h_fov / 2.0f)) /
					     INTERNAL_MAP_INCREMENT_DEG);

		// floor values above zero, ceil values below zero
		if (lower_bound < 0) { lower_bound++; }

		if (upper_bound < 0) { upper_bound++; }

		// rotate vehicle attitude into the sensor body frame
		matrix::Quatf attitude_sensor_frame = vehicle_attitude;
		attitude_sensor_frame.rotate(Vector3f(0.f, 0.f, sensor_yaw_body_rad));
		float sensor_dist_scale = cosf(Eulerf(attitude_sensor_frame).theta());

		if (distance_reading < distance_sensor.max_distance) {
			distance_reading = distance_reading * sensor_dist_scale;
		}

		uint16_t sensor_range = static_cast<uint16_t>(100.0f * distance_sensor.max_distance + 0.5f); // convert to cm

		for (int bin = lower_bound; bin <= upper_bound; ++bin) {
			int wrapped_bin = wrap_bin(bin);

			if (_enterData(wrapped_bin, distance_sensor.max_distance, distance_reading)) {
				_obstacle_map_body_frame.distances[wrapped_bin] = static_cast<uint16_t>(100.0f * distance_reading + 0.5f);
				_data_timestamps[wrapped_bin] = _obstacle_map_body_frame.timestamp;
				_data_maxranges[wrapped_bin] = sensor_range;
				_data_fov[wrapped_bin] = 1;
			}
		}
	}
}

void
CollisionPrevention::_adaptSetpointDirection(Vector2f &setpoint_dir, int &setpoint_index, float vehicle_yaw_angle_rad)
{
	const float col_prev_d = _param_cp_dist.get();
	const int guidance_bins = floor(_param_cp_guide_ang.get() / INTERNAL_MAP_INCREMENT_DEG);
	const int sp_index_original = setpoint_index;
	float best_cost = 9999.f;
	int new_sp_index = setpoint_index;

	for (int i = sp_index_original - guidance_bins; i <= sp_index_original + guidance_bins; i++) {

		// apply moving average filter to the distance array to be able to center in larger gaps
		const int filter_size = 1;
		float mean_dist = 0;

		for (int j = i - filter_size; j <= i + filter_size; j++) {
			int bin = wrap_bin(j);

			if (_obstacle_map_body_frame.distances[bin] == UINT16_MAX) {
				mean_dist += col_prev_d * 100.f;

			} else {
				mean_dist += _obstacle_map_body_frame.distances[bin];
			}
		}

		const int bin = wrap_bin(i);
		mean_dist = mean_dist / (2.f * filter_size + 1.f);
		const float deviation_cost = col_prev_d * 50.f * abs(i - sp_index_original);
		const float bin_cost = deviation_cost - mean_dist - _obstacle_map_body_frame.distances[bin];

		if (bin_cost < best_cost && _obstacle_map_body_frame.distances[bin] != UINT16_MAX) {
			best_cost = bin_cost;
			new_sp_index = bin;
		}
	}

	//only change setpoint direction if it was moved to a different bin
	if (new_sp_index != setpoint_index) {
		float angle = math::radians((float)new_sp_index * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);
		angle = wrap_2pi(vehicle_yaw_angle_rad + angle);
		setpoint_dir = {cosf(angle), sinf(angle)};
		setpoint_index = new_sp_index;
	}
}

float
CollisionPrevention::_sensorOrientationToYawOffset(const distance_sensor_s &distance_sensor, float angle_offset) const
{
	float offset = angle_offset > 0.0f ? math::radians(angle_offset) : 0.0f;

	switch (distance_sensor.orientation) {
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


//lc add
void CollisionPrevention::_ConstrainSetpoint_ZDown(float &setpointz, float stick)
{
    /* 参数说明:
     * stick > 0 向下运动 | stick < 0 向上运动 | 距离单位: 厘米
     * STOP_GAP_PHASE1: 第一阶段警戒距离
     * STOP_GAP_PHASE2: 第二阶段危险距离
     * DECEL_EXPONENT:  指数衰减系数(值越大减速越剧烈)
     */
    static constexpr float STOP_GAP_PHASE1 = 250.0f;
    static constexpr float STOP_GAP_PHASE2 = 120.0f;
    static constexpr int CLIP_MAX_PHASE1 = 100;    // 第一阶段需持续推动次数
    static constexpr int CLIP_MAX_PHASE2 = 150;    // 第二阶段需持续推动次数
    static constexpr float DECEL_EXPONENT = 5.0f;  // 指数衰减强度系数

    // 状态变量
    static int phase1_counter = 0;  // 第一阶段操作计数器
    static int phase2_counter = 0;  // 第二阶段操作计数器

	// 初始化最小距离为最大可测距离
    float min_dist = _obstacle_map_body_frame.max_distance;
    const hrt_abstime current_time = getTime();

    // 公共处理逻辑
    auto handlePhase = [&](int &counter, int counter_max, float stop_gap) {
        if (stick > 0.0f) {  // 仅处理向下运动
            if (counter <= counter_max) {
                // 安全模式：强制停止并累积操作计数
                setpointz = 0.0f;
                // 摇杆强度>50%时累积，否则重置
                counter += (stick > 0.5f) ? 1 : -counter;
            } else {
                // 解除限制后：根据距离指数衰减速度
				//setpointz = stick * expf(-DECEL_EXPONENT * math::constrain((stop_gap - min_dist) / stop_gap, 0.0f, 1.0f));
                float distance_ratio = (stop_gap - min_dist) / stop_gap;  // 危险程度[0,1]
                distance_ratio = math::constrain(distance_ratio, 0.0f, 1.0f);
                float speed_decay = expf(-DECEL_EXPONENT * distance_ratio);  // 指数衰减
                setpointz = stick * speed_decay;  // 应用衰减后的速度
            }
        } else {  // 摇杆释放时重置状态
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
		for(int i = INTERNAL_MAP_USED_BINS; i < INTERNAL_MAP_USED_BINS + (INTERNAL_MAP_UPDOWN_BLOCK / 2); ++i){
			if(_obstacle_map_body_frame.distances[i] < min_dist)
			min_dist = _obstacle_map_body_frame.distances[i];
		}

        // 分级安全处理
        if (min_dist < STOP_GAP_PHASE2) {       // 进入危险距离
            handlePhase(phase2_counter, CLIP_MAX_PHASE2, STOP_GAP_PHASE2);
            phase1_counter = 0;  // 重置第一阶段计数器
        }
        else if (min_dist < STOP_GAP_PHASE1) {  // 进入警戒距离
            handlePhase(phase1_counter, CLIP_MAX_PHASE1, STOP_GAP_PHASE1);
            phase2_counter = 0;  // 重置第二阶段计数器
        }
        else {  // 安全距离内
            setpointz = stick;
            phase1_counter = phase2_counter = 0;
        }
    }
    else {  // 传感器数据超时处理
        setpointz = 0.0f;
        phase1_counter = phase2_counter = 0;
    }

    // 调试数据发布
    struct debug_key_value_s dbg{};
    strncpy(dbg.key, "min_dist", sizeof(dbg.key));
    dbg.value = min_dist;
    dbg.timestamp = current_time;
    orb_advert_t pub_dbg = orb_advertise(ORB_ID(debug_key_value), &dbg);
    orb_publish(ORB_ID(debug_key_value), pub_dbg, &dbg);
}

void CollisionPrevention::_ConstrainSetpoint_ZUp(float &setpointz, float stick){

	static constexpr float DECEL_EXPONENT = 4.0f;  // 指数衰减强度系数
	const constexpr float stop_gap = 200.0f;
	const constexpr float slow_gap = 350.0f;
    const hrt_abstime current_time = getTime();
	// 初始化最小距离为最大可测距离
    float min_dist = _obstacle_map_body_frame.max_distance;

	// 更新障碍物地图数据
    _updateObstacleMap();

    // 检查传感器数据有效性(300ms超时)
    if ((current_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US)
    {

		if(stick < 0.0f)
		{
			// 遍历指定区域寻找最小障碍物距离
			for(int i = INTERNAL_MAP_USED_BINS + (INTERNAL_MAP_UPDOWN_BLOCK / 2); i < INTERNAL_MAP_USED_BINS + INTERNAL_MAP_UPDOWN_BLOCK; ++i){
				if(_obstacle_map_body_frame.distances[i] < min_dist)
				min_dist = _obstacle_map_body_frame.distances[i];
			}

			if(min_dist < stop_gap)
				setpointz = 0.0f;

			else if(min_dist < slow_gap){
				// 计算减速系数：越接近停止距离，衰减越强
				const float decel_range = slow_gap - stop_gap;
				const float distance_ratio = (min_dist - stop_gap) / decel_range;

				// 使用三次曲线实现平滑过渡（可替换为其他缓动函数）
				const float eased_ratio = distance_ratio * distance_ratio * (3.0f - 2.0f * distance_ratio);
				const float speed_decay = expf(-DECEL_EXPONENT * (1.0f - eased_ratio));

				setpointz = stick * speed_decay;
			}
			else
				return;
		}

		else
			return;
	}

	else{
		setpointz = 0.0f;
	}


}


void
CollisionPrevention::_calculateConstrainedSetpoint(Vector2f &setpoint, const Vector2f &curr_pos,
		const Vector2f &curr_vel)
{
	_updateObstacleMap();

	// read parameters
	const float col_prev_d = _param_cp_dist.get();
	const float col_prev_dly = _param_cp_delay.get();
	const bool move_no_data = _param_cp_go_nodata.get();
	const float xy_p = _param_mpc_xy_p.get();
	const float max_jerk = _param_mpc_jerk_max.get();
	const float max_accel = _param_mpc_acc_hor.get();
	const matrix::Quatf attitude = Quatf(_sub_vehicle_attitude.get().q);
	const float vehicle_yaw_angle_rad = Eulerf(attitude).psi();

	const float setpoint_length = setpoint.norm();

	const hrt_abstime constrain_time = getTime();
	int num_fov_bins = 0;

	if ((constrain_time - _obstacle_map_body_frame.timestamp) < RANGE_STREAM_TIMEOUT_US) {
		if (setpoint_length > 0.001f) {

			Vector2f setpoint_dir = setpoint / setpoint_length;
			float vel_max = setpoint_length;
			const float min_dist_to_keep = math::max(_obstacle_map_body_frame.min_distance / 100.0f, col_prev_d);

			const float sp_angle_body_frame = atan2f(setpoint_dir(1), setpoint_dir(0)) - vehicle_yaw_angle_rad;
			const float sp_angle_with_offset_deg = wrap_360(math::degrees(sp_angle_body_frame) -
							       _obstacle_map_body_frame.angle_offset);
			int sp_index = floor(sp_angle_with_offset_deg / INTERNAL_MAP_INCREMENT_DEG);

			// change setpoint direction slightly (max by _param_cp_guide_ang degrees) to help guide through narrow gaps
			_adaptSetpointDirection(setpoint_dir, sp_index, vehicle_yaw_angle_rad);

			// limit speed for safe flight
			for (int i = 0; i < INTERNAL_MAP_USED_BINS; i++) { // disregard unused bins at the end of the message

				// delete stale values
				const hrt_abstime data_age = constrain_time - _data_timestamps[i];

				if (data_age > RANGE_STREAM_TIMEOUT_US) {
					_obstacle_map_body_frame.distances[i] = UINT16_MAX;
				}

				const float distance = _obstacle_map_body_frame.distances[i] * 0.01f; // convert to meters
				const float max_range = _data_maxranges[i] * 0.01f; // convert to meters
				float angle = math::radians((float)i * INTERNAL_MAP_INCREMENT_DEG + _obstacle_map_body_frame.angle_offset);

				// convert from body to local frame in the range [0, 2*pi]
				angle = wrap_2pi(vehicle_yaw_angle_rad + angle);

				// get direction of current bin
				const Vector2f bin_direction = {cosf(angle), sinf(angle)};

				//count number of bins in the field of valid_new
				if (_obstacle_map_body_frame.distances[i] < UINT16_MAX) {
					num_fov_bins ++;
				}

				if (_obstacle_map_body_frame.distances[i] > _obstacle_map_body_frame.min_distance
				    && _obstacle_map_body_frame.distances[i] < UINT16_MAX) {

					if (setpoint_dir.dot(bin_direction) > 0) {
						// calculate max allowed velocity with a P-controller (same gain as in the position controller)
						const float curr_vel_parallel = math::max(0.f, curr_vel.dot(bin_direction));
						float delay_distance = curr_vel_parallel * col_prev_dly;

						if (distance < max_range) {
							delay_distance += curr_vel_parallel * (data_age * 1e-6f);
						}

						const float stop_distance = math::max(0.f, distance - min_dist_to_keep - delay_distance);
						const float vel_max_posctrl = xy_p * stop_distance;

						const float vel_max_smooth = math::trajectory::computeMaxSpeedFromDistance(max_jerk, max_accel, stop_distance, 0.f);
						const float projection = bin_direction.dot(setpoint_dir);
						float vel_max_bin = vel_max;

						if (projection > 0.01f) {
							vel_max_bin = math::min(vel_max_posctrl, vel_max_smooth) / projection;
						}

						// constrain the velocity
						if (vel_max_bin >= 0) {
							vel_max = math::min(vel_max, vel_max_bin);
						}
					}

				} else if (_obstacle_map_body_frame.distances[i] == UINT16_MAX && i == sp_index) {
					if (!move_no_data || (move_no_data && _data_fov[i])) {
						vel_max = 0.f;
					}
				}
			}

			//if the sensor field of view is zero, never allow to move (even if move_no_data=1)
			if (num_fov_bins == 0) {
				vel_max = 0.f;
			}

			setpoint = setpoint_dir * vel_max;
		}

	} else {
		//allow no movement
		float vel_max = 0.f;
		setpoint = setpoint * vel_max;

		// if distance data is stale, switch to Loiter
		if (getElapsedTime(&_last_timeout_warning) > 1_s && getElapsedTime(&_time_activated) > 1_s) {

			if ((constrain_time - _obstacle_map_body_frame.timestamp) > TIMEOUT_HOLD_US
			    && getElapsedTime(&_time_activated) > TIMEOUT_HOLD_US) {
				_publishVehicleCmdDoLoiter();
			}

			_last_timeout_warning = getTime();
		}


	}
}

void
CollisionPrevention::modifySetpoint(Vector2f &original_setpoint, const float max_speed, const Vector2f &curr_pos,
				    const Vector2f &curr_vel)
{
	//calculate movement constraints based on range data
	Vector2f new_setpoint = original_setpoint;
	_calculateConstrainedSetpoint(new_setpoint, curr_pos, curr_vel);

	//warn user if collision prevention starts to interfere
	bool currently_interfering = (new_setpoint(0) < original_setpoint(0) - 0.05f * max_speed
				      || new_setpoint(0) > original_setpoint(0) + 0.05f * max_speed
				      || new_setpoint(1) < original_setpoint(1) - 0.05f * max_speed
				      || new_setpoint(1) > original_setpoint(1) + 0.05f * max_speed);

	_interfering = currently_interfering;

	// publish constraints
	collision_constraints_s	constraints{};
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
	command.param1 = (float)1; // base mode
	command.param3 = (float)0; // sub mode
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
