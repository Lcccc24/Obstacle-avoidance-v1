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
 * @file collisionprevention_params.c
 *
 * Parameters defined by the collisionprevention lib.
 *
 * @author Tanja Baumann <tanja@auterion.com>
 */

/**
 * Minimum distance the vehicle should keep to all obstacles
 *
 * Only used in Position mode. Collision avoidance is disabled by setting this parameter to a negative value
 *
 * @min -1
 * @max 15
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DIST, -1.0f);

/**
 * Average delay of the range sensor message plus the tracking delay of the position controller in seconds
 *
 * Only used in Position mode.
 *
 * @min 0
 * @max 1
 * @unit s
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DELAY, 0.4f);

/**
 * Angle left/right from the commanded setpoint by which the collision prevention algorithm can choose to change the setpoint direction
 *
 * Only used in Position mode.
 *
 * @min 0
 * @max 90
 * @unit deg
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_GUIDE_ANG, 30.f);

/**
 * Boolean to allow moving into directions where there is no sensor data (outside FOV)
 *
 * Only used in Position mode.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CP_GO_NO_DATA, 0);

/**
 * 0 for Brake Mode, 1 for Bypass Mode
 *
 * Only used in Position mode.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CP_MODE, 0);

/**
 * Down Stop Gate1 -- Initiate slow braking early
 *
 * Used in Brake Mode only.
 *
 * @min 1.5
 * @max 3
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DOWN_GATE1, 2.5f);

/**
 * Down Stop Gate2 -- Apply stronger braking closer
 *
 * Used in Brake Mode only.
 *
 * @min 0.5
 * @max 1.5
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DOWN_GATE2, 1.2f);

/**
 * Down Brake -- Exponential Decay Coefficient
 *
 * Used in Brake Mode only.
 *
 * @min 1
 * @max 50
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DOWN_DECAY, 5.0f);

/**
 * Up Stop Gate1  -- Decelerate at a longer distance
 *
 * Used in Brake Mode only.
 *
 * @min 2
 * @max 4
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_UP_GATE1, 3.5f);

/**
 * Up Stop Gate2 -- Full braking upon reaching the threshold
 *
 * Used in Brake Mode only.
 *
 * @min 0.5
 * @max 2
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_UP_GATE2, 2.0f);

/**
 * Up Brake -- Exponential Decay Coefficient
 *
 * Used in Brake Mode only.
 *
 * @min 1
 * @max 50
 * @decimal 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_UP_DECAY, 4.f);

/**
 * Bypass Deceleration Distance
 *
 * Used in Bypass Mode only.
 *
 * @min 3
 * @max 9
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DECEL_DIS, 4.f);

/**
 * Triggering Distance for Bypass Mode to take effect
 *
 * Used in Bypass Mode only.
 *
 * @min 1
 * @max 4
 * @unit m
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_BYPASS_DIS, 2.8f);

/**
 * MAX VEL in Bypass effect
 *
 * Used in Bypass Mode only.
 *
 * @min 0.5
 * @max 3
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_BYPASS_VEL, 1.0f);

/**
 * NEIGHBOR_BINS -- Bypass detect
 *
 * Used in Bypass Mode only.
 *
 * @min 1
 * @max 30
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(CP_NEI_BINS, 4);

/**
 * ALIGNMENT_GAIN in Bypass score
 *
 * Used in Bypass Mode only.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_ALIGN_GAIN, 0.4f);

/**
 * DISTANCE_GAIN in Bypass score
 *
 * Used in Bypass Mode only.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_DIS_GAIN, 0.6f);

/**
 * CP_HOR_DENSE in Bypass score , Determine use vertical bypass or not
 *
 * Used in Bypass Mode only.
 *
 * @min 0.0
 * @max 15.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_HOR_DENSE, 7.5f);

/**
 * CP_VER_GATE in Bypass score, Judge vertical distance to decide vertical bypass or not.
 *
 * Used in Bypass Mode only.
 *
 * @min 0.0
 * @max 15.0
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(CP_VER_GATE, 3.0f);
