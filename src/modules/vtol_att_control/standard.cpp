/****************************************************************************
 *
 *   Copyright (c) 2015-2022 PX4 Development Team. All rights reserved.
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
 * @file standard.cpp
 *
 * @author Simon Wilks		<simon@uaventure.com>
 * @author Roman Bapst		<bapstroman@gmail.com>
 * @author Andreas Antener	<andreas@uaventure.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
*/

#include "standard.h"
#include <float.h>

using namespace matrix;

Standard::Standard(VtolAttitudeControl *attc) :
	VtolType(attc)
{
}

void Standard::parameters_update()
{
	VtolType::updateParams();

	// make sure that pusher ramp in backtransition is smaller than back transition (max) duration
	_param_vt_b_trans_ramp.set(math::min(_param_vt_b_trans_ramp.get(), _param_vt_b_trans_dur.get()));
}


void Standard::update_vtol_state()
{
	// After flipping the switch the vehicle will start the pusher (or tractor) motor, picking up
	// forward speed. After the vehicle has picked up enough speed the rotors shutdown.
	// For the back transition the pusher motor is immediately stopped and rotors reactivated.

	float mc_weight = _mc_roll_weight;

	if (!_attc->is_fixed_wing_requested()) {

		// the transition to fw mode switch is off
		if (_vtol_mode == vtol_mode::MC_MODE) {
			// in mc mode
			_vtol_mode = vtol_mode::MC_MODE;
			mc_weight = 1.0f;
			//mavlink_log_critical(&_mavlink_log_pub, "MC MODE");

		} else {
			// Regular backtransition
			//resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_MC;
			//mavlink_log_critical(&_mavlink_log_pub, "TRANSITION TO MC");
		}
	} else {
		// the transition to fw mode switch is on
		if (_vtol_mode == vtol_mode::FW_MODE) {
			// in fw mode
			_vtol_mode = vtol_mode::FW_MODE;
			mc_weight = 0.0f;
			//mavlink_log_critical(&_mavlink_log_pub, "FW MODE");

		} else {
			resetTransitionStates();
			_vtol_mode = vtol_mode::TRANSITION_TO_FW;
			//mavlink_log_critical(&_mavlink_log_pub, "TRANSITION TO FW");
		}
	}

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;

	// map specific control phases to simple control modes
	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:
		_common_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_common_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_TO_FW:
		_common_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_TO_MC:
		_common_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}


void Standard::update_transition_state()
{
	float mc_weight = _mc_roll_weight;

	// 1. 初始化过渡状态时间戳 (每次进入过渡态时只执行一次)
	if (_trans_start_time == 0) {
		_trans_start_time = hrt_absolute_time();
		_transformation_complete = false;
		//mavlink_log_critical(&_mavlink_log_pub, "Transformation mechanism started.");
	}

	// 2. 更新编码器数据 (你提供的逻辑)
	bool updated;
	orb_check(_encoder_sub, &updated);
	if (updated) {
		sensor_encoder_s encoder_data;
		orb_copy(ORB_ID(sensor_encoder), _encoder_sub, &encoder_data);
		_current_mechanism_angle = encoder_data.position_rad;
	}

	// 3. 超时保护逻辑 (检测变形是否卡死)
	float timeout_us = 10.0f * 1000000.0f; // 从参数获取的超时时间(秒转微秒)
	if (hrt_absolute_time() - _trans_start_time > timeout_us && !_transformation_complete) {
		//mavlink_log_critical(&_mavlink_log_pub, "TRANSFORMATION TIMEOUT! Mechanism may be stuck.");
		// 处理卡死情况：这里选择安全回退到 MC 模式
		_vtol_mode = vtol_mode::MC_MODE;
		mc_weight = 1.0f;
		_trans_start_time = 0;
		return;
	}

	// =========================================================================
	// 4. 方向 A: 多旋翼 -> 漫游车 (MC -> FW)
	// =========================================================================
	if (_vtol_mode == vtol_mode::TRANSITION_TO_FW) {
		// 在变形期间，保持多旋翼姿态全权控制 (确保如果在坡上或有风时车身稳定)
		mc_weight = 1.0f;

		// 目标角度是漫游车(FW)的角度
		float target_angle = _param_vt_mech_ang_fw.get();

		// 假设编码器角度从小到大变化 (MC较小，Rover较大)
		// 注意：这里的方向判断 (< 或 >) 取决于你的机械设计和编码器安装方向
		if (_current_mechanism_angle > target_angle - 0.05f) { // 加一点死区容差 (0.05rad)
		output_cmd = _param_vt_mech_cmd_fw.get();
		} else {
		// 变形到位，切换模式！
		_transformation_complete = true;
		_vtol_mode = vtol_mode::FW_MODE; // 切入漫游车模式
		mc_weight = 0.0f;               // 彻底关闭多旋翼控制权
		output_cmd = 0.0f;     // 停止变形机构电机
		_trans_start_time = 0;           // 重置时间戳，为下次变形做准备
		//mavlink_log_critical(&_mavlink_log_pub, "Transformed to Rover Mode.");
		}
	}
	// =========================================================================
	// 5. 方向 B: 漫游车 -> 多旋翼 (FW -> MC)
	// =========================================================================
	else if (_vtol_mode == vtol_mode::TRANSITION_TO_MC) {
		// 变形回多旋翼期间，我们依然让多旋翼控制权为0，直到变形完成再开启旋翼
		mc_weight = 1.0f;

		// 目标角度是多旋翼(MC)的角度
		float target_angle = _param_vt_mech_ang_mc.get();

		if (_current_mechanism_angle < target_angle + 0.05f) { // 死区容差
		output_cmd = -_param_vt_mech_cmd_mc.get(); // 反向驱动机构
		} else {
		// 变形到位，切换模式！
		_transformation_complete = true;
		_vtol_mode = vtol_mode::MC_MODE; // 切入多旋翼模式
		mc_weight = 1.0f;               // 恢复旋翼全权控制 (准备起飞)
		output_cmd = 0.0f;     // 停止变形机构
		_trans_start_time = 0;
		//mavlink_log_critical(&_mavlink_log_pub, "Transformed to Multicopter Mode.");
		}
	}
	mc_weight = math::constrain(mc_weight, 0.0f, 1.0f);

	_mc_roll_weight = mc_weight;
	_mc_pitch_weight = mc_weight;
	_mc_yaw_weight = mc_weight;
	_mc_throttle_weight = mc_weight;
}

void Standard::update_mc_state()
{
	//VtolType::update_mc_state();

	//_pusher_throttle = VtolType::pusher_assist();
}

void Standard::update_fw_state()
{
	//VtolType::update_fw_state();
}

/**
 * Prepare message to actuators with data from mc and fw attitude controllers. An mc attitude weighting will determine
 * what proportion of control should be applied to each of the control groups (mc and fw).
 */
void Standard::fill_actuator_outputs()
{
	_torque_setpoint_0->timestamp = hrt_absolute_time();
	_torque_setpoint_0->timestamp_sample = _vehicle_torque_setpoint_virtual_mc->timestamp_sample;
	_torque_setpoint_0->xyz[0] = 0.f;
	_torque_setpoint_0->xyz[1] = 0.f;
	_torque_setpoint_0->xyz[2] = 0.f;

	_torque_setpoint_1->timestamp = hrt_absolute_time();
	_torque_setpoint_1->timestamp_sample = _vehicle_torque_setpoint_virtual_fw->timestamp_sample;
	_torque_setpoint_1->xyz[0] = 0.f;
	_torque_setpoint_1->xyz[1] = 0.f;
	_torque_setpoint_1->xyz[2] = 0.f;

	_thrust_setpoint_0->timestamp = hrt_absolute_time();
	_thrust_setpoint_0->timestamp_sample = _vehicle_thrust_setpoint_virtual_mc->timestamp_sample;
	_thrust_setpoint_0->xyz[0] = 0.f;
	_thrust_setpoint_0->xyz[1] = 0.f;
	_thrust_setpoint_0->xyz[2] = 0.f;

	_thrust_setpoint_1->timestamp = hrt_absolute_time();
	_thrust_setpoint_1->timestamp_sample = _vehicle_thrust_setpoint_virtual_fw->timestamp_sample;
	_thrust_setpoint_1->xyz[0] = 0.f;
	_thrust_setpoint_1->xyz[1] = 0.f;
	_thrust_setpoint_1->xyz[2] = 0.f;

	switch (_vtol_mode) {
	case vtol_mode::MC_MODE:

		// MC actuators:
		_torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0];
		_torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1];
		_torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2];
		_thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2];

		// FW actuators:
		if (!_param_vt_elev_mc_lock.get()) {
			_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
			_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		}

		_thrust_setpoint_0->xyz[0] = _pusher_throttle;
		break;

	case vtol_mode::TRANSITION_TO_FW:

	// FALLTHROUGH
	case vtol_mode::TRANSITION_TO_MC:
		// // MC actuators:
		// _torque_setpoint_0->xyz[0] = _vehicle_torque_setpoint_virtual_mc->xyz[0] * _mc_roll_weight;
		// _torque_setpoint_0->xyz[1] = _vehicle_torque_setpoint_virtual_mc->xyz[1] * _mc_pitch_weight;
		// _torque_setpoint_0->xyz[2] = _vehicle_torque_setpoint_virtual_mc->xyz[2] * _mc_yaw_weight;
		// _thrust_setpoint_0->xyz[2] = _vehicle_thrust_setpoint_virtual_mc->xyz[2] * _mc_throttle_weight;

		// // FW actuators
		// _torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0] * (1.f - _mc_roll_weight);
		// _torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1] * (1.f - _mc_pitch_weight);
		// _torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2] * (1.f - _mc_yaw_weight);
		// _thrust_setpoint_0->xyz[0] = _pusher_throttle;

		// break;

	case vtol_mode::FW_MODE:

		// FW actuators
		_torque_setpoint_1->xyz[0] = _vehicle_torque_setpoint_virtual_fw->xyz[0];
		_torque_setpoint_1->xyz[1] = _vehicle_torque_setpoint_virtual_fw->xyz[1];
		_torque_setpoint_1->xyz[2] = _vehicle_torque_setpoint_virtual_fw->xyz[2];
		_thrust_setpoint_0->xyz[0] = _vehicle_thrust_setpoint_virtual_fw->xyz[0];
		break;
	}

	// 新增发布actuator命令到vehicle_command主题
	if (fabsf(output_cmd - _prev_output_cmd) > 0.01f) {
	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
	vcmd.param1 = output_cmd;
	vcmd.param2 = -output_cmd;
	vcmd.param3 = NAN;
	vcmd.param4 = NAN;
	vcmd.param5 = NAN;
	vcmd.param6 = NAN;
	vcmd.param7 = 0.0f;
	vcmd.target_system = 1;
	vcmd.target_component = 1;
	vcmd.source_system = 1;
	vcmd.source_component = 1;
	vcmd.from_external = false;

	_vehicle_cmd_pub.publish(vcmd);

	// 更新记录值
	_prev_output_cmd = output_cmd;
	}
}

void Standard::waiting_on_tecs()
{
	// keep thrust from transition
	_v_att_sp->thrust_body[0] = _pusher_throttle;
};

void Standard::blendThrottleAfterFrontTransition(float scale)
{
	const float tecs_throttle = _v_att_sp->thrust_body[0];
	_v_att_sp->thrust_body[0] = scale * tecs_throttle + (1.0f - scale) * _pusher_throttle;
}
