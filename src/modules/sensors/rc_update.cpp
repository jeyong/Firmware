/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file rc_update.cpp
 *
 * @author Beat Kueng <beat-kueng@gmx.net>
 */

#include "rc_update.h"

#include <string.h>
#include <float.h>
#include <errno.h>

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/manual_control_setpoint.h>

using namespace sensors;

RCUpdate::RCUpdate(const Parameters &parameters)
	: _parameters(parameters),
	  _filter_roll(50.0f, 10.f), /* get replaced by parameter */
	  _filter_pitch(50.0f, 10.f),
	  _filter_yaw(50.0f, 10.f),
	  _filter_throttle(50.0f, 10.f)
{
	memset(&_rc, 0, sizeof(_rc));
	memset(&_rc_parameter_map, 0, sizeof(_rc_parameter_map));
	memset(&_param_rc_values, 0, sizeof(_param_rc_values));
}

// rc관련 subscribe 초기화. input_rc(sbus driver))와 rc_parameter_map subscribe(mavlink)를 위한 초기화.
// rc_parameter_map은 특정 param id와 rc 채널과 매핑시켜서 rc로 해당 param 값을 조정가능.
int RCUpdate::init()
{
	_rc_sub = orb_subscribe(ORB_ID(input_rc));

	if (_rc_sub < 0) {
		return -errno;
	}

	_rc_parameter_map_sub = orb_subscribe(ORB_ID(rc_parameter_map));

	if (_rc_parameter_map_sub < 0) {
		return -errno;
	}

	return 0;
}

void RCUpdate::deinit()
{
	orb_unsubscribe(_rc_sub);
	orb_unsubscribe(_rc_parameter_map_sub);
}

// rc기능 <-> rc채널을 매핑
void RCUpdate::update_rc_functions()
{
	// 채널은 18개, function은 총 26개. initialize_parameter_handles()에서 _parameters를 업데이트 
	// RC의 기능과 param에 설정된 채널을 매핑. 설정되지 않은 경우 0보다 작은 값인 -1.
	/* update RC function mappings */
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE] = _parameters.rc_map_throttle - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ROLL] = _parameters.rc_map_roll - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PITCH] = _parameters.rc_map_pitch - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_YAW] = _parameters.rc_map_yaw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MODE] = _parameters.rc_map_mode_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RETURN] = _parameters.rc_map_return_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE] = _parameters.rc_map_rattitude_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL] = _parameters.rc_map_posctl_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_LOITER] = _parameters.rc_map_loiter_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ACRO] = _parameters.rc_map_acro_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD] = _parameters.rc_map_offboard_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH] = _parameters.rc_map_kill_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH] = _parameters.rc_map_arm_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION] = _parameters.rc_map_trans_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_GEAR] = _parameters.rc_map_gear_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_STAB] = _parameters.rc_map_stab_sw - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_MAN] = _parameters.rc_map_man_sw - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS] = _parameters.rc_map_flaps - 1;

	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1] = _parameters.rc_map_aux1 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2] = _parameters.rc_map_aux2 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3] = _parameters.rc_map_aux3 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4] = _parameters.rc_map_aux4 - 1;
	_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5] = _parameters.rc_map_aux5 - 1;

	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] = _parameters.rc_map_param[i] - 1;
	}

	// RC 로우패스 필터 사용을 위한 인자로 sample rate와 cutoff rate를 파라미터에서 읽어서 설정
	// roll, pitch, yaw, throttle에 대해서 부드럽게 동작하도록 하기 위해서 lowpass filter 먹임
	/* update the RC low pass filter frequencies */
	_filter_roll.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_pitch.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_yaw.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_throttle.set_cutoff_frequency(_parameters.rc_flt_smp_rate, _parameters.rc_flt_cutoff);
	_filter_roll.reset(0.f);
	_filter_pitch.reset(0.f);
	_filter_yaw.reset(0.f);
	_filter_throttle.reset(0.f);
}

// mavlink로부터 rc_parameter_map topic관련 업데이트가 있는지 보고 업데이트
void
RCUpdate::rc_parameter_map_poll(ParameterHandles &parameter_handles, bool forced)
{
	bool map_updated;
	orb_check(_rc_parameter_map_sub, &map_updated);

	if (map_updated) {
		orb_copy(ORB_ID(rc_parameter_map), _rc_parameter_map_sub, &_rc_parameter_map);

		// RC 채널과 매핑되어 있는 param 핸들을 업데이트 
		/* update parameter handles to which the RC channels are mapped */
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) {
				/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
				 * or no request to map this channel to a param has been sent via mavlink
				 */
				// 이 RC 채널이 RC-Parameter Channel과 매핑되어 있는지 않는 경우 혹은 이 채널과 mavlink로 받은 param에 대한 매핑 요청이 없는 경우
				continue;
			}

			// index가 설정되어 있는 경우에는 index로 핸들을 설정하고 그렇지 않으면 id를 사용
			/* Set the handle by index if the index is set, otherwise use the id */
			if (_rc_parameter_map.param_index[i] >= 0) { // index로 param 핸들 설정
				parameter_handles.rc_param[i] = param_for_used_index((unsigned)_rc_parameter_map.param_index[i]);

			} else { // param_index[i]가 -1이면 param_id를 사용. param_id는 문자열
				parameter_handles.rc_param[i] = param_find(&_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]);
			}

		}

		PX4_DEBUG("rc to parameter map updated");

		// 매핑된 정보 출력
		for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
			PX4_DEBUG("\ti %d param_id %s scale %.3f value0 %.3f, min %.3f, max %.3f",
				  i,
				  &_rc_parameter_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)],
				  (double)_rc_parameter_map.scale[i],
				  (double)_rc_parameter_map.value0[i],
				  (double)_rc_parameter_map.value_min[i],
				  (double)_rc_parameter_map.value_max[i]
				 );
		}
	}
}

//해당 rc 기능(func)이 설정되어 있는 경우 매핑된 채널의 값을 가져오기
float
RCUpdate::get_rc_value(uint8_t func, float min_value, float max_value)
{
	if (_rc.function[func] >= 0) {
		float value = _rc.channels[_rc.function[func]];
		return math::constrain(value, min_value, max_value);

	} else {
		return 0.0f;
	}
}

// 3 단계 스위치로 사용하는 경우, 현재 RC 조정기의 값으로 ON/Middle/OFF 를 반환
switch_pos_t
RCUpdate::get_rc_sw3pos_position(uint8_t func, float on_th, bool on_inv, float mid_th, bool mid_inv)
{
	if (_rc.function[func] >= 0) { // 해당 rc채널이 사용 매핑이 된 경우
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		// rc 조정기의 값으로 ON/MIDDLE/OFF 상태를 반환
		if (on_inv ? value < on_th : value > on_th) { // inverse 상태 체크. threshold 보다 크면 ON
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else if (mid_inv ? value < mid_th : value > mid_th) { // mid threshold보다 크면 MIDDLE
			return manual_control_setpoint_s::SWITCH_POS_MIDDLE;

		} else { // 그렇지 않으면 OFF 상태
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else { // 아예 사용 설정 매핑이 안된 경우 NONE
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

// 2 단계 스위치로 사용하는 경우, 현재 RC 조정기의 값으로 ON/OFF 를 반환 
switch_pos_t
RCUpdate::get_rc_sw2pos_position(uint8_t func, float on_th, bool on_inv)
{
	if (_rc.function[func] >= 0) { // 해당 rc채널이 사용 매핑이 된 경우
		float value = 0.5f * _rc.channels[_rc.function[func]] + 0.5f;

		if (on_inv ? value < on_th : value > on_th) { // inverse 체크.  threshold 보다 크면 ON
			return manual_control_setpoint_s::SWITCH_POS_ON;

		} else { // 그렇지 않으면 OFF
			return manual_control_setpoint_s::SWITCH_POS_OFF;
		}

	} else { // 매핑이 안되어 있는 경우 NONE
		return manual_control_setpoint_s::SWITCH_POS_NONE;
	}
}

// rc에서 조정기 값(knob 타입)으로 param 값을 설정
void
RCUpdate::set_params_from_rc(const ParameterHandles &parameter_handles)
{
	// RC_PARAM_MAP_NCHAN 즉 3개 채널에 대해서 매핑 여부 확인하고 매핑이 되어 있는 경우에 대해서 해당 param 값을 변경
	for (int i = 0; i < rc_parameter_map_s::RC_PARAM_MAP_NCHAN; i++) {
		if (_rc.function[rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i] < 0 || !_rc_parameter_map.valid[i]) { // RC-Parameter 채널이 매핑이 안되어 있으면 빠져나감
			/* This RC channel is not mapped to a RC-Parameter Channel (e.g. RC_MAP_PARAM1 == 0)
			 * or no request to map this channel to a param has been sent via mavlink
			 */
			continue;
		}

		// rc 채널의 값과 param에 저장된 값을 비교해서 
		float rc_val = get_rc_value((rc_channels_s::RC_CHANNELS_FUNCTION_PARAM_1 + i), -1.0, 1.0); //해당 채널로 rc 값을 읽어서

		// rc 값과 기존 param 값 사이에 차이가 있다고 판단되면, param에 변경된 값을 계산하여 update 한다.
		/* Check if the value has changed,
		 * maybe we need to introduce a more aggressive limit here */
		if (rc_val > _param_rc_values[i] + FLT_EPSILON || rc_val < _param_rc_values[i] - FLT_EPSILON) {
			_param_rc_values[i] = rc_val;
			float param_val = math::constrain(
						  _rc_parameter_map.value0[i] + _rc_parameter_map.scale[i] * rc_val,
						  _rc_parameter_map.value_min[i], _rc_parameter_map.value_max[i]);
			param_set(parameter_handles.rc_param[i], &param_val);
		}
	}
}

// raw rc값이 update된 경우, 이 값을 이용해서 rc_channels, manual_control_setpoint, actuator_controls_3를 publish 하기
void
RCUpdate::rc_poll(const ParameterHandles &parameter_handles)
{
	bool rc_updated;
	orb_check(_rc_sub, &rc_updated);

	if (rc_updated) {
		/* read low-level values from FMU or IO RC inputs (PPM, Spektrum, S.Bus) */
		struct rc_input_values rc_input;

		orb_copy(ORB_ID(input_rc), _rc_sub, &rc_input);

		/* detect RC signal loss */
		bool signal_lost;

		// 기본적으로 rc 신호가 유효한 상태인지를 체크하기 위해 4개 flag 상태를 조사. 채널 설정은 최소 5개 이상인 되어 있어야 정상.
		/* check flags and require at least four channels to consider the signal valid */
		if (rc_input.rc_lost || rc_input.rc_failsafe || rc_input.channel_count < 4) {
			/* signal is lost or no enough channels */
			signal_lost = true;

		} else { // rc 신호가 양호한 상태인 경우
			/* signal looks good */
			signal_lost = false;

			// 특정 채널을 failsfae 채널로 두고 이 채널이 특정 범위를 벗어나는 경우 signal_lost로 판정.
			// 기본적으로 throttle 채널을 사용하며 사용자가 18개 채널 중에 하나를 선택할 수도 있음.
			/* check failsafe */ // failsafe를 검사. failsafe 채널을 가지고 와서 
			int8_t fs_ch = _rc.function[_parameters.rc_map_failsafe]; // get channel mapped to throttle

			// rc_map_failsafe가 0인 경우에는 RC_CHANNELS_FUNCTION_THROTTLE = 0과 같이 throttle 채널을 fs_ch에 할당함.
			// 0이 아닌 값이면 rc_map_failsafe가 채널값이므로 이를 fs_ch에 할당
			if (_parameters.rc_map_failsafe > 0) { // if not 0, use channel number instead of rc.function mapping
				fs_ch = _parameters.rc_map_failsafe - 1;
			}

			// fs_ch 채널이 설정되어 있고 rc_fails_thr는 0보다 큰 경우 
			if (_parameters.rc_fails_thr > 0 && fs_ch >= 0) { // failsafe throttle 값이 failsafe 채널의 값의 범위를 벗어나는 경우, 리시버에서 신호를 받지 못한 상태라고 결정하고 failsafe를 시작
				/* failsafe configured */
				if ((_parameters.rc_fails_thr < _parameters.min[fs_ch] && rc_input.values[fs_ch] < _parameters.rc_fails_thr) ||
				    (_parameters.rc_fails_thr > _parameters.max[fs_ch] && rc_input.values[fs_ch] > _parameters.rc_fails_thr)) {
					/* failsafe triggered, signal is lost by receiver */
					signal_lost = true;
				}
			}
		}

		unsigned channel_limit = rc_input.channel_count;

		if (channel_limit > RC_MAX_CHAN_COUNT) { //18개보다 큰 값인 경우 18로 제한
			channel_limit = RC_MAX_CHAN_COUNT;
		}

		// raw 메시지에서 scale에 해당되는 부분을 읽어온다.
		/* read out and scale values from raw message even if signal is invalid */
		for (unsigned int i = 0; i < channel_limit; i++) {

			/*
			 * min/max 범위를 벗어나지 않도록 처리
			 * 1) Constrain to min/max values, as later processing depends on bounds.
			 */
			if (rc_input.values[i] < _parameters.min[i]) {
				rc_input.values[i] = _parameters.min[i];
			}

			if (rc_input.values[i] > _parameters.max[i]) {
				rc_input.values[i] = _parameters.max[i];
			}

			/*
			 * lower와 upper 범위에 대해서 중간 지점으로 scale
			 * 동일한 endpoint나 slop을 공유하지 않을 수 있으므로 필요.
			 * trim은 param에서 Mid point 값으로 설정. channels[]는 -1..1 로 스케일링 한 값이 들어감.
			 * 먼저 중간의 위나 아래에 대해서 0..1 범위로 정규화하며 이렇게 하면 전체 범위는 2(-1..1)가 된다. (중간 아래가 1, 중간 위가 1)
			 * 중간(trim)이 min 값이면 0..1로 스케일링하고 중간(trim)이 max인 경우에는 -1..0로 스케일링한다.
			 * 위에 1단계에서 min/max 범위에서 0으로 나누는 경우는 발생하지 않는다. 중간이 min이나 max인 경우에 NaN 경우가 되는 일이 없음.
			 * 2) Scale around the mid point differently for lower and upper range.
			 *
			 * This is necessary as they don't share the same endpoints and slope.
			 *
			 * First normalize to 0..1 range with correct sign (below or above center),
			 * the total range is 2 (-1..1).
			 * If center (trim) == min, scale to 0..1, if center (trim) == max,
			 * scale to -1..0.
			 *
			 * As the min and max bounds were enforced in step 1), division by zero
			 * cannot occur, as for the case of center == min or center == max the if
			 * statement is mutually exclusive with the arithmetic NaN case.
			 *
			 * DO NOT REMOVE OR ALTER STEP 1!
			 */
			// rc 값 > trim + deadzone경우 측 rc 값이 중간값 상위에 있는 경우
			if (rc_input.values[i] > (_parameters.trim[i] + _parameters.dz[i])) {
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] - _parameters.dz[i]) / (float)(
							  _parameters.max[i] - _parameters.trim[i] - _parameters.dz[i]);

			// rc 값 < trim - deadzone경우 측 rc 값이 중간값 하위에 있는 경우
			} else if (rc_input.values[i] < (_parameters.trim[i] - _parameters.dz[i])) { // 
				_rc.channels[i] = (rc_input.values[i] - _parameters.trim[i] + _parameters.dz[i]) / (float)(
							  _parameters.trim[i] - _parameters.min[i] - _parameters.dz[i]);

			} else { // rc 값이 (trim +- dead zone)내에 있는 경우 출력은 0이 됨
				/* in the configured dead zone, output zero */
				_rc.channels[i] = 0.0f;
			}

			_rc.channels[i] *= _parameters.rev[i]; // rever 설정된 경우 적용

			/* handle any parameter-induced blowups */ // 채널 값으로 사용할 수 없는 값이 경우 0으로 처리
			if (!PX4_ISFINITE(_rc.channels[i])) {
				_rc.channels[i] = 0.0f;
			}
		}

		// publish를 위해 _rc에 결과 저장
		_rc.channel_count = rc_input.channel_count; // 유효한 채널 갯수
		_rc.rssi = rc_input.rssi;
		_rc.signal_lost = signal_lost;
		_rc.timestamp = rc_input.timestamp_last_signal;
		_rc.frame_drop_count = rc_input.rc_lost_frame_count;

		// rc_channels을 publish(rc 신호가 유효하지 않더로도 디버깅 목적으로 publish)
		/* publish rc_channels topic even if signal is invalid, for debug */
		int instance;
		orb_publish_auto(ORB_ID(rc_channels), &_rc_pub, &_rc, &instance, ORB_PRIO_DEFAULT);

		// 신호가 유효한 경우에는 manual setpoint를 controller에게 전달
		/* only publish manual control if the signal is still present and was present once */
		if (!signal_lost && rc_input.timestamp_last_signal > 0) {

			/* initialize manual setpoint */
			struct manual_control_setpoint_s manual = {};
			/* set mode slot to unassigned */
			manual.mode_slot = manual_control_setpoint_s::MODE_SLOT_NONE;
			/* set the timestamp to the last signal time */
			manual.timestamp = rc_input.timestamp_last_signal;
			manual.data_source = manual_control_setpoint_s::SOURCE_RC;

			/* limit controls */ // 기본적으로 roll, pitch, yaw, throttle 및 aux 관련 제어
			manual.y = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_ROLL, -1.0, 1.0);
			manual.x = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_PITCH, -1.0, 1.0);
			manual.r = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_YAW, -1.0, 1.0);
			manual.z = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE, 0.0, 1.0);
			manual.flaps = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_FLAPS, -1.0, 1.0);
			manual.aux1 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_1, -1.0, 1.0);
			manual.aux2 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_2, -1.0, 1.0);
			manual.aux3 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_3, -1.0, 1.0);
			manual.aux4 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_4, -1.0, 1.0);
			manual.aux5 = get_rc_value(rc_channels_s::RC_CHANNELS_FUNCTION_AUX_5, -1.0, 1.0);

			/* filter controls */ // roll, pitch, yaw, throttle에 대해서 smooth하게 제어하기 위해서 low pass filter를 통과한 값으로 
			manual.y = math::constrain(_filter_roll.apply(manual.y), -1.f, 1.f);
			manual.x = math::constrain(_filter_pitch.apply(manual.x), -1.f, 1.f);
			manual.r = math::constrain(_filter_yaw.apply(manual.r), -1.f, 1.f);
			manual.z = math::constrain(_filter_throttle.apply(manual.z), 0.f, 1.f);

			// flightmode가 매핑된 채널이 있는 경우, 
			if (_parameters.rc_map_flightmode > 0) {

				// 해당 채널에 최대 6개 slot으로 할당이 가능
				/* the number of valid slots equals the index of the max marker minus one */
				const int num_slots = manual_control_setpoint_s::MODE_SLOT_MAX;

				// 2.0/num_slots은 slot의 폭이고 여기에 2를 나눠서 하나의 slot의 반폭
				/* the half width of the range of a slot is the total range
				 * divided by the number of slots, again divided by two
				 */
				const float slot_width_half = 2.0f / num_slots / 2.0f;

				// slot min/max에 대한 offset 
				/* min is -1, max is +1, range is 2. We offset below min and max */
				const float slot_min = -1.0f - 0.05f;
				const float slot_max = 1.0f + 0.05f;

				// min/max를 이용해서 0..1 구간에 대해서 먼저 정규화해서 매핑을 얻는다.
				// 다음으로 slot의 갯수를 곱해서 해당 slot을 얻을 수 있음.
				// 마지막으로 half slot width를 추가해서 정수 반올림이 되도록 해서 최종 index를 구하는 방식
				/* the slot gets mapped by first normalizing into a 0..1 interval using min
				 * and max. Then the right slot is obtained by multiplying with the number of
				 * slots. And finally we add half a slot width to ensure that integer rounding
				 * will take us to the correct final index.
				 */
				manual.mode_slot = (
					(
					(
					(
						(_rc.channels[_parameters.rc_map_flightmode - 1] - slot_min) * 
					num_slots)
					 + slot_width_half)
					 /
					(slot_max - slot_min)
					) + (1.0f / num_slots));

				if (manual.mode_slot >= num_slots) {
					manual.mode_slot = num_slots - 1;
				}
			}

			// mode 스위치 상태 가져오기
			/* mode switches */
			manual.mode_switch = get_rc_sw3pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MODE, _parameters.rc_auto_th,
					     _parameters.rc_auto_inv, _parameters.rc_assist_th, _parameters.rc_assist_inv);
			manual.rattitude_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RATTITUDE,
						  _parameters.rc_rattitude_th,
						  _parameters.rc_rattitude_inv);
			manual.posctl_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_POSCTL, _parameters.rc_posctl_th,
					       _parameters.rc_posctl_inv);
			manual.return_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_RETURN, _parameters.rc_return_th,
					       _parameters.rc_return_inv);
			manual.loiter_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_LOITER, _parameters.rc_loiter_th,
					       _parameters.rc_loiter_inv);
			manual.acro_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ACRO, _parameters.rc_acro_th,
					     _parameters.rc_acro_inv);
			manual.offboard_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_OFFBOARD,
						 _parameters.rc_offboard_th, _parameters.rc_offboard_inv);
			manual.kill_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_KILLSWITCH,
					     _parameters.rc_killswitch_th, _parameters.rc_killswitch_inv);
			manual.arm_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_ARMSWITCH,
					    _parameters.rc_armswitch_th, _parameters.rc_armswitch_inv);
			manual.transition_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_TRANSITION,
						   _parameters.rc_trans_th, _parameters.rc_trans_inv);
			manual.gear_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_GEAR,
					     _parameters.rc_gear_th, _parameters.rc_gear_inv);
			manual.stab_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_STAB,
					     _parameters.rc_stab_th, _parameters.rc_stab_inv);
			manual.man_switch = get_rc_sw2pos_position(rc_channels_s::RC_CHANNELS_FUNCTION_MAN,
					    _parameters.rc_man_th, _parameters.rc_man_inv);

			/* publish manual_control_setpoint topic */
			orb_publish_auto(ORB_ID(manual_control_setpoint), &_manual_control_pub, &manual, &instance,
					 ORB_PRIO_HIGH);

			// Control Group #3 (Manual Passthrough)
			/* copy from mapped manual control to control group 3 */
			struct actuator_controls_s actuator_group_3 = {};

			actuator_group_3.timestamp = rc_input.timestamp_last_signal;

			actuator_group_3.control[0] = manual.y;
			actuator_group_3.control[1] = manual.x;
			actuator_group_3.control[2] = manual.r;
			actuator_group_3.control[3] = manual.z;
			actuator_group_3.control[4] = manual.flaps;
			actuator_group_3.control[5] = manual.aux1;
			actuator_group_3.control[6] = manual.aux2;
			actuator_group_3.control[7] = manual.aux3;

			/* publish actuator_controls_3 topic */
			orb_publish_auto(ORB_ID(actuator_controls_3), &_actuator_group_3_pub, &actuator_group_3, &instance,
					 ORB_PRIO_DEFAULT);

			/* Update parameters from RC Channels (tuning with RC) if activated */
			if (hrt_elapsed_time(&_last_rc_to_param_map_time) > 1e6) { //1초에 1번 rc로 param설정하는 수행
				set_params_from_rc(parameter_handles);
				_last_rc_to_param_map_time = hrt_absolute_time();
			}
		}
	}
}
