/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file rcloss.cpp
 * Helper class for RC Loss Mode according to the OBC rules
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>

#include <uORB/uORB.h>
#include <navigator/navigation.h>
#include <uORB/topics/home_position.h>

#include "navigator.h"
#include "datalinkloss.h"

RCLoss::RCLoss(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator),
	_rcl_state(RCL_STATE_NONE)
{
}

void
RCLoss::on_inactive()
{
	// 정상으로 돌아온 경우에만 NONE으로 설정
	// sp가 변경된 경우 RCL 상태를 초기화
	/* reset RCL state only if setpoint moved */
	if (!_navigator->get_can_loiter_at_sp()) {
		_rcl_state = RCL_STATE_NONE;
	}
}

void
RCLoss::on_activation()
{
	// 시작은 none 상태로 초기화
	_rcl_state = RCL_STATE_NONE;
	advance_rcl();
	set_rcl_item();
}

void
RCLoss::on_active()
{
	if (is_mission_item_reached()) {
		advance_rcl();
		set_rcl_item();
	}
}

// rcl state에 따라서 mission item 채우기(설정하는 경우는 loiter와 terminate 2개 밖에 없음) 
void
RCLoss::set_rcl_item()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->previous = pos_sp_triplet->current;
	_navigator->set_can_loiter_at_sp(false);

	switch (_rcl_state) {
	case RCL_STATE_LOITER: {  //현재 고도에서 loiter하게
			_mission_item.lat = _navigator->get_global_position()->lat;
			_mission_item.lon = _navigator->get_global_position()->lon;
			_mission_item.altitude = _navigator->get_global_position()->alt;  // loiter의 고도는 현재 고도로 설정
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = NAN;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = _param_loitertime.get() < 0.0f ? 0.0f : _param_loitertime.get(); //loiter 시간을 설정
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);
			break;
		}

	case RCL_STATE_TERMINATE: { // 비행 종료 요청. 모든 ps의 valid를 false로 설정
			/* Request flight termination from the commander */
			_navigator->get_mission_result()->flight_termination = true;
			_navigator->set_mission_result_updated();
			warnx("rc not recovered: request flight termination");
			pos_sp_triplet->previous.valid = false;
			pos_sp_triplet->current.valid = false;
			pos_sp_triplet->next.valid = false;
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	// 설정한 mission item으로 sp 설정
	/* convert mission item to current position setpoint and make it valid */
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_position_setpoint_triplet_updated();
}

// 현재 state 기반으로 다음 state 결정 
void
RCLoss::advance_rcl()
{
	switch (_rcl_state) {
	case RCL_STATE_NONE: // loiter 시간이 설정되어 있으면 loiter 모드로 해당 시간동안 loiter, 그렇지 않은 경우 terminate 상태로 설정
		if (_param_loitertime.get() > 0.0f) { // loiter 시간이 설정된 경우 해당 시간 동안 loiter
			warnx("RC loss, OBC mode, loiter");
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "rc loss, loitering");
			_rcl_state = RCL_STATE_LOITER;

		} else {
			warnx("RC loss, OBC mode, slip loiter, terminate");
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "rc loss, terminating");
			_rcl_state = RCL_STATE_TERMINATE;
			_navigator->get_mission_result()->stay_in_failsafe = true;
			_navigator->set_mission_result_updated();
			reset_mission_item_reached();
		}

		break;

	case RCL_STATE_LOITER:  //loiter 모드로 설정 시간동안 동작
		_rcl_state = RCL_STATE_TERMINATE;
		warnx("time is up, no RC regain, terminating");
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RC not regained, terminating");
		_navigator->get_mission_result()->stay_in_failsafe = true;
		_navigator->set_mission_result_updated();
		reset_mission_item_reached();
		break;

	case RCL_STATE_TERMINATE:  //비행 종료 요청 후 rc loss 자동 모드 종료
		warnx("rcl end");
		_rcl_state = RCL_STATE_END;
		break;

	default:
		break;
	}
}
