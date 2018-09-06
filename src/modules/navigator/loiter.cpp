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
 * @file loiter.cpp
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "loiter.h"
#include "navigator.h"

Loiter::Loiter(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
Loiter::on_inactive()
{
	_loiter_pos_set = false;
}

//reposition 명령이 들어온 경우 reposition 위치에서 loiter하고 아니면 
void
Loiter::on_activation()
{
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition();

	} else {
		set_loiter_position();
	}
}

//reposition 명령이 들어온 경우 reposition 위치에서 loiter하고 아니면 
void
Loiter::on_active()
{
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition();
	}

	// armed 상태가 아니면 loiter pos를 설정이 안되어 있는 상태로 표시
	// reset the loiter position if we get disarmed
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_loiter_pos_set = false;
	}
}

// mission item에 현재 위치로 loiter하게 설정
void
Loiter::set_loiter_position()
{
	// arming도 안되어 있고 착륙 상태인 경우 idle 상태로 설정
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED &&
	    _navigator->get_land_detected()->landed) {

		// Not setting loiter position if disarmed and landed, instead mark the current
		// setpoint as invalid and idle (both, just to be sure).

		_navigator->set_can_loiter_at_sp(false);
		_navigator->get_position_setpoint_triplet()->current.type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		_navigator->set_position_setpoint_triplet_updated();
		_loiter_pos_set = false;
		return;

	} else if (_loiter_pos_set) { // 이미 loiter pos가 설정되어 있는 경우라면 아래 동작을 할 필요없음.
		// Already set, nothing to do.
		return;
	}

	_loiter_pos_set = true;

	// 실제로 mission item 채우기. 2번째 인자는 최소 마진
	// set current mission item to loiter
	set_loiter_item(&_mission_item, _navigator->get_loiter_min_alt());

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.velocity_valid = false;
	pos_sp_triplet->previous.valid = false;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

	_navigator->set_position_setpoint_triplet_updated();
}

void
Loiter::reposition()
{
	// armed 상태가 아니면 repositon을 수행할 수 없음
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	// rep가 유효한 값으로 채워져있다면 rep 위치에서 loiter 하도록 함
	struct position_setpoint_triplet_s *rep = _navigator->get_reposition_triplet();

	if (rep->current.valid) {
		// set loiter position based on reposition command

		// rep의 위치 정보를 pos sp에 복사. 현재 위치는 pos sp의 prev에 넣고. 
		// convert mission item to current setpoint
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.velocity_valid = false;
		pos_sp_triplet->previous.yaw = _navigator->get_global_position()->yaw;
		pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
		memcpy(&pos_sp_triplet->current, &rep->current, sizeof(rep->current));
		pos_sp_triplet->next.valid = false;

		// 설정한 yaw 모드에 따라 현재 yaw 유지할지 아니면 rep 기준으로 yaw를 변경할지 결정. 
		// set yaw (depends on the value of parameter MIS_YAWMODE):
		// MISSION_YAWMODE_NONE: do not change yaw setpoint
		// MISSION_YAWMODE_FRONT_TO_WAYPOINT: point to next waypoint
		if (_param_yawmode.get() != MISSION_YAWMODE_NONE) { //yaw를 변경해야하는 경우
			float travel_dist = get_distance_to_next_waypoint(_navigator->get_global_position()->lat,
					    _navigator->get_global_position()->lon,
					    pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

			if (travel_dist > 1.0f) { // 1m 이상 이동해야하는 경우에 yaw를 변경. 
				// calculate direction the vehicle should point to.
				pos_sp_triplet->current.yaw = get_bearing_to_next_waypoint(
								      _navigator->get_global_position()->lat,
								      _navigator->get_global_position()->lon,
								      pos_sp_triplet->current.lat,
								      pos_sp_triplet->current.lon);
			}
		}
		// loiter 모드로 정상적으로 설정하고 pos sp 업데이트 설정
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		_navigator->set_position_setpoint_triplet_updated();

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}
}
