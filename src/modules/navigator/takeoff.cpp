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
 * @file Takeoff.cpp
 *
 * Helper class to Takeoff
 *
 * @author Lorenz Meier <lorenz@px4.io
 */

#include "takeoff.h"
#include "navigator.h"

Takeoff::Takeoff(Navigator *navigator) :
	MissionBlock(navigator)
{
}

void
Takeoff::on_activation()
{
	set_takeoff_position();
}

void
Takeoff::on_active()
{
	//rep는 언제 valid한가? (rep의 경우 요청한 값이다.)
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	if (rep->current.valid) {
		// reset the position
		set_takeoff_position();

	} else if (is_mission_item_reached() && !_navigator->get_mission_result()->finished) {
	// takeoff 위치에 도달하면 loiter가 되도록 mission item 만들어서 mission으로 적용. 현재 position을 sp로 지정.
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		//loiter mission item 생성
		// set loiter item so position controllers stop doing takeoff logic
		set_loiter_item(&_mission_item);
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}
}

// takeoff 고도를 정해서 mission item 만들기. 
// misstion item로 setpoint 채우기
void
Takeoff::set_takeoff_position()
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	float abs_altitude = 0.0f;

	float min_abs_altitude;

	// takeoff 고도 정하기 
	if (_navigator->home_position_valid()) { //only use home position if it is valid
		min_abs_altitude = _navigator->get_global_position()->alt + _navigator->get_takeoff_min_alt();

	} else { //e.g. flow
		min_abs_altitude = _navigator->get_takeoff_min_alt();
	}

	// home position이 유효한 경우 위에서 정한 alt로 설정
	// Use altitude if it has been set. If home position is invalid use min_abs_altitude
	if (rep->current.valid && PX4_ISFINITE(rep->current.alt) && _navigator->home_position_valid()) {
		abs_altitude = rep->current.alt;

		// rep의 고도와 위에서 설정한 고도 중에서 큰 값을 선택. rep의 고도가 작은 경우 이를 알린다. (rep의 경우 요청한 값이므로 이 요청이 받아들이지 않은 경우 알려줘야 한다.)
		// If the altitude suggestion is lower than home + minimum clearance, raise it and complain.
		if (abs_altitude < min_abs_altitude) {
			if (abs_altitude < min_abs_altitude - 0.1f) { // don't complain if difference is smaller than 10cm
				mavlink_log_critical(_navigator->get_mavlink_log_pub(),
						     "Using minimum takeoff altitude: %.2f m", (double)_navigator->get_takeoff_min_alt());
			}

			abs_altitude = min_abs_altitude;
		}

	} else {
		// rep가 유효하지 않은 경우 
		// Use home + minimum clearance but only notify.
		abs_altitude = min_abs_altitude;
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
				 "Using minimum takeoff altitude: %.2f m", (double)_navigator->get_takeoff_min_alt());
	}

	//takeoff할 고도보다 현재 고도가 더 높다면 현재 고도로 (그렇지 않으면 아래로 내려가게 되니까.)
	if (abs_altitude < _navigator->get_global_position()->alt) {
		// If the suggestion is lower than our current alt, let's not go down.
		abs_altitude = _navigator->get_global_position()->alt;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Already higher than takeoff altitude");
	}

	//위에서 정한 고도로 mission item 설정.
	// set current mission item to takeoff
	set_takeoff_item(&_mission_item, abs_altitude);
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	// mission item으로 sp 설정
	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->previous.valid = false;
	pos_sp_triplet->current.yaw_valid = true;
	pos_sp_triplet->next.valid = false;

	// reposition 요청이 있었으면 reposition의 값을 이용해서 sp 설정. (reposition 우선순위가 높음)
	if (rep->current.valid) {

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		if (PX4_ISFINITE(rep->current.lat) && PX4_ISFINITE(rep->current.lon)) {
			pos_sp_triplet->current.lat = rep->current.lat;
			pos_sp_triplet->current.lon = rep->current.lon;
		}

		// mark this as done
		memset(rep, 0, sizeof(*rep));
	}

	_navigator->set_can_loiter_at_sp(true);

	_navigator->set_position_setpoint_triplet_updated();
}
