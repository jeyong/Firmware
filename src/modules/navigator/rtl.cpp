/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file navigator_rtl.cpp
 * Helper class to access RTL
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "rtl.h"
#include "navigator.h"

#include <cfloat>

#include <mathlib/mathlib.h>
#include <systemlib/mavlink_log.h>

using math::max;
using math::min;

static constexpr float DELAY_SIGMA = 0.01f;

RTL::RTL(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
RTL::on_inactive()
{
	// reset RTL state
	_rtl_state = RTL_STATE_NONE;
}

int
RTL::rtl_type() const
{
	// home으로 갈 것인가 mission의 착륙지점으로 갈 것인지 설정
	return _param_rtl_type.get();
}

void
RTL::on_activation()
{
	// 이미 착륙한 상태면 RTL_STATE_LANDED 상태로 설정
	if (_navigator->get_land_detected()->landed) {
		// for safety reasons don't go into RTL if landed
		_rtl_state = RTL_STATE_LANDED;

	} else if ((rtl_type() == RTL_LAND) && _navigator->on_mission_landing()) { // RTL_LAND는 mission의 착륙지점으로 가는 것이므로 mission에서 처리
		// RTL straight to RETURN state, but mission will takeover for landing

	} else if ((_navigator->get_global_position()->alt < _navigator->get_home_position()->alt + _param_return_alt.get())
		   || _rtl_alt_min) {
		// RTL 고도보다 낮은 경우에 일단 RTL 고도까지 올라간다.
		// if lower than return altitude, climb up first
		// if rtl_alt_min is true then forcing altitude change even if above
		_rtl_state = RTL_STATE_CLIMB;

	} else {
		// otherwise go straight to return
		_rtl_state = RTL_STATE_RETURN;
	}

	set_rtl_item();
}

void
RTL::on_active()
{
	if (_rtl_state != RTL_STATE_LANDED && is_mission_item_reached()) {
		advance_rtl();
		set_rtl_item();
	}
}

// 최소 고도 설정
void
RTL::set_return_alt_min(bool min)
{
	_rtl_alt_min = min;
}

// RTL_LAND
void
RTL::set_rtl_item()
{
	// RTL_TYPE: mission landing
	// landing using planned mission landing, fly to DO_LAND_START instead of returning HOME
	// do nothing, let navigator takeover with mission landing
	// DO_LAND_START와 같이 Mission으로 실행하는 경우 navigator가 mission landing으로 처리하게 그냥 return 
	if (rtl_type() == RTL_LAND) { // mission에서 처리
		if (_rtl_state > RTL_STATE_CLIMB) {
			if (_navigator->start_mission_landing()) {  // mission으로 착륙하는 경우 동작 안함. (navigator에서 처리.)
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: using mission landing");
				return;

			} else {
				// otherwise use regular RTL
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "RTL: unable to use mission landing");
			}
		}
	}

	_navigator->set_can_loiter_at_sp(false);

	const home_position_s &home = *_navigator->get_home_position();
	const vehicle_global_position_s &gpos = *_navigator->get_global_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// 현재 위치와 home과의 거리 구하기
	// check if we are pretty close to home already
	const float home_dist = get_distance_to_next_waypoint(home.lat, home.lon, gpos.lat, gpos.lon);

	// return시 고도 계산
	// compute the return altitude
	float return_alt = max(home.alt + _param_return_alt.get(), gpos.alt);

	// home과 충분히 가까운 경우 : 최소 return 고도로 설정
	// we are close to home, limit climb to min
	if (home_dist < _param_rtl_min_dist.get()) {
		return_alt = home.alt + _param_descend_alt.get();
	}

	// loiter 고도 계산
	// compute the loiter altitude
	const float loiter_altitude = min(home.alt + _param_descend_alt.get(), gpos.alt);

	// 아래 각 state에서 사용하는 변수값을 사전에 다 계산하고 아래 상태에 따라 mission item 설정.
	switch (_rtl_state) {
	case RTL_STATE_CLIMB: { //RTL을 위해 상승하는 상태로 return_alt로 상승 고도 설정하는 것이 핵심. 

			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = gpos.lat;
			_mission_item.lon = gpos.lon;
			_mission_item.altitude = return_alt; // RTL을 위해서 올라가는 고도
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = NAN;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: climb to %d m (%d m above home)",
						     (int)ceilf(return_alt), (int)ceilf(return_alt - _navigator->get_home_position()->alt));
			break;
		}

	case RTL_STATE_RETURN: {
			// 이전 상태 대비 고도는(returl_alt 사용) 바꾸지 않고 lat, lon만 home 위치로 설정
			// don't change altitude
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = return_alt;
			_mission_item.altitude_is_relative = false;

			// home이랑 충분히 가까운 위치라면 yaw만 home yaw로 설정.
			// use home yaw if close to home
			/* check if we are pretty close to home already */
			if (home_dist < _param_rtl_min_dist.get()) {
				_mission_item.yaw = home.yaw;

			} else { // home이랑 떨어져 있다면 현재 위치와 home 위치를 이용해서 home을 향하도록 yaw를 계산
				// use current heading to home
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, home.lat, home.lon);
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: return at %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt));

			break;
		}
	//VTOL 무시
	case RTL_STATE_TRANSITION_TO_MC: {
			set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);
			break;
		}

	case RTL_STATE_DESCEND: { //return_alt에서 loiter_alt 까지 하강하는 동작.
			_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = loiter_altitude;  // 고도를 loiter 고도로 설정
			_mission_item.altitude_is_relative = false;

			// except for vtol which might be still off here and should point towards this location
			// VTOL인 경우 무시
			const float d_current = get_distance_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			if (_navigator->get_vstatus()->is_vtol && (d_current > _navigator->get_acceptance_radius())) {
				_mission_item.yaw = get_bearing_to_next_waypoint(gpos.lat, gpos.lon, _mission_item.lat, _mission_item.lon);

			} else { // 이미 도착해서 하강하는 시점이므로 home.yaw로 설정
				_mission_item.yaw = home.yaw;  //home yaw로 설정
			}

			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			/* disable previous setpoint to prevent drift */
			pos_sp_triplet->previous.valid = false;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: descend to %d m (%d m above home)",
						     (int)ceilf(_mission_item.altitude), (int)ceilf(_mission_item.altitude - home.alt));
			break;
		}

	case RTL_STATE_LOITER: {
			// land_delay는 하강 이후 착륙까지 지연 시간 설정. 0보다 작은 값인 경우 loiter 유지로 설정됨. 따라서 0보다 큰 값에서는 지연시간 이후 다음 동작 계속 진행하도록 설정하기 위한 변수로 사용
			const bool autoland = (_param_land_delay.get() > FLT_EPSILON);

			// don't change altitude
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.altitude = loiter_altitude; //loiter 고도로 설정
			_mission_item.altitude_is_relative = false;
			_mission_item.yaw = home.yaw;
			_mission_item.loiter_radius = _navigator->get_loiter_radius();
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = max(_param_land_delay.get(), 0.0f);
			_mission_item.autocontinue = autoland; // loiter 후 다음 동작 자동 실행
			_mission_item.origin = ORIGIN_ONBOARD;

			_navigator->set_can_loiter_at_sp(true);

			// loiter 시간이 설정되어 있는 경우 해당 시간까지 loiter 하고 다음 동작 진행
			if (autoland && (get_time_inside(_mission_item) > FLT_EPSILON)) {
				_mission_item.nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: loiter %.1fs",
							     (double)get_time_inside(_mission_item));

			} else {  // autoland값의 정의에 따라 loiter 상태를 유지.
				_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
				mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: completed, loitering");
			}

			break;
		}

	case RTL_STATE_LAND: { // home 위치에 착륙. 고도는 home의 높이로 설정.
			// land at home position
			_mission_item.nav_cmd = NAV_CMD_LAND;
			_mission_item.lat = home.lat;
			_mission_item.lon = home.lon;
			_mission_item.yaw = home.yaw;
			_mission_item.altitude = home.alt;  // home 고도로 고도 설정하여 착륙하도록 
			_mission_item.altitude_is_relative = false;
			_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
			_mission_item.time_inside = 0.0f;
			_mission_item.autocontinue = true;
			_mission_item.origin = ORIGIN_ONBOARD;

			mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "RTL: land at home");
			break;
		}

	case RTL_STATE_LANDED: { //착륙한 상태로 idle 상태로 설정
			set_idle_item(&_mission_item);
			set_return_alt_min(false);
			break;
		}

	default:
		break;
	}

	reset_mission_item_reached();

	//vtol 관련 skip
	/* execute command if set. This is required for commands like VTOL transition */
	if (!item_contains_position(_mission_item)) {
		issue_command(_mission_item);
	}

	// 설정한 mission item으로 pos sp 설정
	/* convert mission item to current position setpoint and make it valid */
	mission_apply_limitation(_mission_item);

	if (mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current)) {
		_navigator->set_position_setpoint_triplet_updated();
	}
}

// RTL state의 현재 state 기반으로 조건에 따라 다음 state로 전환시키기
void
RTL::advance_rtl()
{
	switch (_rtl_state) {
	case RTL_STATE_CLIMB:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_RETURN:
		_rtl_state = RTL_STATE_DESCEND;
		// VTOL 경우 무시
		if (_navigator->get_vstatus()->is_vtol && !_navigator->get_vstatus()->is_rotary_wing) {
			_rtl_state = RTL_STATE_TRANSITION_TO_MC;
		}

		break;

	case RTL_STATE_TRANSITION_TO_MC:
		_rtl_state = RTL_STATE_RETURN;
		break;

	case RTL_STATE_DESCEND:

		// 지연시간이 유효한 범위면 일단 loiter 상태로 빠지게 하고 추후 autoland 값에 따라서 다음 착륙 상태로 가든지 그냥 loiter로 계속 머물든지 하게 됨.
		/* only go to land if autoland is enabled */
		if (_param_land_delay.get() < -DELAY_SIGMA || _param_land_delay.get() > DELAY_SIGMA) {
			_rtl_state = RTL_STATE_LOITER;

		} else { //저기 범위를 벗어나는 경우에는 그냥 LAND로 
			_rtl_state = RTL_STATE_LAND;
		}

		break;

	case RTL_STATE_LOITER:
		_rtl_state = RTL_STATE_LAND;
		break;

	case RTL_STATE_LAND:
		_rtl_state = RTL_STATE_LANDED;
		break;

	default:
		break;
	}
}
