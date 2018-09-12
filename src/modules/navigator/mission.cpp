/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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
 * @file navigator_mission.cpp
 *
 * Helper class to access missions
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author Simon Wilks <simon@uaventure.com>
 * @author Andreas Antener <andreas@uaventure.com>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "mission.h"
#include "navigator.h"

#include <string.h>
#include <drivers/drv_hrt.h>
#include <dataman/dataman.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <navigator/navigation.h>
#include <uORB/uORB.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

using matrix::wrap_pi;

Mission::Mission(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
Mission::on_inactive()
{
	// mission cruising speed는 리셋하지만 mission velocity는 RTL을 위해 설정될 수 있으므로 리셋하지 않는다. 
	/* We need to reset the mission cruising speed, otherwise the
	 * mission velocity which might have been set using mission items
	 * is used for missions such as RTL. */
	_navigator->set_cruising_speed();

	// home position 없으면 아래 동작 진입 불가. 
	/* Without home a mission can't be valid yet anyway, let's wait. */
	if (!_navigator->home_position_valid()) {
		return;
	}

	// 
	if (_inited) { //아래 초기화 한 번 하고 나서 부터는 이 부분이 실행
		// offboard mission이 들어온 것이 있는지 보고 mission이 존재하면 offboard 타입으로 지정
		bool offboard_updated = false;
		orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);

		if (offboard_updated) {
			update_offboard_mission();

			if (_mission_type == MISSION_TYPE_NONE && _offboard_mission.count > 0) {
				_mission_type = MISSION_TYPE_OFFBOARD;
			}
		}

		// 현재 offboard 미션을 reset 필요성 여부 체크해서 
		/* reset the current offboard mission if needed */
		if (need_to_reset_mission(false)) {
			reset_offboard_mission(_offboard_mission);
			update_offboard_mission();
			_navigator->reset_cruising_speed();
		}

	} else { //처음 실행 되는 부분
		// mission_state을 dm에서 읽어와서 mission이 들어있는 dm 주소나 실행 index 등의 정보를 수집.
		/* load missions from storage */
		mission_s mission_state = {};

		dm_lock(DM_KEY_MISSION_STATE);

		/* read current state */
		int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		dm_unlock(DM_KEY_MISSION_STATE);

		if (read_res == sizeof(mission_s)) {
			_offboard_mission.dataman_id = mission_state.dataman_id;
			_offboard_mission.count = mission_state.count;
			_current_offboard_mission_index = mission_state.current_seq;

			// find and store landing start marker (if available)
			find_offboard_land_start();
		}

		// 실행할 misstion이 정상적인지 체크
		/* On init let's check the mission, maybe there is already one available. */
		check_mission_valid(false);

		_inited = true;
	}

	/* require takeoff after non-loiter or landing */
	if (!_navigator->get_can_loiter_at_sp() || _navigator->get_land_detected()->landed) {
		_need_takeoff = true;
	}

	/* reset so current mission item gets restarted if mission was paused */
	_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	/* reset so MISSION_ITEM_REACHED isn't published */
	_navigator->get_mission_result()->seq_reached = -1;
}

void
Mission::on_inactivation()
{
	// 카메라 관련 trigger를 pause 하도록 설정
	// Disable camera trigger
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// Pause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 1.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

//처음 mission을 실행할때 waypoint 변경이 여부를 확인하고 
void
Mission::on_activation()
{
	if (_mission_waypoints_changed) { // 수행할 mission index 설정.
		//nomal 모드가 아닌 경우에는 그냥 가장 가까운 mission을 실행 시킨다.
		// do not set the closest mission item in the normal mission mode
		if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_NORMAL) {
			_current_offboard_mission_index = index_closest_mission_item();
		}

		_mission_waypoints_changed = false;
	}

	// we already reset the mission items
	_execution_mode_changed = false;

	//dm에서 mission을 가져와서 mission item을 설정
	set_mission_items();

	// 비행체에 있는 카메라 동작되도록 설정
	// unpause triggering if it was paused
	vehicle_command_s cmd = {};
	cmd.command = vehicle_command_s::VEHICLE_CMD_DO_TRIGGER_CONTROL;
	// unpause trigger
	cmd.param1 = -1.0f;
	cmd.param3 = 0.0f;
	_navigator->publish_vehicle_cmd(&cmd);
}

void
Mission::on_active()
{
	check_mission_valid(false);

	/* check if anything has changed */
	bool offboard_updated = false;
	//offboard mission 변경 체크
	orb_check(_navigator->get_offboard_mission_sub(), &offboard_updated);

	if (offboard_updated) {
		update_offboard_mission();
	}

	/* reset the current offboard mission if needed */
	if (need_to_reset_mission(true)) {
		reset_offboard_mission(_offboard_mission);
		update_offboard_mission();
		_navigator->reset_cruising_speed();
		offboard_updated = true;
	}

	_mission_changed = false;

	/* reset mission items if needed */
	if (offboard_updated || _mission_waypoints_changed || _execution_mode_changed) {
		if (_mission_waypoints_changed) {
			// do not set the closest mission item in the normal mission mode
			if (_mission_execution_mode != mission_result_s::MISSION_EXECUTION_MODE_NORMAL) {
				_current_offboard_mission_index = index_closest_mission_item();
			}

			_mission_waypoints_changed = false;
		}

		_execution_mode_changed = false;
		set_mission_items();
	}

	// 현재 mission이 완료되었는지 체크
	/* lets check if we reached the current mission item */
	if (_mission_type != MISSION_TYPE_NONE && is_mission_item_reached()) {
		/* If we just completed a takeoff which was inserted before the right waypoint,
		   there is no need to report that we reached it because we didn't. */
		if (_work_item_type != WORK_ITEM_TYPE_TAKEOFF) {
			set_mission_item_reached();
		}

		if (_mission_item.autocontinue) {
			//autocontinue가 설정되어 있으면 dm에서 다은 mission 꺼내와서 mission item 설정
			/* switch to next waypoint if 'autocontinue' flag set */
			advance_mission();
			set_mission_items();
		}

	} else if (_mission_type != MISSION_TYPE_NONE && _param_altmode.get() == MISSION_ALTMODE_FOH) {
		//foh의 경우 linear하게 고도 상승을 위해 alt sp값을 설정
		altitude_sp_foh_update();

	} else {
		//waypoint 위치에 도착하면 loiter할 수 있도록 설정.
		/* if waypoint position reached allow loiter on the setpoint */
		if (_waypoint_position_reached && _mission_item.nav_cmd != NAV_CMD_IDLE) {
			_navigator->set_can_loiter_at_sp(true);
		}
	}

	/* check if a cruise speed change has been commanded */
	if (_mission_type != MISSION_TYPE_NONE) {
		cruising_speed_sp_update();
	}
	// ROI가 설정된 경우, heading의 sp를 변경
	/* see if we need to update the current yaw heading */
	if (_navigator->get_vroi().mode == vehicle_roi_s::ROI_LOCATION
	    || (_param_yawmode.get() != MISSION_YAWMODE_NONE
		&& _param_yawmode.get() < MISSION_YAWMODE_MAX
		&& _mission_type != MISSION_TYPE_NONE)
	    || _navigator->get_vstatus()->is_vtol) {

		heading_sp_update();
	}

	// fw인 경우 skip - LAND 명령으로 동작 중에 landing 취소 상태가 되면 landing 취소 동작
	/* check if landing needs to be aborted */
	if ((_mission_item.nav_cmd == NAV_CMD_LAND)
	    && (_navigator->abort_landing())) {

		do_abort_landing();
	}
	//precland 동작인 경우 precland->on_active() 동작
	if (_work_item_type == WORK_ITEM_TYPE_PRECISION_LAND) {
		// switch out of precision land once landed
		if (_navigator->get_land_detected()->landed) {
			_navigator->get_precland()->on_inactivation();
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;

		} else {
			_navigator->get_precland()->on_active();
		}
	}
}

//index를 인자로 주면, 해당 index가 유효한 경우 해당 index의 mission으로 mission item을 설정한다. 
bool
Mission::set_current_offboard_mission_index(uint16_t index)
{
	if (_navigator->get_mission_result()->valid &&
	    (index != _current_offboard_mission_index) && (index < _offboard_mission.count)) {

		_current_offboard_mission_index = index;

		// a mission offboard index is set manually which has the higher priority than the closest mission item
		// as it is set by the user
		_mission_waypoints_changed = false;

		// 현재 mission 모드로 동작 중이라면 인자로 받은 index의 mission이 구동될 수 있도록 set_mission_items()를 호출하면 됨.
		// update mission items if already in active mission
		if (_navigator->is_planned_mission()) {
			// prevent following "previous - current" line
			_navigator->get_position_setpoint_triplet()->previous.valid = false;
			_navigator->get_position_setpoint_triplet()->current.valid = false;
			_navigator->get_position_setpoint_triplet()->next.valid = false;
			set_mission_items();
		}

		return true;
	}

	return false;
}

//현재 mission index를 dm에 있는 mission들 중에서 현재 위치와 가장 가까운 mission의 index로 설정
void
Mission::set_closest_item_as_current()
{
	_current_offboard_mission_index = index_closest_mission_item();
}

//실행 모드 MISSION_EXECUTION_MODE_NORMAL, MISSION_EXECUTION_MODE_FAST_FORWARD, MISSION_EXECUTION_MODE_REVERSE를 인자로 지정
// 핵심은 _current_offboard_mission_index을 1 감소 혹은 증가 시킨다.
void
Mission::set_execution_mode(const uint8_t mode)
{
	if (_mission_execution_mode != mode) {
		_execution_mode_changed = true;
		_navigator->get_mission_result()->execution_mode = mode;

		//reverse로 모드 전환시 _current_offboard_mission_index를 1 감소 시킴
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD:
			if (mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				// command a transition if in vtol mc mode
				if (_navigator->get_vstatus()->is_rotary_wing &&
				    _navigator->get_vstatus()->is_vtol &&
				    !_navigator->get_land_detected()->landed) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

					position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
					pos_sp_triplet->previous = pos_sp_triplet->current;
					generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
					_navigator->set_position_setpoint_triplet_updated();
					issue_command(_mission_item);
				}

				if (_mission_type == MISSION_TYPE_NONE && _offboard_mission.count > 0) {
					_mission_type = MISSION_TYPE_OFFBOARD;
				}

				if (_current_offboard_mission_index > _offboard_mission.count - 1) {
					_current_offboard_mission_index = _offboard_mission.count - 1;

				} else if (_current_offboard_mission_index > 0) {
					--_current_offboard_mission_index;
				}

				_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			}

			break;
		//reverse에서 일반 모드로 전환시 _current_offboard_mission_index를 1 증가
		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE:
			if ((mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL) ||
			    (mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD)) {
				// handle switch from reverse to forward mission
				if (_current_offboard_mission_index < 0) {
					_current_offboard_mission_index = 0;

				} else if (_current_offboard_mission_index < _offboard_mission.count - 1) {
					++_current_offboard_mission_index;
				}

				_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			}

			break;

		}

		_mission_execution_mode = mode;
	}
}

// dm의 mission 중에 착륙 관련 명령(NAV_CMD_LAND, MAV_CMD_DO_LAND_START)이 있는지 여부를 확인. _land_start_index 에 저장
bool
Mission::find_offboard_land_start()
{
	/* return true if a MAV_CMD_DO_LAND_START is found and internally save the index
	 *  return false if not found
	 *
	 * TODO: implement full spec and find closest landing point geographically
	 */

	const dm_item_t dm_current = (dm_item_t)_offboard_mission.dataman_id;

	for (size_t i = 0; i < _offboard_mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}

		if ((missionitem.nav_cmd == NAV_CMD_DO_LAND_START) ||
		    ((missionitem.nav_cmd == NAV_CMD_VTOL_LAND) && _navigator->get_vstatus()->is_vtol) ||
		    (missionitem.nav_cmd == NAV_CMD_LAND)) {
			_land_start_available = true;
			_land_start_index = i;
			return true;
		}
	}

	_land_start_available = false;
	return false;
}

//mission 중에 착륙관련 명령이 있는 경우 착륙 동작 수행
bool
Mission::land_start()
{
	// if not currently landing, jump to do_land_start
	if (_land_start_available) {
		if (landing()) {
			return true;

		} else {
			set_current_offboard_mission_index(get_land_start_index());
			return landing();
		}
	}

	return false;
}

//현재 비행체가 착륙 중인지 여부 확인 
bool
Mission::landing()
{
	// vehicle is currently landing if
	//  mission valid, still flying, and in the landing portion of mission

	const bool mission_valid = _navigator->get_mission_result()->valid;
	const bool on_landing_stage = _land_start_available && (_current_offboard_mission_index >= get_land_start_index());

	return mission_valid && on_landing_stage;
}

//mission을 새로 subscribe한 경우, 새로 받은 mission을 실행하기 위해서 mission item 설정 
void
Mission::update_offboard_mission()
{

	bool failed = true;

	/* reset triplets */
	_navigator->reset_triplets();

	struct mission_s old_offboard_mission = _offboard_mission;

	if (orb_copy(ORB_ID(mission), _navigator->get_offboard_mission_sub(), &_offboard_mission) == OK) {
		// current_seq가 유효한 값이면 이 값으로 _current_offboard_mission_index 를 설정
		/* determine current index */
		if (_offboard_mission.current_seq >= 0 && _offboard_mission.current_seq < (int)_offboard_mission.count) {
			_current_offboard_mission_index = _offboard_mission.current_seq;

		} else {
			// 유효하지 않은 범위인 경우 초기화
			/* if less items available, reset to first item */
			if (_current_offboard_mission_index >= (int)_offboard_mission.count) {
				_current_offboard_mission_index = 0;

			} else if (_current_offboard_mission_index < 0) {
				/* if not initialized, set it to 0 */
				_current_offboard_mission_index = 0;
			}

			/* otherwise, just leave it */
		}
		// 정상적인 처리가 가능한 mission인지 검사 
		check_mission_valid(true);
		// mission 유효성 검사 후 유효하지 않다고 판단된 경우 
		failed = !_navigator->get_mission_result()->valid;

		if (!failed) { // 유효한 경우, 초기화 
			/* reset mission failure if we have an updated valid mission */
			_navigator->get_mission_result()->failure = false;

			/* reset sequence info as well */
			_navigator->get_mission_result()->seq_reached = -1;
			_navigator->get_mission_result()->seq_total = _offboard_mission.count;

			/* reset work item if new mission has been accepted */
			_work_item_type = WORK_ITEM_TYPE_DEFAULT;
			_mission_changed = true;
		}

		// 비행 중에 waypoint 미션이 변경된 경우, 미션이 변경되었다는 _mission_waypoints_changed 변수를 설정
		/* check if the mission waypoints changed while the vehicle is in air
		 * TODO add a flag to mission_s which actually tracks if the position of the waypoint changed */
		if (((_offboard_mission.count != old_offboard_mission.count) ||
		     (_offboard_mission.dataman_id != old_offboard_mission.dataman_id)) &&
		    !_navigator->get_land_detected()->landed) {
			_mission_waypoints_changed = true;
		}

	} else {
		PX4_ERR("offboard mission update failed, handle: %d", _navigator->get_offboard_mission_sub());
	}

	// 유효한 mission이 아닌 경우 아래와 같이 초기화 
	if (failed) {
		_offboard_mission.count = 0;
		_offboard_mission.current_seq = 0;
		_current_offboard_mission_index = 0;

		PX4_ERR("mission check failed");
	}

	// 착륙 가능한 mission이 있는지 찾아놓기
	// find and store landing start marker (if available)
	find_offboard_land_start();

	// 수행할 mission item 설정
	set_current_offboard_mission_item();
}

//다음 mission으로 이동하기 위해서 _current_offboard_mission_index 증가/감소
void
Mission::advance_mission()
{
	/* do not advance mission item if we're processing sub mission work items */
	if (_work_item_type != WORK_ITEM_TYPE_DEFAULT) {
		return;
	}

	switch (_mission_type) {
	case MISSION_TYPE_OFFBOARD: // offboard 미션인 경우, 정상인 상황에서는 다음 mission을 찾기 위해 index만 증가
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				_current_offboard_mission_index++;
				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: { // reverse 모드인 경우 현재 index에서 감소시키면서 position 관련 mission이 있는지 확인
				// find next position item in reverse order
				dm_item_t dm_current = (dm_item_t)(_offboard_mission.dataman_id);

				for (int32_t i = _current_offboard_mission_index - 1; i >= 0; i--) {
					struct mission_item_s missionitem = {};
					const ssize_t len = sizeof(missionitem);

					if (dm_read(dm_current, i, &missionitem, len) != len) {
						/* not supposed to happen unless the datamanager can't access the SD card, etc. */
						PX4_ERR("dataman read failure");
						break;
					}
					// position 정보가 있는 mission item이면 index를 설정하고 종료
					if (item_contains_position(missionitem)) {
						_current_offboard_mission_index = i;
						return;
					}
				}
				// mission 종료
				// finished flying back the mission
				_current_offboard_mission_index = -1;
				break;
			}

		default: //기본은 index 증가해서 다음 mission 가져오기
			_current_offboard_mission_index++;
		}

		break;

	case MISSION_TYPE_NONE:
	default:
		break;
	}
}

void
Mission::set_mission_items()
{
	// foh(first order)에서 사용하는 목적의 변수 초기화
	/* reset the altitude foh (first order hold) logic, if altitude foh is enabled (param) a new foh element starts now */
	_min_current_sp_distance_xy = FLT_MAX;

	// QGC 사용자에게 상태 알림이 필요한 경우 알람을 수행했는지 여부 체크 변수
	/* the home dist check provides user feedback, so we initialize it to this */
	bool user_feedback_done = false;

	// 위치 정보를 가지는 다음 mission item
	/* mission item that comes after current if available */
	struct mission_item_s mission_item_next_position;
	bool has_next_position_item = false;

	work_item_type new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

	// 현재 misson item과 다음 position을 가지는 item을 얻어오기
	if (prepare_mission_items(&_mission_item, &mission_item_next_position, &has_next_position_item)) {
		// mission type이 다른 타입에서 offboard로 변경된 경우에 대해서 알림. 
		/* if mission type changed, notify */
		if (_mission_type != MISSION_TYPE_OFFBOARD) {
			mavlink_log_info(_navigator->get_mavlink_log_pub(),
					 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Executing Reverse Mission" :
					 "Executing Mission");
			user_feedback_done = true; // 알림 완료 설정
		}
		//offboard로 설정
		_mission_type = MISSION_TYPE_OFFBOARD;

	} else { // mission이 유효하지 않거나 종료된 경우 일 수 있으므로 loiter 모드로 변경하고 빠져나감.
		/* no mission available or mission finished, switch to loiter */
		if (_mission_type != MISSION_TYPE_NONE) {
			// 착륙한 상태면 로그
			if (_navigator->get_land_detected()->landed) {
				mavlink_log_info(_navigator->get_mavlink_log_pub(),
						 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Reverse Mission finished, landed" :
						 "Mission finished, landed.");

			} else { // 착륙한 상태가 아니면 loiter 가능 모드로 설정
				/* https://en.wikipedia.org/wiki/Loiter_(aeronautics) */
				mavlink_log_info(_navigator->get_mavlink_log_pub(),
						 _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE ? "Reverse Mission finished, loitering" :
						 "Mission finished, loitering.");

				/* use last setpoint for loiter */
				_navigator->set_can_loiter_at_sp(true);
			}

			user_feedback_done = true;
		}
		// mission type을 None으로 설정
		_mission_type = MISSION_TYPE_NONE;

		// loiter mission 설정
		/* set loiter mission item and ensure that there is a minimum clearance from home */
		set_loiter_item(&_mission_item, _navigator->get_takeoff_min_alt());

		/* update position setpoint triplet  */
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->previous.valid = false;
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
		pos_sp_triplet->next.valid = false;

		// IDLE 상태(착륙상태)가 아닌 경우 loiter 가능 상태로 설정
		/* reuse setpoint for LOITER only if it's not IDLE */
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		// set mission finished
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		if (!user_feedback_done) { // mission item 수행하면서 최소한 사용자에게 정보를 알려주기 위해서. 여기까지 정보 알림이 없었다면 아래 정보 제공하도록
			/* only tell users that we got no mission if there has not been any
			 * better, more specific feedback yet
			 * https://en.wikipedia.org/wiki/Loiter_(aeronautics)
			 */

			if (_navigator->get_land_detected()->landed) {
				/* landed, refusing to take off without a mission */
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, refusing takeoff.");

			} else {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "No valid mission available, loitering.");
			}

			user_feedback_done = true;
		}

		_navigator->set_position_setpoint_triplet_updated();
		return;
	}

	//mission item 정상 수행하는 경우
	/*********************************** handle mission item *********************************************/

	/* handle mission items depending on the mode */

	const position_setpoint_s current_setpoint_copy = _navigator->get_position_setpoint_triplet()->current;

	//position 정보를 가진 mission item인 경우
	if (item_contains_position(_mission_item)) {
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				/* force vtol land */
				if (_navigator->force_vtol() && _mission_item.nav_cmd == NAV_CMD_LAND) {
					_mission_item.nav_cmd = NAV_CMD_VTOL_LAND;
				}

				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

				//지정한 sp로 이동을 위해서 수직 이륙이 필요한 경우라면 takeoff 부터 수행하도록. 
				/* do takeoff before going to setpoint if needed and not already in takeoff */
				/* in fixed-wing this whole block will be ignored and a takeoff item is always propagated */
				if (do_need_vertical_takeoff() &&
				    _work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					//takeoff로 지정
					new_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					// 원래 수행하기로 한 waypoit를 다음 mission으로 지정하고 takeoff mission을 생성해서 수행
					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					mission_item_next_position.nav_cmd = NAV_CMD_WAYPOINT;
					has_next_position_item = true;

					float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

					mavlink_log_info(_navigator->get_mavlink_log_pub(), "Takeoff to %.1f meters above home.",
							 (double)(takeoff_alt - _navigator->get_home_position()->alt));

					// takeoff mission 생성
					_mission_item.nav_cmd = NAV_CMD_TAKEOFF;
					_mission_item.lat = _navigator->get_global_position()->lat;
					_mission_item.lon = _navigator->get_global_position()->lon;
					/* hold heading for takeoff items */
					_mission_item.yaw = _navigator->get_global_position()->yaw;
					_mission_item.altitude = takeoff_alt;
					_mission_item.altitude_is_relative = false;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF //takeoff 명령인 경우
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && _navigator->get_vstatus()->is_rotary_wing) {

					// takeoff가 필요없는 경우면서 takeoff명령을 수행 하는 경우 그냥 takeoff를 waypoint 정보로 사용
					/* if there is no need to do a takeoff but we have a takeoff item, treat is as waypoint */
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
					/* since _mission_item.time_inside and and _mission_item.pitch_min build a union, we need to set time_inside to zero
					 * since in NAV_CMD_TAKEOFF mode there is currently no time_inside.
					 * Note also that resetting time_inside to zero will cause pitch_min to be zero as well.
					 */
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF //vtol 이륙 skip
					   && _work_item_type == WORK_ITEM_TYPE_DEFAULT
					   && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					if (_navigator->get_vstatus()->is_rotary_wing) {
						/* haven't transitioned yet, trigger vtol takeoff logic below */
						_work_item_type = WORK_ITEM_TYPE_TAKEOFF;

					} else {
						/* already in fixed-wing, go to waypoint */
						_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					}

					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
				}

				// 바로 이전에 이륙 명령을 수행한 경우, waypoint 명령으로 사용
				/* if we just did a normal takeoff navigate to the actual waypoint now */
				if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					/* ignore yaw here, otherwise it might yaw before heading_sp_update takes over */
					_mission_item.yaw = NAN;
					/* since _mission_item.time_inside and and _mission_item.pitch_min build a union, we need to set time_inside to zero
					 * since in NAV_CMD_TAKEOFF mode there is currently no time_inside.
					 */
					_mission_item.time_inside = 0.0f;
				}

				//VTOL 무시 
				/* if we just did a VTOL takeoff, prepare transition */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF &&
				    _work_item_type == WORK_ITEM_TYPE_TAKEOFF &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT &&
				    _navigator->get_vstatus()->is_rotary_wing &&
				    !_navigator->get_land_detected()->landed) {

					/* check if the vtol_takeoff waypoint is on top of us */
					if (do_need_move_to_takeoff()) {
						new_work_item_type = WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF;
					}

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_FW);

					/* set position setpoint to target during the transition */
					// TODO: if has_next_position_item and use_next set next, or if use_heading set generated
					generate_waypoint_from_heading(&pos_sp_triplet->current, _mission_item.yaw);
				}

				//VTOL 무시
				/* takeoff completed and transitioned, move to takeoff wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
				    && _work_item_type == WORK_ITEM_TYPE_TRANSITON_AFTER_TAKEOFF
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				// fw 무시 
				/* move to land wp as fixed wing */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && !_navigator->get_land_detected()->landed) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					float altitude = _navigator->get_global_position()->alt;

					if (pos_sp_triplet->current.valid && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
					_mission_item.vtol_back_transition = true;
				}

				// VTOL 무시
				/* transition to MC */
				if (_mission_item.nav_cmd == NAV_CMD_VTOL_LAND
				    && _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && !_navigator->get_vstatus()->is_rotary_wing
				    && !_navigator->get_land_detected()->landed) {

					set_vtol_transition_item(&_mission_item, vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC);

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION;
				}

				// 착륙지점으로 이동이 필요한 경우 하강하기 전에 착륙지점으로 waypoint 이동이 필요함.
				/* move to landing waypoint before descent if necessary */
				if (do_need_move_to_land() &&
				    (_work_item_type == WORK_ITEM_TYPE_DEFAULT ||
				     _work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND_AFTER_TRANSITION) &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_MOVE_TO_LAND;

					// 먼저 착륙지점 이동해야하니까 현재 mission은 다음에 동작으로 보관
					/* use current mission item as next position item */
					mission_item_next_position = _mission_item;
					has_next_position_item = true;

					// waypoint 고도 무시. 너무 빠르게 하강하는 것을 막기 위해서 동일한 고도로 설정. 
					/*
					 * Ignoring waypoint altitude:
					 * Set altitude to the same as we have now to prevent descending too fast into
					 * the ground. Actual landing will descend anyway until it touches down.
					 * XXX: We might want to change that at some point if it is clear to the user
					 * what the altitude means on this waypoint type.
					 */
					float altitude = _navigator->get_global_position()->alt;
					// 사용자의 고도 지정이 있었다면 해당 고도로 설정. 지정이 없으면 현재 고도를 사용.
					if (pos_sp_triplet->current.valid
					    && pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
						altitude = pos_sp_triplet->current.alt;
					}

					_mission_item.altitude = altitude;
					_mission_item.altitude_is_relative = false;
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else if (_mission_item.nav_cmd == NAV_CMD_LAND && _work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;
						// precland이 설정된 경우 precland 실행
						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}
				}
				// 착륙지점으로 이동했으면 이제 하강 수행
				/* we just moved to the landing waypoint, now descend */
				if (_work_item_type == WORK_ITEM_TYPE_MOVE_TO_LAND &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {
					// precland 모드면 precland 수행
					if (_mission_item.land_precision > 0 && _mission_item.land_precision < 3) {
						new_work_item_type = WORK_ITEM_TYPE_PRECISION_LAND;
						// 
						if (_mission_item.land_precision == 1) {
							_navigator->get_precland()->set_mode(PrecLandMode::Opportunistic);

						} else { //_mission_item.land_precision == 2
							_navigator->get_precland()->set_mode(PrecLandMode::Required);
						}

						_navigator->get_precland()->on_activation();

					}

				}
				// 착륙 명령시 yaw는 무시. 만약 heading이 필요한 경우면 하강 하기전에 비행체의 heading을 설정하는 단계 추가 가능.
				/* ignore yaw for landing items */
				/* XXX: if specified heading for landing is desired we could add another step before the descent
				 * that aligns the vehicle first */
				if (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND) {
					_mission_item.yaw = NAN;
				}

				// fast_forward 모드(loiter 하지 않고 waypoint 지점만 찍는 비행)인 경우 자동으로 계속 mission들 수행하도록 설정.
				// for fast forward convert certain types to simple waypoint
				// XXX: add other types which should be ignored in fast forward
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD &&
				    ((_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED) ||
				     (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT))) {
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}
		// reverse 모드인 경우 현재 mission item에 position정보가 있는 경우 waypoint 명령으로 연속 수행으로 설정
		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: {
				if (item_contains_position(_mission_item)) {
					// convert mission item to a simple waypoint
					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;

				} else {
					mavlink_log_critical(_navigator->get_mavlink_log_pub(), "MissionReverse: Got a non-position mission item, ignoring it");
				}

				break;
			}
		}

	} else { 	//position 정보가 없는 mission item 처리
		/* handle non-position mission items such as commands */
		switch (_mission_execution_mode) {
		case mission_result_s::MISSION_EXECUTION_MODE_NORMAL:
		case mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD: {
				position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
				// vtol 경우 skip
				/* turn towards next waypoint before MC to FW transition */
				if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && _navigator->get_vstatus()->is_rotary_wing
				    && !_navigator->get_land_detected()->landed
				    && has_next_position_item) {

					new_work_item_type = WORK_ITEM_TYPE_ALIGN;

					set_align_mission_item(&_mission_item, &mission_item_next_position);

					/* set position setpoint to target during the transition */
					mission_apply_limitation(_mission_item);
					mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->current);
				}
				// yaw가 맞추는 단계. vtol skip
				/* yaw is aligned now */
				if (_work_item_type == WORK_ITEM_TYPE_ALIGN &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

					// 현재 위치에서 진행 waypoint 방향으로의 yaw 구하기 
					/* set position setpoint to target during the transition */
					pos_sp_triplet->previous = pos_sp_triplet->current;
					generate_waypoint_from_heading(&pos_sp_triplet->current, pos_sp_triplet->current.yaw);
				}
				// vtol 모드 skip
				/* don't advance mission after FW to MC command */
				if (_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
				    && _work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && !_navigator->get_vstatus()->is_rotary_wing
				    && !_navigator->get_land_detected()->landed
				    && pos_sp_triplet->current.valid) {

					new_work_item_type = WORK_ITEM_TYPE_CMD_BEFORE_MOVE;
				}
				// vtol 모드 skip
				/* after FW to MC transition finish moving to the waypoint */
				if (_work_item_type == WORK_ITEM_TYPE_CMD_BEFORE_MOVE &&
				    new_work_item_type == WORK_ITEM_TYPE_DEFAULT
				    && pos_sp_triplet->current.valid) {

					new_work_item_type = WORK_ITEM_TYPE_DEFAULT;

					_mission_item.nav_cmd = NAV_CMD_WAYPOINT;
					copy_position_if_valid(&_mission_item, &pos_sp_triplet->current);
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}
				// fast_forward에서는 delay 시간을 0으로 설정해서 해당 명령 바로 넘어가게 하는 방식
				// ignore certain commands in mission fast forward
				if ((_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_FAST_FORWARD) &&
				    (_mission_item.nav_cmd == NAV_CMD_DELAY)) {
					_mission_item.autocontinue = true;
					_mission_item.time_inside = 0.0f;
				}

				break;
			}

		case mission_result_s::MISSION_EXECUTION_MODE_REVERSE: { //reverse 인 경우 무시
				// nothing to do, all commands are ignored
				break;
			}
		}
	}
	// mission item으로부터 sp 지정하기
	/*********************************** set setpoints and check next *********************************************/

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// precland가 아닌 경우에 적용. precland인 경우 거기서 처리
	/* set current position setpoint from mission item (is protected against non-position items) */
	if (new_work_item_type != WORK_ITEM_TYPE_PRECISION_LAND) {
		mission_apply_limitation(_mission_item);
		mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	}

	// 새로 계산한 sp가 변경이 발생한 경우에만 previsou에 저장
	/* only set the previous position item if the current one really changed */
	if ((_work_item_type != WORK_ITEM_TYPE_MOVE_TO_LAND) &&
	    !position_setpoint_equal(&pos_sp_triplet->current, &current_setpoint_copy)) {
		pos_sp_triplet->previous = current_setpoint_copy;
	}
	//position mission이 외의 명령을 처리. publish하면 해당 모듈이 처리할 수 있게 하는 역할.
	/* issue command if ready (will do nothing for position mission items) */
	issue_command(_mission_item);

	// 새로 시작할 type을 저장 
	/* set current work item type */
	_work_item_type = new_work_item_type;

	// 착륙 혹은 idle 상태가 되면 _need_takeoff를 true로 설정. takeoff 없이 waypoint misson 명령이 오는 경우 takeoff부터 동작하도록 돕는 flag.
	/* require takeoff after landing or idle */
	if (pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LAND
	    || pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {

		_need_takeoff = true;
	}

	_navigator->set_can_loiter_at_sp(false);
	reset_mission_item_reached();

	// offboard인 경우 현재 수행할 mission item을 설정
	if (_mission_type == MISSION_TYPE_OFFBOARD) {
		set_current_offboard_mission_item();
	}
	
	// autocontinue 설정되어 있고 delay가 0인 경우, 다음 mission이 있는 경우 sp->next에 pos정보 설정
	if (_mission_item.autocontinue && get_time_inside(_mission_item) < FLT_EPSILON) {
		/* try to process next mission item */
		if (has_next_position_item) {
			/* got next mission item, update setpoint triplet */
			mission_apply_limitation(mission_item_next_position);
			mission_item_to_position_setpoint(mission_item_next_position, &pos_sp_triplet->next);

		} else { // 다음 mission이 없는 경우 
			/* next mission item is not available */
			pos_sp_triplet->next.valid = false;
		}

	} else { // autocontinue가 설정안된 경우 next로 자동 넘어가지 않으므로 next.valid를 false로 설정
		/* vehicle will be paused on current waypoint, don't set next item */
		pos_sp_triplet->next.valid = false;
	}

	// 현재, 이전 sp가 모두 유효하다면 이들 두 지점사이의 거리 구하기 
	/* Save the distance between the current sp and the previous one */
	if (pos_sp_triplet->current.valid && pos_sp_triplet->previous.valid) {

		_distance_current_previous = get_distance_to_next_waypoint(
						     pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
						     pos_sp_triplet->previous.lat, pos_sp_triplet->previous.lon);
	}

	_navigator->set_position_setpoint_triplet_updated();
}

//수직 이륙이 필요한지 체크 
// 실제로는 _need_takeoff에 값 채우기
bool
Mission::do_need_vertical_takeoff()
{
	if (_navigator->get_vstatus()->is_rotary_wing) {

		float takeoff_alt = calculate_takeoff_altitude(&_mission_item);

		if (_navigator->get_land_detected()->landed) {
			//착륙된 상태이므로 takeoff 가능
			/* force takeoff if landed (additional protection) */
			_need_takeoff = true;

		} else if (_navigator->get_global_position()->alt > takeoff_alt - _navigator->get_altitude_acceptance_radius()) {
			// 이미 공중에 있는 상태고 takeoff 고도 이상에 위치하고 있으므로 takeoff 필요없음!
			/* if in-air and already above takeoff height, don't do takeoff */
			_need_takeoff = false;

		} else if (_navigator->get_global_position()->alt <= takeoff_alt - _navigator->get_altitude_acceptance_radius()
			   && (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
			       || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)) {
			// 이미 공중에 있으나 takeoff 고도 이하에 있으므로 takeoff 가능
			/* if in-air but below takeoff height and we have a takeoff item */
			_need_takeoff = true;
		}

		//현재 mission을 봤을 때 이미 이전에 takeoff가 수행되었을 것으로 보이므로 false로 설정
		/* check if current mission item is one that requires takeoff before */
		if (_need_takeoff && (
			    _mission_item.nav_cmd == NAV_CMD_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_WAYPOINT ||
			    _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED)) {

			_need_takeoff = false;
			return true;
		}
	}

	return false;
}

//착륙지로 이동 가능 여부 확인. 현재 mission의 명령이 LAND이고 착륙 지점 허용 반경을 벗어난 경우에 착륙지로 이동이 필요함. 
bool
Mission::do_need_move_to_land()
{
	if (_navigator->get_vstatus()->is_rotary_wing
	    && (_mission_item.nav_cmd == NAV_CMD_LAND || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND)) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

//VTOL 무시
bool
Mission::do_need_move_to_takeoff()
{
	if (_navigator->get_vstatus()->is_rotary_wing && _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF) {

		float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
				  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

		return d_current > _navigator->get_acceptance_radius();
	}

	return false;
}

// sp가 유효한 경웨 mission item으로 이 값을 복사.
void
Mission::copy_position_if_valid(struct mission_item_s *mission_item, struct position_setpoint_s *setpoint)
{
	if (setpoint->valid && setpoint->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
		mission_item->lat = setpoint->lat;
		mission_item->lon = setpoint->lon;
		mission_item->altitude = setpoint->alt;

	} else {
		mission_item->lat = _navigator->get_global_position()->lat;
		mission_item->lon = _navigator->get_global_position()->lon;
		mission_item->altitude = _navigator->get_global_position()->alt;
	}

	mission_item->altitude_is_relative = false;
}

//현재 위치 lat, lon과 다음 mission 위치를 이용해서 heading 방향을 설정.
void
Mission::set_align_mission_item(struct mission_item_s *mission_item, struct mission_item_s *mission_item_next)
{
	mission_item->nav_cmd = NAV_CMD_WAYPOINT;
	copy_position_if_valid(mission_item, &(_navigator->get_position_setpoint_triplet()->current));
	mission_item->altitude_is_relative = false;
	mission_item->autocontinue = true;
	mission_item->time_inside = 0.0f;
	mission_item->yaw = get_bearing_to_next_waypoint(
				    _navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
				    mission_item_next->lat, mission_item_next->lon);
	mission_item->force_heading = true;
}

//takeoff 고도 계산
float
Mission::calculate_takeoff_altitude(struct mission_item_s *mission_item)
{
	/* calculate takeoff altitude */
	float takeoff_alt = get_absolute_altitude_for_item(*mission_item);

	/* takeoff to at least MIS_TAKEOFF_ALT above home/ground, even if first waypoint is lower */
	// 착륙한 상태라면 (mission 고도)와 (현재 고도 + takeoff 최소고도) 중에 큰 값이 이륙 고도
	if (_navigator->get_land_detected()->landed) {
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_global_position()->alt + _navigator->get_takeoff_min_alt());

	} else {
		// (mission의 고도)과 (home 고도 +  takeoff 최소 고도) 중에 큰 값이 이륙 고도
		takeoff_alt = fmaxf(takeoff_alt, _navigator->get_home_position()->alt + _navigator->get_takeoff_min_alt());
	}

	return takeoff_alt;
}

// takeoff나 land하는 경우에는 heading 업데이트 하지 않음.
// 비행체가 향하는 방향을 결정하는 동작으로 결국 mission.yaw 값을 설정하는 것이 목적
void
Mission::heading_sp_update()
{
	//마지막 waypoing가 유효한 상태가 아니였다면 빠져나감
	/* we don't want to be yawing during takeoff, landing or aligning for a transition */
	if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF
	    || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF
	    || _mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION
	    || _mission_item.nav_cmd == NAV_CMD_LAND
	    || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND
	    || _work_item_type == WORK_ITEM_TYPE_ALIGN) {

		return;
	}

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	//마지막 waypoing가 유효한 상태가 아니였다면 빠져나감
	/* Don't change setpoint if last and current waypoint are not valid */
	if (!pos_sp_triplet->current.valid) {
		return;
	}

	// 비행체가 heading 방향 계산
	/* Calculate direction the vehicle should point to. */

	double point_from_latlon[2];
	double point_to_latlon[2];

	point_from_latlon[0] = _navigator->get_global_position()->lat;
	point_from_latlon[1] = _navigator->get_global_position()->lon;

	// ROI 지정된 경우 ROI 방향으로 heading이 가게 
	if (_navigator->get_vroi().mode == vehicle_roi_s::ROI_LOCATION && !_param_mnt_yaw_ctl.get()) {
		point_to_latlon[0] = _navigator->get_vroi().lat;
		point_to_latlon[1] = _navigator->get_vroi().lon;

		// 과도한 yaw를 막기 위해서 위치가 가까워지면 yaw 계산 안함. (가까운 경우 조금만 위치가 바뀌어도 yaw가 크게 발생하니까)
		/* stop if positions are close together to prevent excessive yawing */
		float d_current = get_distance_to_next_waypoint(
					  point_from_latlon[0], point_from_latlon[1],
					  point_to_latlon[0], point_to_latlon[1]);
		// 거리가 허용 반경 이상인 경우, roi으로의 yaw를 계산하여 적용
		if (d_current > _navigator->get_acceptance_radius()) {
			float yaw = get_bearing_to_next_waypoint(
					    point_from_latlon[0], point_from_latlon[1],
					    point_to_latlon[0], point_to_latlon[1]);

			_mission_item.yaw = yaw;
			pos_sp_triplet->current.yaw = _mission_item.yaw;
		}

	} else { // ROI 모드가 아닌 경우 
		// waypoint에 도착하여 loiter 하는 경우, 시간을 지정한 경우 현재 처리 없음.
		/* set yaw angle for the waypoint if a loiter time has been specified */
		if (_waypoint_position_reached && get_time_inside(_mission_item) > FLT_EPSILON) {
			// XXX: should actually be param4 from mission item
			// at the moment it will just keep the heading it has
			//_mission_item.yaw = _on_arrival_yaw;
			//pos_sp_triplet->current.yaw = _mission_item.yaw;

		} else { // 도착해서 loiter 하는 경우 이외. front, back 등과 같이 위치 지정에 따라 yaw 계산하기.
			//비행체 heading의 파라미터 설정에 따라서 타겟이 home이 경우 home의 lat, lon으로 설정
			/* target location is home */
			if ((_param_yawmode.get() == MISSION_YAWMODE_FRONT_TO_HOME
			     || _param_yawmode.get() == MISSION_YAWMODE_BACK_TO_HOME)
			    // need to be rotary wing for this but not in a transition
			    // in VTOL mode this will prevent updating yaw during FW flight
			    // (which would result in a wrong yaw setpoint spike during back transition)
			    && _navigator->get_vstatus()->is_rotary_wing
			    && !(_mission_item.nav_cmd == NAV_CMD_DO_VTOL_TRANSITION || _navigator->get_vstatus()->in_transition_mode)) {

				point_to_latlon[0] = _navigator->get_home_position()->lat;
				point_to_latlon[1] = _navigator->get_home_position()->lon;

			} else {
				// 타겟이 다음 waypoint인 경우 해당 위치를 lat, lon으로 설정
				/* target location is next (current) waypoint */
				point_to_latlon[0] = pos_sp_triplet->current.lat;
				point_to_latlon[1] = pos_sp_triplet->current.lon;
			}

			//현재 위치에서 향하는 지점까지의 거리 구하기
			float d_current = get_distance_to_next_waypoint(
						  point_from_latlon[0], point_from_latlon[1],
						  point_to_latlon[0], point_to_latlon[1]);

			// 타겟 지점으로 인정하는 거리보다 멀리 떨어져 있는 경우. yaw를 구한다. 
			/* stop if positions are close together to prevent excessive yawing */
			if (d_current > _navigator->get_acceptance_radius()) {
				float yaw = get_bearing_to_next_waypoint(
						    point_from_latlon[0],
						    point_from_latlon[1],
						    point_to_latlon[0],
						    point_to_latlon[1]);

				// 비행체의 뒤가 home을 향하는 경우 180도를 더한다. 
				/* always keep the back of the rotary wing pointing towards home */
				if (_param_yawmode.get() == MISSION_YAWMODE_BACK_TO_HOME) {
					_mission_item.yaw = wrap_pi(yaw + M_PI_F);
					pos_sp_triplet->current.yaw = _mission_item.yaw;

				} else if (_param_yawmode.get() == MISSION_YAWMODE_FRONT_TO_WAYPOINT
					   && _navigator->get_vroi().mode == vehicle_roi_s::ROI_WPNEXT && !_param_mnt_yaw_ctl.get()) {
					// 비행체 기준 roi가 설정된 경우 roi의 yaw offset만큼 yaw변경
					/* if yaw control for the mount is disabled and we have a valid ROI that points to the next
					 * waypoint, we add the gimbal's yaw offset to the vehicle's yaw */
					yaw += _navigator->get_vroi().yaw_offset;
					_mission_item.yaw = yaw;
					pos_sp_triplet->current.yaw = _mission_item.yaw;

				} else {
					//일반적인 경우로 heading이 yaw 방향과 일치하는 경우
					_mission_item.yaw = yaw;
					pos_sp_triplet->current.yaw = _mission_item.yaw;
				}
			}
		}
	}

	// we set yaw directly so we can run this in parallel to the FOH update
	_navigator->set_position_setpoint_triplet_updated();
}

//waypoint 이동시 linear 하게 상승/하강하면서 이동.
void
Mission::altitude_sp_foh_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	/* Don't change setpoint if last and current waypoint are not valid
	 * or if the previous altitude isn't from a position or loiter setpoint or
	 * if rotary wing since that is handled in the mc_pos_control
	 */


	if (!pos_sp_triplet->previous.valid || !pos_sp_triplet->current.valid || !PX4_ISFINITE(pos_sp_triplet->previous.alt)
	    || !(pos_sp_triplet->previous.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		 pos_sp_triplet->previous.type == position_setpoint_s::SETPOINT_TYPE_LOITER) ||
	    _navigator->get_vstatus()->is_rotary_wing) {

		return;
	}

	// waypoint 허용 범위에 들어온 경우 더 이상 계산할 필요 없음.
	/* Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one */
	if (_distance_current_previous - _navigator->get_acceptance_radius(_mission_item.acceptance_radius) < FLT_EPSILON) {
		return;
	}

	// 이륙, 착륙 동작이 아닌 경우 FOH 사용하지 않음. 지상과의 거리가 가까운 경우 문제 소지. 
	/* Don't do FOH for non-missions, landing and takeoff waypoints, the ground may be near
	 * and the FW controller has a custom landing logic
	 *
	 * NAV_CMD_LOITER_TO_ALT doesn't change altitude until reaching desired lat/lon
	 * */
	if (_mission_item.nav_cmd == NAV_CMD_LAND
	    || _mission_item.nav_cmd == NAV_CMD_VTOL_LAND
	    || _mission_item.nav_cmd == NAV_CMD_TAKEOFF
	    || _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT
	    || !_navigator->is_planned_mission()) {

		return;
	}

	// 현재 위치와 waypoint 사이 수평 거리 계산
	/* Calculate distance to current waypoint */
	float d_current = get_distance_to_next_waypoint(_mission_item.lat, _mission_item.lon,
			  _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	// 최대한 작은 거리를 선택. waypoint까지의 거리와 sp변경 되는 동안 이동한 거리 사이 사이에 작은 값을 선택 
	/* Save distance to waypoint if it is the smallest ever achieved, however make sure that
	 * _min_current_sp_distance_xy is never larger than the distance between the current and the previous wp */
	_min_current_sp_distance_xy = math::min(math::min(d_current, _min_current_sp_distance_xy),
						_distance_current_previous);

	// 수용 반경이 계산한 최소 거리 크면 waypoint 고도를 그대로 설정. (고도를 linear하게 올리는 경우 waypoint 범위가 더 넓으므로 그 안에서 linear하게 올려도 alt가 범위를 벗어나지는 않으니까)
	/* if the minimal distance is smaller then the acceptance radius, we should be at waypoint alt
	 * navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude */
	if (_min_current_sp_distance_xy < _navigator->get_acceptance_radius(_mission_item.acceptance_radius)) {
		pos_sp_triplet->current.alt = get_absolute_altitude_for_item(_mission_item);

	} else {  // linear하게 올라가다가 waypoint 안에서 괜히 고도 허용 범위를 벗어날 수 있으니까. 적정 고도를 다시 계산
		/* update the altitude sp of the 'current' item in the sp triplet, but do not update the altitude sp
		 * of the mission item as it is used to check if the mission item is reached
		 * The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
		 * radius around the current waypoint
		 **/
		// 현재 설정하는 waypoint 근처 허용 반경에서 current alt가 도달하도록. 
		// delta_alt(sp 업데이트 마다의 고도 변화), grad(수평 거리당 고도(delta_alt / (최대 허용 반경 - 이동할 거리))),
		// a(이전 sp 고도 - ) 
		// sp 고도 = 수평 거리당 고도 * 이동할 수평 거리 + 
		float delta_alt = (get_absolute_altitude_for_item(_mission_item) - pos_sp_triplet->previous.alt);
		float grad = -delta_alt / (_distance_current_previous - _navigator->get_acceptance_radius(
						   _mission_item.acceptance_radius));
		float a = pos_sp_triplet->previous.alt - grad * _distance_current_previous;
		pos_sp_triplet->current.alt = a + grad * _min_current_sp_distance_xy;
	}

	// we set altitude directly so we can run this in parallel to the heading update
	_navigator->set_position_setpoint_triplet_updated();
}

//mission 비행 속도 변경. sp의 curising speed를 변경. 
void
Mission::cruising_speed_sp_update()
{
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	const float cruising_speed = _navigator->get_cruising_speed();

	// 현재 sp가 유효하지 않으면 설정하지 않음.
	/* Don't change setpoint if the current waypoint is not valid */
	if (!pos_sp_triplet->current.valid ||
	    fabsf(pos_sp_triplet->current.cruising_speed - cruising_speed) < FLT_EPSILON) {
		return;
	}

	pos_sp_triplet->current.cruising_speed = cruising_speed;

	_navigator->set_position_setpoint_triplet_updated();
}

// fw 모드 skip. 착륙 취소.
// 핵심은 착륙 중에 취소하면 loiter 명령으로 전환된다는 점. 
void
Mission::do_abort_landing()
{
	// Abort FW landing
	//  first climb out then loiter over intended landing location

	if (_mission_item.nav_cmd != NAV_CMD_LAND) {
		return;
	}

	// loiter at the larger of MIS_LTRMIN_ALT above the landing point
	//  or 2 * FW_CLMBOUT_DIFF above the current altitude
	const float alt_landing = get_absolute_altitude_for_item(_mission_item);
	const float alt_sp = math::max(alt_landing + _navigator->get_loiter_min_alt(),
				       _navigator->get_global_position()->alt + 20.0f);

	// turn current landing waypoint into an indefinite loiter
	_mission_item.nav_cmd = NAV_CMD_LOITER_UNLIMITED;
	_mission_item.altitude_is_relative = false;
	_mission_item.altitude = alt_sp;
	_mission_item.loiter_radius = _navigator->get_loiter_radius();
	_mission_item.acceptance_radius = _navigator->get_acceptance_radius();
	_mission_item.autocontinue = false;
	_mission_item.origin = ORIGIN_ONBOARD;

	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &_navigator->get_position_setpoint_triplet()->current);
	_navigator->set_position_setpoint_triplet_updated();

	mavlink_and_console_log_info(_navigator->get_mavlink_log_pub(), "Holding at %d m above landing.",
				     (int)(alt_sp - alt_landing));

	// reset mission index to start of landing
	if (_land_start_available) {
		_current_offboard_mission_index = get_land_start_index();

	} else {
		// move mission index back (landing approach point)
		_current_offboard_mission_index -= 1;
	}

	// send reposition cmd to get out of mission
	vehicle_command_s vcmd = {};

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_REPOSITION;
	vcmd.param1 = -1;
	vcmd.param2 = 1;
	vcmd.param5 = _mission_item.lat;
	vcmd.param6 = _mission_item.lon;
	vcmd.param7 = alt_sp;

	_navigator->publish_vehicle_cmd(&vcmd);
}

// 현재 mission iterm, 다음 mission item중에 position 정보가 있는 mission item, position item 유무를 나타내는 변수
bool
Mission::prepare_mission_items(mission_item_s *mission_item,
			       mission_item_s *next_position_mission_item, bool *has_next_position_item)
{
	*has_next_position_item = false;
	bool first_res = false;
	int offset = 1;

	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		offset = -1;
	}

	// 현재 mission item 읽어와서 
	if (read_mission_item(0, mission_item)) {

		first_res = true;

		// 다음 mission item 읽어서 position 정보가 있는 mission item이면 완료.
		/* trying to find next position mission item */
		while (read_mission_item(offset, next_position_mission_item)) {

			if (item_contains_position(*next_position_mission_item)) {
				*has_next_position_item = true;
				break;
			}

			if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
				offset--;

			} else {
				offset++;

			}
		}
	}

	return first_res;
}

// 현재 index 기준 offset을 적용한 mission item 가져오기
// mission item 가져올 때 DO_JUMP로 무한루프 도는 경우를 미리 검사하도록 구현
bool
Mission::read_mission_item(int offset, struct mission_item_s *mission_item)
{
	// 현재 mission index 기준으로 offset 뒤에 있는 mission item 읽기
	/* select offboard mission */
	const int current_index = _current_offboard_mission_index;
	int index_to_read = current_index + offset;

	// offset이 0이면 현재 index 그대로 사용 아니면 offset 더한 index 사용
	int *mission_index_ptr = (offset == 0) ? &_current_offboard_mission_index : &index_to_read;
	const dm_item_t dm_item = (dm_item_t)_offboard_mission.dataman_id;

	// mission이 없는 경우 그냥 false 반환
	/* do not work on empty missions */
	if (_offboard_mission.count == 0) {
		return false;
	}

	// DO JUMPS가 여러 개 있는 경우 여러 번 반복해봐야 함. 10번 iteration을 넘어가면 DO JUMPS 무한 루프가 있다고 보고 판단.  
	// mission_index_ptr 으로 검사 대상 mission item을 찾기.
	// 즉 10번 동안 DO_JUMP -> DO_JUMP .... -> DO_JUMP 반복되는 상황 검사
	/* Repeat this several times in case there are several DO JUMPS that we need to follow along, however, after
	 * 10 iterations we have to assume that the DO JUMPS are probably cycling and give up. */
	for (int i = 0; i < 10; i++) {
		// 찾기를 원하는 index가 실제 저장된 index 범위를 벗어나는 경우 false
		if (*mission_index_ptr < 0 || *mission_index_ptr >= (int)_offboard_mission.count) {
			/* mission item index out of bounds - if they are equal, we just reached the end */
			if ((*mission_index_ptr != (int)_offboard_mission.count) && (*mission_index_ptr != -1)) {
				mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Mission item index out of bound, index: %d, max: %d.",
						     *mission_index_ptr, _offboard_mission.count);
			}

			return false;
		}

		const ssize_t len = sizeof(struct mission_item_s);

		// 혹시 dm 데이터에 문제가 있을 수 있으므로 mission_item_tmp 임시 변수 사용해서 체크. 최종 정상이라고 판단되면 memcpy로 복사
		/* read mission item to temp storage first to not overwrite current mission item if data damaged */
		struct mission_item_s mission_item_tmp;

		// dm에서 해당 index의 mission item 읽기
		/* read mission item from datamanager */
		if (dm_read(dm_item, *mission_index_ptr, &mission_item_tmp, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Waypoint could not be read.");
			return false;
		}

		// 읽어온 mission item이 DO_JUMP item인 경우 검사. 10번 반복 횟수만큼 돌려보기.
		/* check for DO_JUMP item, and whether it hasn't not already been repeated enough times */
		if (mission_item_tmp.nav_cmd == NAV_CMD_DO_JUMP) {
			const bool execute_jumps = _mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_NORMAL;

			// 일반 모드(reverse가 아닌)에서 돌리는 경우
			/* do DO_JUMP as many times as requested if not in reverse mode */
			if ((mission_item_tmp.do_jump_current_count < mission_item_tmp.do_jump_repeat_count) && execute_jumps) {

				/* only raise the repeat count if this is for the current mission item
				 * but not for the read ahead mission item */
				if (offset == 0) { // 현재 실행할 mission item인 경우 do_jump count 증가(정상인 경우 mission item에 복사되므로)
					(mission_item_tmp.do_jump_current_count)++;

					/* save repeat count */
					if (dm_write(dm_item, *mission_index_ptr, DM_PERSIST_POWER_ON_RESET, &mission_item_tmp, len) != len) {
						/* not supposed to happen unless the datamanager can't access the dataman */
						mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP waypoint could not be written.");
						return false;
					}

					report_do_jump_mission_changed(*mission_index_ptr, mission_item_tmp.do_jump_repeat_count);
				}

				// do_jump할 다음 mission item을 가져오기 위해서 다음 mission item index는 do_jump_mission_index가 됨.
				/* set new mission item index and repeat
				 * we don't have to validate here, if it's invalid, we should realize this later .*/
				*mission_index_ptr = mission_item_tmp.do_jump_mission_index;

			} else { // repeat이 완료된 경우 or reverse 모드인 경우
				if (offset == 0 && execute_jumps) { // normal 모드에서 repeat이 완료했다고 메시지 날려주고
					mavlink_log_info(_navigator->get_mavlink_log_pub(), "DO JUMP repetitions completed.");
				}
				// reverse인 경우 이전 index mission item 가지고 옴.
				/* no more DO_JUMPS, therefore just try to continue with next mission item */
				if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
					(*mission_index_ptr)--;

				} else { //normal 모드면 다음 mission item 읽어오기
					(*mission_index_ptr)++;
				}
			}

		} else { //DO_JUMP 아니므로 성공. mission item에 복사
			/* if it's not a DO_JUMP, then we were successful */
			memcpy(mission_item, &mission_item_tmp, sizeof(struct mission_item_s));
			return true;
		}
	}

	// 10번 반복했는데 반복되 었으므로 cycle이 있다고 판단!
	/* we have given up, we don't want to cycle forever */
	mavlink_log_critical(_navigator->get_mavlink_log_pub(), "DO JUMP is cycling, giving up.");
	return false;
}

//mission state를 dm의 DM_KEY_MISSION_STATE에 저장. 
void
Mission::save_offboard_mission_state()
{
	mission_s mission_state = {};

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	// 현재 mission state 읽어와서 
	/* read current state */
	int read_res = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

	if (read_res == sizeof(mission_s)) {
		// 정상적으로 읽어 왔는지 검사 id, items count 검사. 
		/* data read successfully, check dataman ID and items count */
		if (mission_state.dataman_id == _offboard_mission.dataman_id && mission_state.count == _offboard_mission.count) {
			// navigator가 seq만 변경할 수도 있
			/* navigator may modify only sequence, write modified state only if it changed */
			if (mission_state.current_seq != _current_offboard_mission_index) {
				mission_state.timestamp = hrt_absolute_time();

				if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
					     sizeof(mission_s)) != sizeof(mission_s)) {

					PX4_ERR("Can't save mission state");
				}
			}
		}

	} else {
		// 여기 상태로 들어오면 안됨. 만약 이런 경우라면 error 상태를 publish해야함.
		/* invalid data, this must not happen and indicates error in offboard_mission publisher */
		mission_state.timestamp = hrt_absolute_time();
		mission_state.dataman_id = _offboard_mission.dataman_id;
		mission_state.count = _offboard_mission.count;
		mission_state.current_seq = _current_offboard_mission_index;

		mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Invalid mission state.");

		/* write modified state only if changed */
		if (dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission_state,
			     sizeof(mission_s)) != sizeof(mission_s)) {

			PX4_ERR("Can't save mission state");
		}
	}

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}
}

//do_jump 명령이 update되었다고 설정. 상태 변경하고 flag로 설정.
void
Mission::report_do_jump_mission_changed(int index, int do_jumps_remaining)
{
	/* inform about the change */
	_navigator->get_mission_result()->item_do_jump_changed = true;
	_navigator->get_mission_result()->item_changed_index = index;
	_navigator->get_mission_result()->item_do_jump_remaining = do_jumps_remaining;

	_navigator->set_mission_result_updated();
}

// 현재 mission을 완료했다고 설정. seq_reached에 현재 index를 대입
void
Mission::set_mission_item_reached()
{
	_navigator->get_mission_result()->seq_reached = _current_offboard_mission_index;
	_navigator->set_mission_result_updated();

	reset_mission_item_reached();
}

// _current_offboard_mission_index를 현재 수행할 mission으로 설정
void
Mission::set_current_offboard_mission_item()
{
	_navigator->get_mission_result()->finished = false;
	_navigator->get_mission_result()->seq_current = _current_offboard_mission_index;

	_navigator->set_mission_result_updated();

	save_offboard_mission_state();
}

//mission이 유효한지 검사
//missionFeasibilityChecker.checkMissionFeasible() 체크가 핵심
void
Mission::check_mission_valid(bool force)
{
	//home_inited가 안된 경우 동작하는데 force가 true인 경우 home_inited가 된 경우에도 다시 검사하도록 함.
	if ((!_home_inited && _navigator->home_position_valid()) || force) {

		MissionFeasibilityChecker _missionFeasibilityChecker(_navigator);

		_navigator->get_mission_result()->valid =
			_missionFeasibilityChecker.checkMissionFeasible(_offboard_mission,
					_param_dist_1wp.get(),
					_param_dist_between_wps.get(),
					_navigator->mission_landing_required());

		_navigator->get_mission_result()->seq_total = _offboard_mission.count;
		_navigator->increment_mission_instance_count();
		_navigator->set_mission_result_updated();
		_home_inited = _navigator->home_position_valid();

		// find and store landing start marker (if available)
		find_offboard_land_start();
	}
}

// mission 상태를 기록하는 정보를 리셋. 사실 간단히 하는 일은 mission.current_seq = 0 후에 다시 dm에 저장하는 것인데
// do_jump의 경우 각 mission에 jump count 정보를 저장하고 있어서 이를 reset 해주기 위해서 코드가 길다. 
void
Mission::reset_offboard_mission(struct mission_s &mission)
{
	dm_lock(DM_KEY_MISSION_STATE);

	if (dm_read(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s)) == sizeof(mission_s)) {
		if (mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 || mission.dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_1) {
			/* set current item to 0 */
			mission.current_seq = 0;

			//JUMP의 경우 해당 mission에 do_jump_current_count 정보를 가지고 있어서 이 부분까지 초기화 해야한다.
			/* reset jump counters */
			if (mission.count > 0) {
				const dm_item_t dm_current = (dm_item_t)mission.dataman_id;

				for (unsigned index = 0; index < mission.count; index++) {
					struct mission_item_s item;
					const ssize_t len = sizeof(struct mission_item_s);

					if (dm_read(dm_current, index, &item, len) != len) {
						PX4_WARN("could not read mission item during reset");
						break;
					}

					if (item.nav_cmd == NAV_CMD_DO_JUMP) {
						item.do_jump_current_count = 0;

						if (dm_write(dm_current, index, DM_PERSIST_POWER_ON_RESET, &item, len) != len) {
							PX4_WARN("could not save mission item during reset");
							break;
						}
					}
				}
			}

		} else {
			mavlink_log_critical(_navigator->get_mavlink_log_pub(), "Could not read mission.");

			/* initialize mission state in dataman */
			mission.timestamp = hrt_absolute_time();
			mission.dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
			mission.count = 0;
			mission.current_seq = 0;
		}

		dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));
	}

	dm_unlock(DM_KEY_MISSION_STATE);
}

// mission reset 필요 여부를 나타내는 _need_mission_reset 변수 설정. active 인자는 현재 mission이 실행중인 여부를 나타냄. 
// diarm 상태인 경우는 무조건 _need_mission_reset이 false로 설정. 
bool
Mission::need_to_reset_mission(bool active)
{
	// disarm인 경우 mission reset이 필요한 상태로 설정이 되어 있을때만 true 반환. 다시 mission reset할 필요없으므로 false 설정.
	/* reset mission state when disarmed */
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED && _need_mission_reset) {
		_need_mission_reset = false;
		return true;

	} else if (_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED && active) {
		// disarm 되고 난 후에 reset 필요
		/* mission is running, need reset after disarm */
		_need_mission_reset = true;
	}

	return false;
}

//yaw값인 heading으로부터 sp 구하기 
void
Mission::generate_waypoint_from_heading(struct position_setpoint_s *setpoint, float yaw)
{
	//현재 위치에서 인자 yaw 방향으로 1000000m 떨어진 lat, lon 지정.
	waypoint_from_heading_and_distance(
		_navigator->get_global_position()->lat, _navigator->get_global_position()->lon,
		yaw, 1000000.0f,
		&(setpoint->lat), &(setpoint->lon));
	setpoint->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	setpoint->yaw = yaw;
}

// mission들 중에서 position 정보를 가지고 있는 것 중에서 현재위치에서 가장 가까이 있는 mission의 index를 반환.
// 이 위치부터 mission을 시작하도록 하기 위한 목적
int32_t
Mission::index_closest_mission_item() const
{
	int32_t min_dist_index(0);
	float min_dist(FLT_MAX), dist_xy(FLT_MAX), dist_z(FLT_MAX);

	dm_item_t dm_current = (dm_item_t)(_offboard_mission.dataman_id);

	for (size_t i = 0; i < _offboard_mission.count; i++) {
		struct mission_item_s missionitem = {};
		const ssize_t len = sizeof(missionitem);

		if (dm_read(dm_current, i, &missionitem, len) != len) {
			/* not supposed to happen unless the datamanager can't access the SD card, etc. */
			PX4_ERR("dataman read failure");
			break;
		}
		//position 정보가 있는 mission인 경우만 처리
		if (item_contains_position(missionitem)) {
			// do not consider land waypoints for a fw
			if (!((missionitem.nav_cmd == NAV_CMD_LAND) &&
			      (!_navigator->get_vstatus()->is_rotary_wing) &&
			      (!_navigator->get_vstatus()->is_vtol))) {
				float dist = get_distance_to_point_global_wgs84(missionitem.lat, missionitem.lon,
						get_absolute_altitude_for_item(missionitem),
						_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_navigator->get_global_position()->alt,
						&dist_xy, &dist_z);

				if (dist < min_dist) {
					min_dist = dist;
					min_dist_index = i;
				}
			}
		}
	}

	// reverse 모드인 경우에는 현재 위치기준 home과 위에서 구한 최소거리 mission과의 거리 중에서 가까운 것을 구한다. 
	// for mission reverse also consider the home position
	if (_mission_execution_mode == mission_result_s::MISSION_EXECUTION_MODE_REVERSE) {
		float dist = get_distance_to_point_global_wgs84(
				     _navigator->get_home_position()->lat,
				     _navigator->get_home_position()->lon,
				     _navigator->get_home_position()->alt,
				     _navigator->get_global_position()->lat,
				     _navigator->get_global_position()->lon,
				     _navigator->get_global_position()->alt,
				     &dist_xy, &dist_z);

		if (dist < min_dist) {
			min_dist = dist;
			min_dist_index = -1;
		}
	}

	return min_dist_index;
}

// p1, p2가 동일한 setpoint를 가리키는지 여부 확인
bool Mission::position_setpoint_equal(const position_setpoint_s *p1, const position_setpoint_s *p2) const
{
	return ((p1->valid == p2->valid) &&
		(p1->type == p2->type) &&
		(fabsf(p1->x - p2->x) < FLT_EPSILON) &&
		(fabsf(p1->y - p2->y) < FLT_EPSILON) &&
		(fabsf(p1->z - p2->z) < FLT_EPSILON) &&
		(p1->position_valid == p2->position_valid) &&
		(fabsf(p1->vx - p2->vx) < FLT_EPSILON) &&
		(fabsf(p1->vy - p2->vy) < FLT_EPSILON) &&
		(fabsf(p1->vz - p2->vz) < FLT_EPSILON) &&
		(p1->velocity_valid == p2->velocity_valid) &&
		(p1->velocity_frame == p2->velocity_frame) &&
		(p1->alt_valid == p2->alt_valid) &&
		(fabs(p1->lat - p2->lat) < DBL_EPSILON) &&
		(fabs(p1->lon - p2->lon) < DBL_EPSILON) &&
		(fabsf(p1->alt - p2->alt) < FLT_EPSILON) &&
		((fabsf(p1->yaw - p2->yaw) < FLT_EPSILON) || (!PX4_ISFINITE(p1->yaw) && !PX4_ISFINITE(p2->yaw))) &&
		(p1->yaw_valid == p2->yaw_valid) &&
		(fabsf(p1->yawspeed - p2->yawspeed) < FLT_EPSILON) &&
		(p1->yawspeed_valid == p2->yawspeed_valid) &&
		(fabsf(p1->loiter_radius - p2->loiter_radius) < FLT_EPSILON) &&
		(p1->loiter_direction == p2->loiter_direction) &&
		(fabsf(p1->pitch_min - p2->pitch_min) < FLT_EPSILON) &&
		(fabsf(p1->a_x - p2->a_x) < FLT_EPSILON) &&
		(fabsf(p1->a_y - p2->a_y) < FLT_EPSILON) &&
		(fabsf(p1->a_z - p2->a_z) < FLT_EPSILON) &&
		(p1->acceleration_valid == p2->acceleration_valid) &&
		(p1->acceleration_is_force == p2->acceleration_is_force) &&
		(fabsf(p1->acceptance_radius - p2->acceptance_radius) < FLT_EPSILON) &&
		(fabsf(p1->cruising_speed - p2->cruising_speed) < FLT_EPSILON) &&
		(fabsf(p1->cruising_throttle - p2->cruising_throttle) < FLT_EPSILON));

}
