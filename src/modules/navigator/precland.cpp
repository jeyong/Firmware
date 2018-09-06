/****************************************************************************
 *
 *   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
 * @file precland.cpp
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#include "precland.h"
#include "navigator.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_command.h>

#define SEC2USEC 1000000.0f

#define STATE_TIMEOUT 10000000 // [us] Maximum time to spend in any state  10초

PrecLand::PrecLand(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
}

void
PrecLand::on_activation()
{
	//landing_target_pose를 수신
	// We need to subscribe here and not in the constructor because constructor is called before the navigator task is spawned
	if (_target_pose_sub < 0) {
		_target_pose_sub = orb_subscribe(ORB_ID(landing_target_pose));
	}

	_state = PrecLandState::Start;
	_search_cnt = 0;
	_last_slewrate_time = 0;

	// local pos를 projection의 기준으로 설정
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	if (!map_projection_initialized(&_map_ref)) {
		map_projection_init(&_map_ref, vehicle_local_position->ref_lat, vehicle_local_position->ref_lon);
	}

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	pos_sp_triplet->next.valid = false;

	// 현재 위치가 유효하지 않은 경우, prcland하지 말고 그냥 현재 위치에 착륙하도록
	// Check that the current position setpoint is valid, otherwise land at current position
	if (!pos_sp_triplet->current.valid) {
		PX4_WARN("Resetting landing position to current position");
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
		pos_sp_triplet->current.valid = true;
	}

	_sp_pev = matrix::Vector2f(0, 0); // x,y 이전 떨어진 거리 
	_sp_pev_prev = matrix::Vector2f(0, 0); // x, y 이전전 떨어진 거리
	_last_slewrate_time = 0;

	switch_to_state_start();

}

void
PrecLand::on_active()
{
	// target pose를 subscribe 하기
	// get new target measurement
	orb_check(_target_pose_sub, &_target_pose_updated);

	if (_target_pose_updated) {
		orb_copy(ORB_ID(landing_target_pose), _target_pose_sub, &_target_pose);
		_target_pose_valid = true;
	}

	// timeout 시간이 지나면 target_pos_valid를 false로 설정
	if ((hrt_elapsed_time(&_target_pose.timestamp) / 1e6f) > _param_timeout.get()) {
		_target_pose_valid = false;
	}

	// 이미 착륙한 경우 done 상태로 설정. stop if we are landed
	if (_navigator->get_land_detected()->landed) {
		switch_to_state_done();
	}

	// 각 state에 따라 해당 state에서 수행해야하는 method를 호출
	switch (_state) {
	case PrecLandState::Start:
		run_state_start();
		break;

	case PrecLandState::HorizontalApproach:
		run_state_horizontal_approach();
		break;

	case PrecLandState::DescendAboveTarget:
		run_state_descend_above_target();
		break;

	case PrecLandState::FinalApproach:
		run_state_final_approach();
		break;

	case PrecLandState::Search:
		run_state_search();
		break;

	case PrecLandState::Fallback:
		run_state_fallback();
		break;

	case PrecLandState::Done:
		// nothing to do
		break;

	default:
		// unknown state
		break;
	}

}

//precland 시작 동작
void
PrecLand::run_state_start()
{
	// 수평 접근 상태로 전환 가능하면 전환하고 빠져나감.
	// check if target visible and go to horizontal approach
	if (switch_to_state_horizontal_approach()) {
		return;
	}

	// 처음에 타겟을 못찾으면 일반 착륙모드로 동작
	if (_mode == PrecLandMode::Opportunistic) {
		// could not see the target immediately, so just fall back to normal landing
		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to search or fallback landing");
		}
	}

	// pos sp와 현재 위치 사이의 거리 구하기
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	float dist = get_distance_to_next_waypoint(pos_sp_triplet->current.lat, pos_sp_triplet->current.lon,
			_navigator->get_global_position()->lat, _navigator->get_global_position()->lon);

	// 허용 범위내로 들어가면 타겟을 탐색. 
	// check if we've reached the start point
	if (dist < _navigator->get_acceptance_radius()) {
		if (!_point_reached_time) { // 허용 범위내에 처음 들어간 시간 저장
			_point_reached_time = hrt_absolute_time();
		}
		// 1초 후에 타겟을 탐색하지 못했다면 다시 탐색.
		// if we don't see the target after 1 second, search for it
		if (_param_search_timeout.get() > 0) { // timeout 시간이 지정되어 있는 경우 

			if (hrt_absolute_time() - _point_reached_time > 2000000) { //허용 범위에 들어가서 2초 지나면 타겟 찾기하고 못 찾으면 일반 착륙모드로
				if (!switch_to_state_search()) {  // 다시 찾는 시도 하고 
					if (!switch_to_state_fallback()) {  // 못찾았으면 그냥 일반 착륙으로 동작!
						PX4_ERR("Can't switch to search or fallback landing");
					}
				}
			}

		} else { // timeout이 시간이 0이면 바로 일반 착륙으로 전환
			if (!switch_to_state_fallback()) {
				PX4_ERR("Can't switch to search or fallback landing");
			}
		}
	}
}

// 수평 접근 모드 수행
void
PrecLand::run_state_horizontal_approach()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// 수평접근 가능한 상태가 아니라면(타겟을 놓친 경우, 허용 범위 벗어난 경우), 현재 위치에서 타겟을 찾기 위해서 sp를 현재 위치로 설정
	// check if target visible, if not go to start
	if (!check_state_conditions(PrecLandState::HorizontalApproach)) {
		PX4_WARN("Lost landing target while landing (horizontal approach).");

		// 타겟을 찾기 위해서 현재 위치에서 대기하도록
		// Stay at current position for searching for the landing target
		pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;

		// START 상태로 전환하거나 일반 착륙 모드로 전환
		if (!switch_to_state_start()) {
			if (!switch_to_state_fallback()) { // start모드도 실패한 경우면 그냥 일반 착륙 모드로
				PX4_ERR("Can't switch to fallback landing");
			}
		}

		return;
	}

	// 타겟 위에서 하강 가능한 상태라면, 하강 상태로 전환
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!_point_reached_time) { 
			_point_reached_time = hrt_absolute_time();
		}

		if (hrt_absolute_time() - _point_reached_time > 2000000) { //DescendAboveTarget 전환 가능한 상태로 2초 이상 지났으면 DescendAboveTarget 상태로 전환 
			// if close enough for descent above target go to descend above target
			if (switch_to_state_descend_above_target()) {
				return;
			}
		}

	}

	// 수평이동 상태로 너무 오래 남아 있는 경우(10초 이상), 그냥 일반 착륙 모드로 전환
	if (hrt_absolute_time() - _state_start_time > STATE_TIMEOUT) {
		PX4_ERR("Precision landing took too long during horizontal approach phase.");

		if (switch_to_state_fallback()) {
			return;
		}

		PX4_ERR("Can't switch to fallback landing");
	}

	// 수평 이동을 위한 lat, lon 구하기. 정밀 착륙의 경우 타겟을 벗어나지 않게 천천히 이동해야 하므로 최대 이동 속도나 가속도 제한을 가지고 현재 위치구하고 pos sp의 lat, lon을 구한다. 
	float x = _target_pose.x_abs;  //비행체와 타겟의 NED의 X/north 으로 떨어진 거리 (m단위)
	float y = _target_pose.y_abs;  //비행체와 타겟의 NED의 Y/east 으로 떨어진 거리 (m단위)

	// 최대한 부드럽게 이용할 수 있는 거리 x, y를 계산. 센서에서는 x, y만큼 떨어져 있다고 하지만 slew rate(입력이 갑작스럽게 변화하는 경우 최대로 낼수 있는 값의 범위를 제한)을 고려하여 x, y 값을 허용 범위 값으로 재조정.
	slewrate(x, y);

	// x,y를 떨어진 거리를 통해 타겟의 lat, lon 구하기. 구한 lat, lon을 sp의 lat, lon에 사용.
	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	double lat, lon;
	map_projection_reproject(&_map_ref, x, y, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;
	pos_sp_triplet->current.alt = _approach_alt; // 접근 고도
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;

	_navigator->set_position_setpoint_triplet_updated();
}

// 타겟 위에서 하강하는 state
void
PrecLand::run_state_descend_above_target()
{
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// 타겟 하강 상태로 전환 가능한지 체크해서 불가능한 경우, 타겟을 현재 위치에서 찾기 위해서 sp를 현재 위치로 설정
	// check if target visible
	if (!check_state_conditions(PrecLandState::DescendAboveTarget)) {
		if (!switch_to_state_final_approach()) {
			PX4_WARN("Lost landing target while landing (descending).");

			// Stay at current position for searching for the target
			pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
			pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
			pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;

			// start 상태로 시도해서 안되면 일반 착륙 모드로 전환
			if (!switch_to_state_start()) {
				if (!switch_to_state_fallback()) {
					PX4_ERR("Can't switch to fallback landing");
				}
			}
		}

		return;
	}

	//x, y로 lat, lon을 구해서 sp의 lat, lon 설정
	// XXX need to transform to GPS coords because mc_pos_control only looks at that
	double lat, lon;
	map_projection_reproject(&_map_ref, _target_pose.x_abs, _target_pose.y_abs, &lat, &lon);

	pos_sp_triplet->current.lat = lat;
	pos_sp_triplet->current.lon = lon;

	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;

	_navigator->set_position_setpoint_triplet_updated();
}

void
PrecLand::run_state_final_approach()
{
	// nothing to do, will land
}

// search 상태 실행하기
void
PrecLand::run_state_search()
{
	// 수평 접근 상태가 가능하다면 타겟을 찾았다는 뜻이다. 따라서 타겟을 찾은 시간, sp 찾지 못했다면 
	// check if we can see the target
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		if (!_target_acquired_time) {
			// 너무 갑자기 정지하게 하지 않기 위해서 마진을 줬음. 1m. 탐색 모드로 되면 탐색 고도로 올라가기 때문에 이를 부드럽게 멈추게 해야함. 
			// target just became visible. Stop climbing, but give it some margin so we don't stop too apruptly
			_target_acquired_time = hrt_absolute_time();
			position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
			float new_alt = _navigator->get_global_position()->alt + 1.0f; // 고도 값이 너무 타이트하면 갑자기 멈추니까. 대략 1m 마진을 줬음.
			//고도는 작은 값을 선택함. 착륙모드라 내려가야하니까 괜히 상승하는 것은 바람직하지 않음.
			pos_sp_triplet->current.alt = new_alt < pos_sp_triplet->current.alt ? new_alt : pos_sp_triplet->current.alt;
			_navigator->set_position_setpoint_triplet_updated();
		}

	}

	// 해당 위치에서 안정을 찾도록 1초 동안 해당 높이에서 머무르고 나서 수평 이동 모드로 전환되도록 함.
	// stay at that height for a second to allow the vehicle to settle
	if (_target_acquired_time && (hrt_absolute_time() - _target_acquired_time) > 1000000) {
		// try to switch to horizontal approach
		if (switch_to_state_horizontal_approach()) {
			return;
		}
	}

	// 검색 타임아웃이 경우 일반 착륙으로 전환
	// check if search timed out and go to fallback
	if (hrt_absolute_time() - _state_start_time > _param_search_timeout.get()*SEC2USEC) {
		PX4_WARN("Search timed out");

		if (!switch_to_state_fallback()) {
			PX4_ERR("Can't switch to fallback landing");
		}
	}
}

void
PrecLand::run_state_fallback()
{
	// nothing to do, will land
}

//start 상태로 변환
bool
PrecLand::switch_to_state_start()
{
	// Start 상태인지 체크. 맞으면 sp의 타입을 SETPOINT_TYPE_POSITION로 설정
	if (check_state_conditions(PrecLandState::Start)) {
		position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		_navigator->set_position_setpoint_triplet_updated();
		_search_cnt++;

		_point_reached_time = 0;

		_state = PrecLandState::Start;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

// 수평 접근 상태로 전환이 가능한 상태인 수평접근 상태로 설정하고 현재 고도를 approach_alt로 설정
bool
PrecLand::switch_to_state_horizontal_approach()
{
	if (check_state_conditions(PrecLandState::HorizontalApproach)) {
		_approach_alt = _navigator->get_global_position()->alt;

		_point_reached_time = 0;

		_state = PrecLandState::HorizontalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

//타겟 위에서 하강이 가능한 상태로 전환이 가능한 상태인 경우 상태를 DescendAboveTarget로 설정
bool
PrecLand::switch_to_state_descend_above_target()
{
	if (check_state_conditions(PrecLandState::DescendAboveTarget)) {
		_state = PrecLandState::DescendAboveTarget;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

// 최종 접근 상태로 전환이 가능한 경우에 상태를 FinalApproach로 설정
bool
PrecLand::switch_to_state_final_approach()
{
	if (check_state_conditions(PrecLandState::FinalApproach)) {
		_state = PrecLandState::FinalApproach;
		_state_start_time = hrt_absolute_time();
		return true;
	}

	return false;
}

// 탐색 상태로 전환하면 탐색이 가능한 고도로 이동. 
bool
PrecLand::switch_to_state_search()
{
	PX4_INFO("Climbing to search altitude.");
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.alt = vehicle_local_position->ref_alt + _param_search_alt.get();  // 탐색 고도 설정이 핵심
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_POSITION;
	_navigator->set_position_setpoint_triplet_updated();

	_target_acquired_time = 0;

	_state = PrecLandState::Search;
	_state_start_time = hrt_absolute_time();
	return true;
}

// 일반 착륙 모드로 전환시키기 위해서 sp를 현재 위치로 설정
bool
PrecLand::switch_to_state_fallback()
{
	PX4_WARN("Falling back to normal land.");
	position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.lat = _navigator->get_global_position()->lat;
	pos_sp_triplet->current.lon = _navigator->get_global_position()->lon;
	pos_sp_triplet->current.alt = _navigator->get_global_position()->alt;
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_LAND;
	_navigator->set_position_setpoint_triplet_updated();

	_state = PrecLandState::Fallback;
	_state_start_time = hrt_absolute_time();
	return true;
}

// Done 상태로 전환
bool
PrecLand::switch_to_state_done()
{
	_state = PrecLandState::Done;
	_state_start_time = hrt_absolute_time();
	return true;
}

// 현재 인자로 준 상태에 들어갈 수 있는지 여부를 검사
bool PrecLand::check_state_conditions(PrecLandState state)
{
	vehicle_local_position_s *vehicle_local_position = _navigator->get_local_position();

	switch (state) {
	case PrecLandState::Start: // 검색 횟수 <= 최대 검색 횟수 인 경우 true
		return _search_cnt <= _param_max_searches.get();

	case PrecLandState::HorizontalApproach: // 수평 접근
		// 수평접근 상태에서 허용 범위 안에 있는 경우 pose 정보가 정상이면 true를 반환
		// if we're already in this state, only want to make it invalid if we reached the target but can't see it anymore
		if (_state == PrecLandState::HorizontalApproach) {
			if (fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_hacc_rad.get()
			    && fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_hacc_rad.get()) {
				// we've reached the position where we last saw the target. If we don't see it now, we need to do something
				return _target_pose_valid && _target_pose.abs_pos_valid;

			} else {
				// We've seen the target sometime during horizontal approach.
				// Even if we don't see it as we're moving towards it, continue approaching last known location
				return true;
			}
		}

		// 허용 범위 밖이 경우에는 pose를 업데이트 된 조건이 추가됨. 
		// If we're trying to switch to this state, the target needs to be visible
		return _target_pose_updated && _target_pose_valid && _target_pose.abs_pos_valid;

	case PrecLandState::DescendAboveTarget: // 타겟 위에서 하강

		// 현재 이미 DescendAboveTarget 상태인 경우, 
		// if we're already in this state, only leave it if target becomes unusable, don't care about horizontall offset to target
		if (_state == PrecLandState::DescendAboveTarget) {
			// 다음 단계인 최종 접근이 가능한 상태라면, 0.5초도 안지났는데 최종 접근이 가능한 상태니까 
			// if we're close to the ground, we're more critical of target timeouts so we quickly go into descend
			if (check_state_conditions(PrecLandState::FinalApproach)) {
				return hrt_absolute_time() - _target_pose.timestamp < 500000; // 0.5s

			} else { // 
				return _target_pose_valid && _target_pose.abs_pos_valid;
			}

		} else { //DescendAboveTarget 상태가 아닌 경우
			// 수평 허용 범위내에 있으면 DescendAboveTarget 상태로 전환이 가능하다. 충분히 수평 위치에 가깝제 접근했으니 이제 하강만 하면 된다.
			// if not already in this state, need to be above target to enter it
			return _target_pose_updated && _target_pose.abs_pos_valid
			       && fabsf(_target_pose.x_abs - vehicle_local_position->x) < _param_hacc_rad.get()
			       && fabsf(_target_pose.y_abs - vehicle_local_position->y) < _param_hacc_rad.get();
		}

	case PrecLandState::FinalApproach: // 최종 접근
		return _target_pose_valid && _target_pose.abs_pos_valid
		       && (_target_pose.z_abs - vehicle_local_position->z) < _param_final_approach_alt.get();

	case PrecLandState::Search: // 타겟 검색 중
		return true;

	case PrecLandState::Fallback: //일반 착륙 모드로 
		return true;

	default:
		return false;
	}
}

void PrecLand::slewrate(float &sp_x, float &sp_y)
{
	//현재 x, y로 sp_curr 생성
	matrix::Vector2f sp_curr(sp_x, sp_y);
	uint64_t now = hrt_absolute_time();

	float dt = (now - _last_slewrate_time);

	if (dt < 1) {
		// bad dt, can't divide by it
		return;
	}
	// dt 단위를 sec 단위로 변환
	dt /= SEC2USEC;

	// precland로 전환되고 처음 slewrate를 실행하는 경우 
	if (!_last_slewrate_time) {
		// running the first time since switching to precland

		// 처음에는 dt가 대략 50ms라고 가정
		// assume dt will be about 50000us
		dt = 50000 / SEC2USEC;

		// sp_pev는 현재 비행체 위치와 sp의 lat, lon 사이의 거리
		// 이전 시간에 sp_pev가 sp_pev_prev. 
		// sp_pev 현재 x, y. sp_pev_prev는 과거 sp로 부드럽게 전환 되도록 하기 위해 계산함.
		// set a best guess for previous setpoints for smooth transition
		map_projection_project(&_map_ref, _navigator->get_position_setpoint_triplet()->current.lat,
				       _navigator->get_position_setpoint_triplet()->current.lon, &_sp_pev(0), &_sp_pev(1));
		_sp_pev_prev(0) = _sp_pev(0) - _navigator->get_local_position()->vx * dt; //이전 거리 = 현재 거리 - (현재 속도 * 시간). 이전과 현재가 대략 속도가 같았다고 가정. 
		_sp_pev_prev(1) = _sp_pev(1) - _navigator->get_local_position()->vy * dt;
	}

	_last_slewrate_time = now;

	// sp speed를 최대 비행 속도로 제한한 경우에 sp_curr 구하기
	// limit the setpoint speed to the maximum cruise speed
	matrix::Vector2f sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	// 최대 속도보다 현재 속도(sp_vel)이 더 크면 줄여야함. 따라서 속도(sp_vel)을 가지고 현재의 sp 계산하기(sp_cur)
	if (sp_vel.length() > _param_xy_vel_cruise.get()) {
		sp_vel = sp_vel.normalized() * _param_xy_vel_cruise.get();
		sp_curr = _sp_pev + sp_vel * dt;
	}

	// sp accel을 최대 accel로 제한하기
	// limit the setpoint acceleration to the maximum acceleration
	matrix::Vector2f sp_acc = (sp_curr - _sp_pev * 2 + _sp_pev_prev) / (dt * dt); // acceleration of the setpoints

	// 최대 가속도가 현재 가속도보다 크면 줄여야함. 가속도(sp_acc)를 가지고 현재 sp 계산하기(sp_curr)
	if (sp_acc.length() > _param_acceleration_hor.get()) {
		sp_acc = sp_acc.normalized() * _param_acceleration_hor.get();
		sp_curr = _sp_pev * 2 - _sp_pev_prev + sp_acc * (dt * dt);
	}

	// 최대 가속도가 주어진 경우 특정 sp에 멈출 수 있도록, sp 속도를 제한이 필요. 즉 max_spd 이하 속도여야 sp에 부드럽게 멈추는게 가능하므로.
	// limit the setpoint speed such that we can stop at the setpoint given the maximum acceleration/deceleration
	float max_spd = sqrtf(_param_acceleration_hor.get() * ((matrix::Vector2f)(_sp_pev - matrix::Vector2f(sp_x,
			      sp_y))).length());
	sp_vel = (sp_curr - _sp_pev) / dt; // velocity of the setpoints

	if (sp_vel.length() > max_spd) { // 최대 속도보다 크면 max_spd보다 작게 하기 위해서 normalized()하여 sp_vel을 다시 계산하고 이를 이용해서 sp_curr 다시 계산
		sp_vel = sp_vel.normalized() * max_spd;
		sp_curr = _sp_pev + sp_vel * dt;
	}

	_sp_pev_prev = _sp_pev;
	_sp_pev = sp_curr;

	sp_x = sp_curr(0);
	sp_y = sp_curr(1);
}
