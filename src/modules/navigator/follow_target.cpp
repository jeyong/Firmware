/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file followme.cpp
 *
 * Helper class to track and follow a given position
 *
 * @author Jimmy Johnson <catch22@fastmail.net>
 */

#include "follow_target.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/follow_target.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/math/Limits.hpp>

#include "navigator.h"

using matrix::wrap_pi;

constexpr float FollowTarget::_follow_position_matricies[4][9];

FollowTarget::FollowTarget(Navigator *navigator) :
	MissionBlock(navigator),
	ModuleParams(navigator)
{
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	_target_position_delta.zero();
}

void FollowTarget::on_inactive()
{
	reset_target_validity();
}

void FollowTarget::on_activation()
{
	// 추적시 거리 설정. 최소 값은 1m. 
	_follow_offset = _param_tracking_dist.get() < 1.0F ? 1.0F : _param_tracking_dist.get();

	// 반응성 0.1 ~ 1.0까지 값으로 제한
	_responsiveness = math::constrain((float) _param_tracking_resp.get(), .1F, 1.0F);

	// 추적시 위치 (앞, 뒤 혹은 왼쪽, 오른쪽 등 설정)
	_follow_target_position = _param_tracking_side.get();

	if ((_follow_target_position > FOLLOW_FROM_LEFT) || (_follow_target_position < FOLLOW_FROM_RIGHT)) {
		_follow_target_position = FOLLOW_FROM_BEHIND;
	}

	// 회전 행렬 (어느쪽에서 추적하느냐에 따라서 회전 행렬 초기화)
	_rot_matrix = (_follow_position_matricies[_follow_target_position]);

	if (_follow_target_sub < 0) {
		_follow_target_sub = orb_subscribe(ORB_ID(follow_target));
	}
}

void FollowTarget::on_active()
{
	struct map_projection_reference_s target_ref;
	follow_target_s target_motion_with_offset = {};
	uint64_t current_time = hrt_absolute_time();
	bool _radius_entered = false;
	bool _radius_exited = false;
	bool updated = false;
	float dt_ms = 0;

	orb_check(_follow_target_sub, &updated);

	// 추적하는 타겟 정보가 업데이트 된 경우에 현재 타겟 motion 정보를 업데이트
	// 목적 : _current_target_motion을 최신 값으로 업데이트
	if (updated) {
		follow_target_s target_motion;

		_target_updates++;

		// save last known motion topic

		_previous_target_motion = _current_target_motion;

		orb_copy(ORB_ID(follow_target), _follow_target_sub, &target_motion);

		if (_current_target_motion.timestamp == 0) {
			_current_target_motion = target_motion;
		}

		_current_target_motion.timestamp = target_motion.timestamp;
		_current_target_motion.lat = (_current_target_motion.lat * (double)_responsiveness) + target_motion.lat * (double)(
						     1 - _responsiveness);
		_current_target_motion.lon = (_current_target_motion.lon * (double)_responsiveness) + target_motion.lon * (double)(
						     1 - _responsiveness);

	} else if (((current_time - _current_target_motion.timestamp) / 1000) > TARGET_TIMEOUT_MS && target_velocity_valid()) {
		reset_target_validity();
	}

	// 타겟까지 거리 업데이트
	// update distance to target
	if (target_position_valid()) {

		// get distance to target

		map_projection_init(&target_ref, _navigator->get_global_position()->lat, _navigator->get_global_position()->lon);
		map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon, &_target_distance(0),
				       &_target_distance(1));

	}

	// 타겟 속도가 업데이트 된 경우에 -> target_motion 값에서 offset을 적용한 target_motion_with_offset 값을 계산하는게 목적. 
	// update target velocity
	if (target_velocity_valid() && updated) {

		dt_ms = ((_current_target_motion.timestamp - _previous_target_motion.timestamp) / 1000);

		// 최소 10ms 마다 타겟의 이동 거리를 구해서 결국 타겟의 이동 속도 구하기.  ignore a small dt
		if (dt_ms > 10.0F) {
			// 가장 마지막에 알고 있는 타겟의 lat, lon 정보를 ref로 설정
			// get last gps known reference for target
			map_projection_init(&target_ref, _previous_target_motion.lat, _previous_target_motion.lon);

			// 타겟이 이동한 거리 구하기 (이전 타겟의 lat, lon 기준으로 현재 motion 위치 정보로 이동한 거리 구하였음)
			// calculate distance the target has moved
			map_projection_project(&target_ref, _current_target_motion.lat, _current_target_motion.lon,
					       &(_target_position_delta(0)), &(_target_position_delta(1)));

			// pos 기반으로 타겟의 평균 속도 계산. 속도 = 이동거리 / 시간
			// update the average velocity of the target based on the position
			_est_target_vel = _target_position_delta / (dt_ms / 1000.0f); // m/s

			// estimation한 속도가 0.5 이상인 경우 영향을 많이 미치므로 이 경우 offset을 반영해야함.
			// 
			// 타겟의 속도가 얼마 이상이면 offset과 rotation을 추가 
			// if the target is moving add an offset and rotation
			if (_est_target_vel.length() > .5F) {
				_target_position_offset = _rot_matrix * _est_target_vel.normalized() * _follow_offset;
			}

			// 타겟과의 거리가 1.5배를 벗어나면 허용 반경을 벗어났다고 설정. 반경 내부에 있으면 반경 내부에 있다고 설정.
			// 속도 제어기가 해당 반경 내에 들어갈 수 있는 기회를 주기 위해서 버퍼(exited는 1.5배)를 좀더 준다.
			// are we within the target acceptance radius?
			// give a buffer to exit/enter the radius to give the velocity controller
			// a chance to catch up

			_radius_exited = ((_target_position_offset + _target_distance).length() > (float) TARGET_ACCEPTANCE_RADIUS_M * 1.5f);
			_radius_entered = ((_target_position_offset + _target_distance).length() < (float) TARGET_ACCEPTANCE_RADIUS_M);

			// 속도 증감을 부드럽게 유지하기 위해서 얼마나 속도를 증가 시키거나 감소 시킬지 계산.
			// 몇 단계로 타겟의 속도와 같아질지 정하고 이 단계에 따라 속도 증감시키는 방식.
			// 만약 타겟과 속도 차이가 더 벌어지는 경우 속도는 다시 계산 하도록 한다.
			// to keep the velocity increase/decrease smooth
			// calculate how many velocity increments/decrements
			// it will take to reach the targets velocity
			// with the given amount of steps also add a feed forward input that adjusts the
			// velocity as the position gap increases since
			// just traveling at the exact velocity of the target will not
			// get any closer or farther from the target

			_step_vel = (_est_target_vel - _current_vel) + (_target_position_offset + _target_distance) * FF_K;
			_step_vel /= (dt_ms / 1000.0F * (float) INTERPOLATION_PNTS);
			_step_time_in_ms = (dt_ms / (float) INTERPOLATION_PNTS);

			// 타겟과의 거리가 1미터 이하인 경우 yaw 설정하지 않ㅇ므. 1미터 이상 벌어지면 그때부터 yaw 설정이 필요. 
			// if we are less than 1 meter from the target don't worry about trying to yaw
			// lock the yaw until we are at a distance that makes sense

			if ((_target_distance).length() > 1.0F) {
				// 1m 이상 거리인 경우 yaw와 yaw_rate를 계산하기. yaw_rate는 attitude 제어기에서 사용. 
				// yaw rate smoothing

				// this really needs to control the yaw rate directly in the attitude pid controller
				// but seems to work ok for now since the yaw rate cannot be controlled directly in auto mode

				_yaw_angle = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_current_target_motion.lat,
						_current_target_motion.lon);

				_yaw_rate = wrap_pi((_yaw_angle - _navigator->get_global_position()->yaw) / (dt_ms / 1000.0f));

			} else { // 1m 이하면 yaw는 설정 안함.
				_yaw_angle = _yaw_rate = NAN;
			}
		}

//		warnx(" _step_vel x %3.6f y %3.6f cur vel %3.6f %3.6f tar vel %3.6f %3.6f dist = %3.6f (%3.6f) mode = %d yaw rate = %3.6f",
//				(double) _step_vel(0),
//				(double) _step_vel(1),
//				(double) _current_vel(0),
//				(double) _current_vel(1),
//				(double) _est_target_vel(0),
//				(double) _est_target_vel(1),
//				(double) (_target_distance).length(),
//				(double) (_target_position_offset + _target_distance).length(),
//				_follow_target_state,
//				(double) _yaw_rate);
	}

	// 현재 target_motion 기준으로 offset이 반영한 lat, lon 계산. 타겟 위치가 유효한 경우 lat, lon 계산
	if (target_position_valid()) {

		// offset을 반영한 lat, lon 계산
		// get the target position using the calculated offset
		map_projection_init(&target_ref,  _current_target_motion.lat, _current_target_motion.lon);
		map_projection_reproject(&target_ref, _target_position_offset(0), _target_position_offset(1),
					 &target_motion_with_offset.lat, &target_motion_with_offset.lon);
	}

	// yaw차이가 3도 이내 인 경우 yaw_rate를 NAN으로 설정하여 yaw_rate가 부드럽게 되도록 함.
	// clamp yaw rate smoothing if we are with in
	// 3 degrees of facing target
	if (PX4_ISFINITE(_yaw_rate)) {
		if (fabsf(fabsf(_yaw_angle) - fabsf(_navigator->get_global_position()->yaw)) < math::radians(3.0F)) {
			_yaw_rate = NAN;
		}
	}

	// update state machine

	switch (_follow_target_state) {

	case TRACK_POSITION: {
			// 허용 반경내에 들어오면 이제 타겟의 속도를 맞추는 상태인 TRACK_VELOCITY로 설정
			if (_radius_entered == true) {
				_follow_target_state = TRACK_VELOCITY;

			} else if (target_velocity_valid()) { // 일반 상황 : 타겟 속도를 정상적으로 계산한 경우
				// target motion으로 mission item 설정
				set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);
				// keep the current velocity updated with the target velocity for when it's needed
				_current_vel = _est_target_vel;

				update_position_sp(true, true, _yaw_rate);

			} else { // 타겟 위치를 기다리는 모드로 (타겟 pos 정보를 못 가지고 온 경우이므로 타겟 pos를 얻기 위해서 wait모드로 설정)
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	case TRACK_VELOCITY: {
			// 반경을 벗어나면 다시 TRACK_POSITION 상태로 설정
			if (_radius_exited == true) {
				_follow_target_state = TRACK_POSITION;

			} else if (target_velocity_valid()) { // 일반 상황 : 타겟 속도를 정상적으로 계산한 경우
				// 주기적으로 속도 계산 업데이트 current_vel
				if ((float)(current_time - _last_update_time) / 1000.0f >= _step_time_in_ms) {
					_current_vel += _step_vel;
					_last_update_time = current_time;
				}

				set_follow_target_item(&_mission_item, _param_min_alt.get(), target_motion_with_offset, _yaw_angle);

				update_position_sp(true, false, _yaw_rate);

			} else { // 타겟 위치를 기다리는 모드로 (타겟 pos 정보를 못 가지고 온 경우이므로 타겟 pos를 얻기 위해서 wait모드로 설정)
				_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
			}

			break;
		}

	case SET_WAIT_FOR_TARGET_POSITION: {
			// 최소 고도로 가서 타겟의 pos 정보를 정상적으로 받을때 까지 기다림
			// Climb to the minimum altitude
			// and wait until a position is received

			follow_target_s target = {};

			// 현재 타겟 정보를 받지 못한 경우이므로 일단 gpos의 lat, lon을 사용하고 고도는 최소 고도 값으로 설정
			// for now set the target at the minimum height above the uav
			target.lat = _navigator->get_global_position()->lat;
			target.lon = _navigator->get_global_position()->lon;
			target.alt = 0.0F;

			set_follow_target_item(&_mission_item, _param_min_alt.get(), target, _yaw_angle);

			update_position_sp(false, false, _yaw_rate);

			_follow_target_state = WAIT_FOR_TARGET_POSITION;
		}

	/* FALLTHROUGH */

	case WAIT_FOR_TARGET_POSITION: {
			// 타겟 pos 정도를 정상적으로 받은 상태가 되면 타겟 위치 설정하고 TRACK_POSITION 상태로 설정
			if (is_mission_item_reached() && target_velocity_valid()) {
				_target_position_offset(0) = _follow_offset;
				_follow_target_state = TRACK_POSITION;
			}

			break;
		}
	}
}

// mission item으로부터 sp값을 설정.  
void FollowTarget::update_position_sp(bool use_velocity, bool use_position, float yaw_rate)
{
	// convert mission item to current setpoint

	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

	// activate line following in pos control if position is valid

	pos_sp_triplet->previous.valid = use_position;
	pos_sp_triplet->previous = pos_sp_triplet->current;
	mission_apply_limitation(_mission_item);
	mission_item_to_position_setpoint(_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.type = position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET;
	pos_sp_triplet->current.position_valid = use_position; //pos 정보가 유효한지 인자로부터 설정
	pos_sp_triplet->current.velocity_valid = use_velocity; //vel 정보가 유효한지 인자로부터 설정
	pos_sp_triplet->current.vx = _current_vel(0);
	pos_sp_triplet->current.vy = _current_vel(1);
	pos_sp_triplet->next.valid = false;
	pos_sp_triplet->current.yawspeed_valid = PX4_ISFINITE(yaw_rate);
	pos_sp_triplet->current.yawspeed = yaw_rate;
	_navigator->set_position_setpoint_triplet_updated();
}

// 사용하는 상태 변수들 모두 리셋
void FollowTarget::reset_target_validity()
{
	_yaw_rate = NAN;
	_previous_target_motion = {};
	_current_target_motion = {};
	_target_updates = 0;
	_current_vel.zero();
	_step_vel.zero();
	_est_target_vel.zero();
	_target_distance.zero();
	_target_position_offset.zero();
	reset_mission_item_reached();
	_follow_target_state = SET_WAIT_FOR_TARGET_POSITION;
}

// 타겟 업데이트가 연속으로 최소 2번 이상 이뤄졌는지 (속도가 나올려면 위치의 변화가 있어야 하므로 2회 이상 필요)
bool FollowTarget::target_velocity_valid()
{
	// need at least 2 continuous data points for velocity estimate
	return (_target_updates >= 2);
}

//타겟 업데이트가 최소 1번 이상 이뤄졌는지로 결정
bool FollowTarget::target_position_valid()
{
	// need at least 1 continuous data points for position estimate
	return (_target_updates >= 1);
}

// follow target을 위한 mission item을 설정. 
void
FollowTarget::set_follow_target_item(struct mission_item_s *item, float min_clearance, follow_target_s &target,
				     float yaw)
{
	// 이미 착륙한 상태면 idle로 설정
	if (_navigator->get_land_detected()->landed) {
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else { // 일반 상황 :
		// 이동할 위치를 타겟의 lat, lon으로 설정. 고도는 home 고도에 마진 고도 더하기
		item->nav_cmd = NAV_CMD_DO_FOLLOW_REPOSITION;

		/* use current target position */
		item->lat = target.lat;
		item->lon = target.lon;
		item->altitude = _navigator->get_home_position()->alt;

		// 최소 8m 이상 고도 마진 추가 
		if (min_clearance > 8.0f) {
			item->altitude += min_clearance;

		} else {
			item->altitude += 8.0f; // if min clearance is bad set it to 8.0 meters (well above the average height of a person)
		}
	}

	item->altitude_is_relative = false;
	item->yaw = yaw;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}
