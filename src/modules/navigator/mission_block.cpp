/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mission_block.cpp
 *
 * Helper class to use mission items
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Sander Smeets <sander@droneslab.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include "mission_block.h"
#include "navigator.h"

#include <math.h>
#include <float.h>

#include <lib/ecl/geo/geo.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vtol_vehicle_status.h>

using matrix::wrap_pi;

MissionBlock::MissionBlock(Navigator *navigator) :
	NavigatorMode(navigator)
{
}

bool
MissionBlock::is_mission_item_reached()
{
	/* handle non-navigation or indefinite waypoints */
	// NAV_CMD_DO_Xxxx(mission cmd)의 경우 명령이 전달되었다는 것으로 true으로 반환
	switch (_mission_item.nav_cmd) {
	case NAV_CMD_DO_SET_SERVO:
		return true;

	case NAV_CMD_LAND: /* fall through */
	case NAV_CMD_VTOL_LAND:
		return _navigator->get_land_detected()->landed;

	case NAV_CMD_IDLE: /* fall through */
	case NAV_CMD_LOITER_UNLIMITED:
		return false;

	case NAV_CMD_DO_LAND_START:
	case NAV_CMD_DO_TRIGGER_CONTROL:
	case NAV_CMD_DO_DIGICAM_CONTROL:
	case NAV_CMD_IMAGE_START_CAPTURE:
	case NAV_CMD_IMAGE_STOP_CAPTURE:
	case NAV_CMD_VIDEO_START_CAPTURE:
	case NAV_CMD_VIDEO_STOP_CAPTURE:
	case NAV_CMD_DO_MOUNT_CONFIGURE:
	case NAV_CMD_DO_MOUNT_CONTROL:
	case NAV_CMD_DO_SET_ROI:
	case NAV_CMD_DO_SET_ROI_LOCATION:
	case NAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
	case NAV_CMD_DO_SET_ROI_NONE:
	case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
	case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
	case NAV_CMD_SET_CAMERA_MODE:
		return true;

	case NAV_CMD_DO_VTOL_TRANSITION:

		/*
		 * We wait half a second to give the transition command time to propagate.
		 * Then monitor the transition status for completion.
		 */
		// TODO: check desired transition state achieved and drop _action_start
		if (hrt_absolute_time() - _action_start > 500000 &&
		    !_navigator->get_vstatus()->in_transition_mode) {

			_action_start = 0;
			return true;

		} else {
			return false;
		}

	case NAV_CMD_DO_CHANGE_SPEED:
	case NAV_CMD_DO_SET_HOME:
		return true;

	default:
		/* do nothing, this is a 3D waypoint */
		break;
	}

	hrt_abstime now = hrt_absolute_time();

	// 미착륙 & waypoint 위치 미도착인 경우
	if (!_navigator->get_land_detected()->landed && !_waypoint_position_reached) {

		float dist = -1.0f;
		float dist_xy = -1.0f;
		float dist_z = -1.0f;

		float altitude_amsl = _mission_item.altitude_is_relative
				      ? _mission_item.altitude + _navigator->get_home_position()->alt
				      : _mission_item.altitude;

		dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, altitude_amsl,
				_navigator->get_global_position()->lat,
				_navigator->get_global_position()->lon,
				_navigator->get_global_position()->alt,
				&dist_xy, &dist_z);

		// fw 무시
		/* FW special case for NAV_CMD_WAYPOINT to achieve altitude via loiter */
		if (!_navigator->get_vstatus()->is_rotary_wing &&
		    (_mission_item.nav_cmd == NAV_CMD_WAYPOINT)) {

			struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			/* close to waypoint, but altitude error greater than twice acceptance */
			if ((dist >= 0.0f)
			    && (dist_z > 2 * _navigator->get_altitude_acceptance_radius())
			    && (dist_xy < 2 * _navigator->get_loiter_radius())) {

				/* SETPOINT_TYPE_POSITION -> SETPOINT_TYPE_LOITER */
				if (curr_sp->type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
					curr_sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
					curr_sp->loiter_radius = _navigator->get_loiter_radius();
					curr_sp->loiter_direction = 1;
					_navigator->set_position_setpoint_triplet_updated();
				}

			} else {
				/* restore SETPOINT_TYPE_POSITION */
				if (curr_sp->type == position_setpoint_s::SETPOINT_TYPE_LOITER) {
					/* loiter acceptance criteria required to revert back to SETPOINT_TYPE_POSITION */
					if ((dist >= 0.0f)
					    && (dist_z < _navigator->get_loiter_radius())
					    && (dist_xy <= _navigator->get_loiter_radius() * 1.2f)) {

						curr_sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
						_navigator->set_position_setpoint_triplet_updated();
					}
				}
			}
		}

		// takeoff 명령인 경우 고도만 허용 범위 안에 들어왔는지 체크한다.
		if ((_mission_item.nav_cmd == NAV_CMD_TAKEOFF || _mission_item.nav_cmd == NAV_CMD_VTOL_TAKEOFF)
		    && _navigator->get_vstatus()->is_rotary_wing) {

			/* We want to avoid the edge case where the acceptance radius is bigger or equal than
			 * the altitude of the takeoff waypoint above home. Otherwise, we do not really follow
			 * take-off procedures like leaving the landing gear down. */

			float takeoff_alt = _mission_item.altitude_is_relative ?
					    _mission_item.altitude :
					    (_mission_item.altitude - _navigator->get_home_position()->alt);

			float altitude_acceptance_radius = _navigator->get_altitude_acceptance_radius();

			/* It should be safe to just use half of the takoeff_alt as an acceptance radius. */
			if (takeoff_alt > 0 && takeoff_alt < altitude_acceptance_radius) {
				altitude_acceptance_radius = takeoff_alt / 2.0f;
			}

			/* require only altitude for takeoff for multicopter */
			if (_navigator->get_global_position()->alt >
			    altitude_amsl - altitude_acceptance_radius) {
				_waypoint_position_reached = true;
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_TAKEOFF) { //vtol인 경우 무시
			/* for takeoff mission items use the parameter for the takeoff acceptance radius */
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius()
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				_waypoint_position_reached = true;
			}

		} else if (!_navigator->get_vstatus()->is_rotary_wing &&  //mc가 아닌 경우이므로 무시
			   (_mission_item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
			    _mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT)) {
			/* Loiter mission item on a non rotary wing: the aircraft is going to circle the
			 * coordinates with a radius equal to the loiter_radius field. It is not flying
			 * through the waypoint center.
			 * Therefore the item is marked as reached once the system reaches the loiter
			 * radius (+ some margin). Time inside and turn count is handled elsewhere.
			 */
			if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius(fabsf(_mission_item.loiter_radius) * 1.2f)
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {

				_waypoint_position_reached = true;

			} else {
				_time_first_inside_orbit = 0;
			}

		} else if (!_navigator->get_vstatus()->is_rotary_wing &&  //mc의 loiter하는 경우 
			   (_mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT)) {

			// NAV_CMD_LOITER_TO_ALT는 mission . 
			// NAV_CMD_LOITER_TO_ALT only uses mission item altitude once it's in the loiter
			//  first check if the altitude setpoint is the mission setpoint
			struct position_setpoint_s *curr_sp = &_navigator->get_position_setpoint_triplet()->current;

			// 현재 고도와 loiter 타겟 고도차이가 허용 범위 밖인 경우 
			if (fabsf(curr_sp->alt - altitude_amsl) >= FLT_EPSILON) {
				// check if the initial loiter has been accepted
				dist_xy = -1.0f;
				dist_z = -1.0f;

				// 타겟 위치와 현재 위치의 거리를 구하고 이 거리가 허용 범위면 고도만 설정한다.
				dist = get_distance_to_point_global_wgs84(_mission_item.lat, _mission_item.lon, curr_sp->alt,
						_navigator->get_global_position()->lat,
						_navigator->get_global_position()->lon,
						_navigator->get_global_position()->alt,
						&dist_xy, &dist_z);

				if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius(fabsf(_mission_item.loiter_radius) * 1.2f)
				    && dist_z <= _navigator->get_altitude_acceptance_radius()) {

					// now set the loiter to the final altitude in the NAV_CMD_LOITER_TO_ALT mission item
					curr_sp->alt = altitude_amsl;
					_navigator->set_position_setpoint_triplet_updated();
				}

			} else { // 현재 위치와 고도가 허용 범위 안에 있는 경우, 최종적으로 yaw 설정해 줘야하는 경우라면(force_heading) yaw를 설정
				if (dist >= 0.0f && dist <= _navigator->get_acceptance_radius(fabsf(_mission_item.loiter_radius) * 1.2f)
				    && dist_z <= _navigator->get_altitude_acceptance_radius()) {

					_waypoint_position_reached = true;

					// set required yaw from bearing to the next mission item
					if (_mission_item.force_heading) {
						struct position_setpoint_s next_sp = _navigator->get_position_setpoint_triplet()->next;

						if (next_sp.valid) {
							_mission_item.yaw = get_bearing_to_next_waypoint(_navigator->get_global_position()->lat,
									    _navigator->get_global_position()->lon,
									    next_sp.lat, next_sp.lon);

							_waypoint_yaw_reached = false;

						} else {
							_waypoint_yaw_reached = true;
						}
					}
				}
			}

		} else if (_mission_item.nav_cmd == NAV_CMD_DELAY) {
			_waypoint_position_reached = true;
			_waypoint_yaw_reached = true;
			_time_wp_reached = now;

		} else { // 이외 mission cmd인 경우
			/* for normal mission items used their acceptance radius */
			float mission_acceptance_radius = _navigator->get_acceptance_radius(_mission_item.acceptance_radius);

			/* if set to zero use the default instead */
			if (mission_acceptance_radius < NAV_EPSILON_POSITION) {
				mission_acceptance_radius = _navigator->get_acceptance_radius();
			}

			/* for vtol back transition calculate acceptance radius based on time and ground speed */
			if (_mission_item.vtol_back_transition && !_navigator->get_vstatus()->is_rotary_wing) {

				float velocity = sqrtf(_navigator->get_local_position()->vx * _navigator->get_local_position()->vx +
						       _navigator->get_local_position()->vy * _navigator->get_local_position()->vy);

				const float back_trans_dec = _navigator->get_vtol_back_trans_deceleration();
				const float reverse_delay = _navigator->get_vtol_reverse_delay();

				if (back_trans_dec > FLT_EPSILON && velocity > FLT_EPSILON) {
					mission_acceptance_radius = ((velocity / back_trans_dec / 2) * velocity) + reverse_delay * velocity;

				}

			}

			if (dist >= 0.0f && dist <= mission_acceptance_radius
			    && dist_z <= _navigator->get_altitude_acceptance_radius()) {
				_waypoint_position_reached = true;
			}
		}

		if (_waypoint_position_reached) {
			// reached just now
			_time_wp_reached = now;
		}
	}

	// wp 도착 & yaw는 아직 미도착
	/* Check if the waypoint and the requested yaw setpoint. */
	if (_waypoint_position_reached && !_waypoint_yaw_reached) {

		if ((_navigator->get_vstatus()->is_rotary_wing
		     || (_mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT && _mission_item.force_heading))
		    && PX4_ISFINITE(_mission_item.yaw)) {

			/* check course if defined only for rotary wing except takeoff */
			float cog = _navigator->get_vstatus()->is_rotary_wing ? _navigator->get_global_position()->yaw : atan2f(
					    _navigator->get_global_position()->vel_e,
					    _navigator->get_global_position()->vel_n);

			float yaw_err = wrap_pi(_mission_item.yaw - cog);

			/* accept yaw if reached or if timeout is set in which case we ignore not forced headings */
			if (fabsf(yaw_err) < math::radians(_navigator->get_yaw_threshold())
			    || (_navigator->get_yaw_timeout() >= FLT_EPSILON && !_mission_item.force_heading)) {

				_waypoint_yaw_reached = true;
			}

			/* if heading needs to be reached, the timeout is enabled and we don't make it, abort mission */
			if (!_waypoint_yaw_reached && _mission_item.force_heading &&
			    (_navigator->get_yaw_timeout() >= FLT_EPSILON) &&
			    (now - _time_wp_reached >= (hrt_abstime)_navigator->get_yaw_timeout() * 1e6f)) {

				_navigator->set_mission_failure("unable to reach heading within timeout");
			}

		} else {
			_waypoint_yaw_reached = true;
		}
	}

	// wp 도착 & yaw 도착
	// loiter 상태로 얼마나 있었는지 보기 위해서 시간 카운트 시작
	/* Once the waypoint and yaw setpoint have been reached we can start the loiter time countdown */
	if (_waypoint_position_reached && _waypoint_yaw_reached) {

		if (_time_first_inside_orbit == 0) {
			_time_first_inside_orbit = now;
		}

		/* check if the MAV was long enough inside the waypoint orbit */
		if ((get_time_inside(_mission_item) < FLT_EPSILON) ||
		    (now - _time_first_inside_orbit >= (hrt_abstime)(get_time_inside(_mission_item) * 1e6f))) {

			position_setpoint_s &curr_sp = _navigator->get_position_setpoint_triplet()->current;
			const position_setpoint_s &next_sp = _navigator->get_position_setpoint_triplet()->next;

			const float range = get_distance_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);

			// exit xtrack location
			// reset lat/lon of loiter waypoint so vehicle follows a tangent
			if (_mission_item.loiter_exit_xtrack && next_sp.valid && PX4_ISFINITE(range) &&
			    (_mission_item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
			     _mission_item.nav_cmd == NAV_CMD_LOITER_TO_ALT)) {

				float bearing = get_bearing_to_next_waypoint(curr_sp.lat, curr_sp.lon, next_sp.lat, next_sp.lon);
				float inner_angle = M_PI_2_F - asinf(_mission_item.loiter_radius / range);

				// Compute "ideal" tangent origin
				if (curr_sp.loiter_direction > 0) {
					bearing -= inner_angle;

				} else {
					bearing += inner_angle;
				}

				// Replace current setpoint lat/lon with tangent coordinate
				waypoint_from_heading_and_distance(curr_sp.lat, curr_sp.lon,
								   bearing, curr_sp.loiter_radius,
								   &curr_sp.lat, &curr_sp.lon);
			}

			return true;
		}
	}

	// true 조건에 못들어간 경우 wp 및 yaw 모두 false인 상태.
	// all acceptance criteria must be met in the same iteration
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	return false;
}

void
MissionBlock::reset_mission_item_reached()
{
	_waypoint_position_reached = false;
	_waypoint_yaw_reached = false;
	_time_first_inside_orbit = 0;
	_time_wp_reached = 0;
}

void
MissionBlock::issue_command(const mission_item_s &item)
{
	// item에 위치 정보가 있는 경우 comm가 아니므로 그냥 return
	if (item_contains_position(item)) {
		return;
	}

	// LAND_START는 land 상태로 들어갔다는 maker로 사용하므로 return
	// NAV_CMD_DO_LAND_START is only a marker
	if (item.nav_cmd == NAV_CMD_DO_LAND_START) {
		return;
	}

	// fw이나 카메라 처리 이므로 무시
	if (item.nav_cmd == NAV_CMD_DO_SET_SERVO) {
		PX4_INFO("DO_SET_SERVO command");

		// XXX: we should issue a vehicle command and handle this somewhere else
		actuator_controls_s actuators = {};
		actuators.timestamp = hrt_absolute_time();

		// params[0] actuator number to be set 0..5 (corresponds to AUX outputs 1..6)
		// params[1] new value for selected actuator in ms 900...2000
		actuators.control[(int)item.params[0]] = 1.0f / 2000 * -item.params[1];

		if (_actuator_pub != nullptr) {
			orb_publish(ORB_ID(actuator_controls_2), _actuator_pub, &actuators);

		} else {
			_actuator_pub = orb_advertise(ORB_ID(actuator_controls_2), &actuators);
		}

	} else {
		// mission item 중에 comm으로 사용가능한 경우 vehicle cmd로 변환하여 publish. 
		_action_start = hrt_absolute_time();

		// mission_item -> vehicle_command

		// we're expecting a mission command item here so assign the "raw" inputs to the command
		// (MAV_FRAME_MISSION mission item)
		vehicle_command_s vcmd = {};
		vcmd.command = item.nav_cmd;
		vcmd.param1 = item.params[0];
		vcmd.param2 = item.params[1];
		vcmd.param3 = item.params[2];
		vcmd.param4 = item.params[3];
		vcmd.param5 = item.params[4];
		vcmd.param6 = item.params[5];

		if (item.nav_cmd == NAV_CMD_DO_SET_ROI_LOCATION && item.altitude_is_relative) {
			vcmd.param7 = item.params[6] + _navigator->get_home_position()->alt;

		} else {
			vcmd.param7 = item.params[6];
		}

		_navigator->publish_vehicle_cmd(&vcmd);
	}
}

//mission item의 time_inside 값을 가져옴
float
MissionBlock::get_time_inside(const struct mission_item_s &item)
{
	// takeoff시에는 명령에 도달하는 시간 count하지 않으므로
	if (item.nav_cmd != NAV_CMD_TAKEOFF) {
		return item.time_inside;
	}

	return 0.0f;
}

// 아래와 같은 명령에만 position 정보가 들어가 있음
bool
MissionBlock::item_contains_position(const mission_item_s &item)
{
	return item.nav_cmd == NAV_CMD_WAYPOINT ||
	       item.nav_cmd == NAV_CMD_LOITER_UNLIMITED ||
	       item.nav_cmd == NAV_CMD_LOITER_TIME_LIMIT ||
	       item.nav_cmd == NAV_CMD_LAND ||
	       item.nav_cmd == NAV_CMD_TAKEOFF ||
	       item.nav_cmd == NAV_CMD_LOITER_TO_ALT ||
	       item.nav_cmd == NAV_CMD_VTOL_TAKEOFF ||
	       item.nav_cmd == NAV_CMD_VTOL_LAND ||
	       item.nav_cmd == NAV_CMD_DO_FOLLOW_REPOSITION;
}

//mission item으로 sp의 값을 설정. sp 값을 채우는 것이 목적!
bool
MissionBlock::mission_item_to_position_setpoint(const mission_item_s &item, position_setpoint_s *sp)
{
	// pos 정보를 없는 item인 경우 중단
	/* don't change the setpoint for non-position items */
	if (!item_contains_position(item)) {
		return false;
	}

	sp->lat = item.lat;
	sp->lon = item.lon;
	sp->alt = item.altitude_is_relative ? item.altitude + _navigator->get_home_position()->alt : item.altitude; //상대 고도는 home고도 기준으로 얼마나 높이 있는지를 뜻함
	sp->yaw = item.yaw;
	sp->yaw_valid = PX4_ISFINITE(item.yaw);
	sp->loiter_radius = (fabsf(item.loiter_radius) > NAV_EPSILON_POSITION) ? fabsf(item.loiter_radius) :
			    _navigator->get_loiter_radius();
	sp->loiter_direction = (item.loiter_radius > 0) ? 1 : -1;
	sp->acceptance_radius = item.acceptance_radius;

	sp->cruising_speed = _navigator->get_cruising_speed();
	sp->cruising_throttle = _navigator->get_cruising_throttle();

	switch (item.nav_cmd) {
	case NAV_CMD_IDLE:
		sp->type = position_setpoint_s::SETPOINT_TYPE_IDLE;
		break;

	case NAV_CMD_TAKEOFF:
		// takeoff의 경우 이미 armed 상태면 위치의 이동이므로 positon 타입이 되고 armed가 안된 경우에는 takeoff 타입이 됨.
		// if already flying (armed and !landed) treat TAKEOFF like regular POSITION
		if ((_navigator->get_vstatus()->arming_state == vehicle_status_s::ARMING_STATE_ARMED)
		    && !_navigator->get_land_detected()->landed) {

			sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;

		} else {
			sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;

			// set pitch and ensure that the hold time is zero
			sp->pitch_min = item.pitch_min;
		}

		break;

	case NAV_CMD_VTOL_TAKEOFF:
		sp->type = position_setpoint_s::SETPOINT_TYPE_TAKEOFF;
		break;

	case NAV_CMD_LAND:
	case NAV_CMD_VTOL_LAND:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LAND;
		break;

	case NAV_CMD_LOITER_TO_ALT:
		// 처음에는 현재 고도를 사용. 나중에 loiter 위치로 들어가게 되면 mission의 고도로 전환
		// initially use current altitude, and switch to mission item altitude once in loiter position
		if (_navigator->get_loiter_min_alt() > 0.0f) { // ignore _param_loiter_min_alt if smaller then 0 (-1)
			sp->alt = math::max(_navigator->get_global_position()->alt,
					    _navigator->get_home_position()->alt + _navigator->get_loiter_min_alt());

		} else {
			sp->alt = _navigator->get_global_position()->alt;
		}

	// fall through //여기서 처럼 loiter모드에서는 mission item의 고도를 사용.
	case NAV_CMD_LOITER_TIME_LIMIT:
	case NAV_CMD_LOITER_UNLIMITED:
		sp->type = position_setpoint_s::SETPOINT_TYPE_LOITER;
		break;

	default:
		sp->type = position_setpoint_s::SETPOINT_TYPE_POSITION;
		break;
	}

	sp->valid = true;

	return sp->valid;
}

// loiter에 필요한 mission item 설정. 인자로 최소 마진 값을 넘겨줌
void
MissionBlock::set_loiter_item(struct mission_item_s *item, float min_clearance)
{
	if (_navigator->get_land_detected()->landed) {
		// 이미 착륙한 상태면 idle로 변환
		/* landed, don't takeoff, but switch to IDLE mode */
		item->nav_cmd = NAV_CMD_IDLE;

	} else { // 
		item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;

		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();

		// 현재 위치에서 loiter가 가능하고 현재 pos가 유효한 경우 현재 pos 위치를 mission item에 넣어줌
		if (_navigator->get_can_loiter_at_sp() && pos_sp_triplet->current.valid) {
			/* use current position setpoint */
			item->lat = pos_sp_triplet->current.lat;
			item->lon = pos_sp_triplet->current.lon;
			item->altitude = pos_sp_triplet->current.alt;

		} else { // pos가 유효하지 않은 경우 gpos를 사용해서 lat, lon, 고도를 설정. gpos의 고도와 home고도 + 마진 중에 큰 값을 선택하도록.
			/* use current position and use return altitude as clearance */
			item->lat = _navigator->get_global_position()->lat;
			item->lon = _navigator->get_global_position()->lon;
			item->altitude = _navigator->get_global_position()->alt;

			if (min_clearance > 0.0f && item->altitude < _navigator->get_home_position()->alt + min_clearance) {
				item->altitude = _navigator->get_home_position()->alt + min_clearance;
			}
		}

		item->altitude_is_relative = false;
		item->yaw = NAN;
		item->loiter_radius = _navigator->get_loiter_radius();
		item->acceptance_radius = _navigator->get_acceptance_radius();
		item->time_inside = 0.0f;
		item->autocontinue = false;
		item->origin = ORIGIN_ONBOARD;
	}
}

//takeoff에 필요한 mission item을 생성
void
MissionBlock::set_takeoff_item(struct mission_item_s *item, float abs_altitude, float min_pitch)
{
	item->nav_cmd = NAV_CMD_TAKEOFF;

	/* use current position */
	item->lat = _navigator->get_global_position()->lat;
	item->lon = _navigator->get_global_position()->lon;
	item->yaw = _navigator->get_global_position()->yaw;

	item->altitude = abs_altitude;
	item->altitude_is_relative = false;

	item->loiter_radius = _navigator->get_loiter_radius();
	item->pitch_min = min_pitch;
	item->autocontinue = false;
	item->origin = ORIGIN_ONBOARD;
}

//land에 필요한 mission item을 생성. 현재 위치에 착륙할 것인지 home 위치에 착륙할 것인지 결정하는 인자에 따라 lat, lon, yaw 설정 
void
MissionBlock::set_land_item(struct mission_item_s *item, bool at_current_location)
{
	// VTOL인 경우 무시 
	/* VTOL transition to RW before landing */
	if (_navigator->force_vtol()) {

		vehicle_command_s vcmd = {};
		vcmd.command = NAV_CMD_DO_VTOL_TRANSITION;
		vcmd.param1 = vtol_vehicle_status_s::VEHICLE_VTOL_STATE_MC;
		_navigator->publish_vehicle_cmd(&vcmd);
	}

	/* set the land item */
	item->nav_cmd = NAV_CMD_LAND;

	/* use current position */
	if (at_current_location) {
		item->lat = NAN; //descend at current position
		item->lon = NAN; //descend at current position
		item->yaw = _navigator->get_local_position()->yaw;

	} else {
		/* use home position */
		item->lat = _navigator->get_home_position()->lat;
		item->lon = _navigator->get_home_position()->lon;
		item->yaw = _navigator->get_home_position()->yaw;
	}

	item->altitude = 0;
	item->altitude_is_relative = false;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

// idle 상태에 필요한 mission item 생성. cmd 타입을 NAV_CMD_IDLE로 설정.
void
MissionBlock::set_idle_item(struct mission_item_s *item)
{
	item->nav_cmd = NAV_CMD_IDLE; // 이것이 핵심
	item->lat = _navigator->get_home_position()->lat;
	item->lon = _navigator->get_home_position()->lon;
	item->altitude_is_relative = false;
	item->altitude = _navigator->get_home_position()->alt;
	item->yaw = NAN;
	item->loiter_radius = _navigator->get_loiter_radius();
	item->acceptance_radius = _navigator->get_acceptance_radius();
	item->time_inside = 0.0f;
	item->autocontinue = true;
	item->origin = ORIGIN_ONBOARD;
}

//vtol 무시
void
MissionBlock::set_vtol_transition_item(struct mission_item_s *item, const uint8_t new_mode)
{
	item->nav_cmd = NAV_CMD_DO_VTOL_TRANSITION;
	item->params[0] = (float) new_mode;
	item->yaw = _navigator->get_global_position()->yaw;
	item->autocontinue = true;
}

// mission 실행되기 전에 각 값이 허용하는 범위를 벗어나는 경우 허용 범위로 값을 조정해 주는 역할
// mission 실행 직전에 호출되는 방식.
// 현재는 고도에 대한 제약만 포함되어 있고 추가로 허용범위 체크가 필요한 경우 여기에다가 추가해 주면 됨.
// 현재는 mission item의 고도 설정에 관한 부분만 구현되어 있음 (고도에 대한 체크 및 설정) 
void
MissionBlock::mission_apply_limitation(mission_item_s &item)
{
	/*
	 * Limit altitude
	 */

	/* do nothing if altitude max is negative */
	if (_navigator->get_land_detected()->alt_max > 0.0f) {

		/* absolute altitude */ // 절대 고도
		float altitude_abs = item.altitude_is_relative
				     ? item.altitude + _navigator->get_home_position()->alt
				     : item.altitude;

		/* limit altitude to maximum allowed altitude */
		if ((_navigator->get_land_detected()->alt_max + _navigator->get_home_position()->alt) < altitude_abs) {
			item.altitude = item.altitude_is_relative ?
					_navigator->get_land_detected()->alt_max :
					_navigator->get_land_detected()->alt_max + _navigator->get_home_position()->alt;

		}
	}

	/*
	 * Add other limitations here
	 */
}

// mission 에 고도 계산해서 넣기
float
MissionBlock::get_absolute_altitude_for_item(struct mission_item_s &mission_item) const
{
	if (mission_item.altitude_is_relative) {
		return mission_item.altitude + _navigator->get_home_position()->alt;

	} else {
		return mission_item.altitude;
	}
}
