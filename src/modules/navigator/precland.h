/***************************************************************************
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
 * @file precland.h
 *
 * Helper class to do precision landing with a landing target
 *
 * @author Nicolas de Palezieux (Sunflower Labs) <ndepal@gmail.com>
 */

#pragma once

#include <matrix/math.hpp>
#include <lib/ecl/geo/geo.h>
#include <px4_module_params.h>
#include <uORB/topics/landing_target_pose.h>

#include "navigator_mode.h"
#include "mission_block.h"

enum class PrecLandState {
	Start, // Starting state
	HorizontalApproach, // Positioning over landing target while maintaining altitude 수평 접근
	DescendAboveTarget, // Stay over landing target while descending  타겟 위에서 하강
	FinalApproach, // Final landing approach, even without landing target 마지막 접근
	Search, // Search for landing target  탐색
	Fallback, // Fallback landing method 일반 착륙 모드로 동작
	Done // Done landing  착륙 완료
};

enum class PrecLandMode {
	Opportunistic = 1, // only do precision landing if landing target visible at the beginning 처음에 타겟이 있는 경우에만 prec landing 동작
	Required = 2 // try to find landing target if not visible at the beginning  처음에 타겟 못찾아도 탐색 동작 시도
};

class PrecLand : public MissionBlock, public ModuleParams
{
public:
	PrecLand(Navigator *navigator);
	~PrecLand() override = default;

	void on_activation() override;
	void on_active() override;

	void set_mode(PrecLandMode mode) { _mode = mode; };

	PrecLandMode get_mode() { return _mode; };

private:
	// 각 상태에 따른 control loop 수행
	// run the control loop for each state
	void run_state_start();
	void run_state_horizontal_approach();
	void run_state_descend_above_target();
	void run_state_final_approach();
	void run_state_search();
	void run_state_fallback();

	// 다른 state로 전환 시도. state 변환이 성공적으로 이뤄지면 true. 
	// attempt to switch to a different state. Returns true if state change was successful, false otherwise
	bool switch_to_state_start();
	bool switch_to_state_horizontal_approach();
	bool switch_to_state_descend_above_target();
	bool switch_to_state_final_approach();
	bool switch_to_state_search();
	bool switch_to_state_fallback();
	bool switch_to_state_done();

	// 해당 state로 전환이 가능하다면 true 반환.
	// check if a given state could be changed into. Return true if possible to transition to state, false otherwise
	bool check_state_conditions(PrecLandState state);
	void slewrate(float &sp_x, float &sp_y);

	landing_target_pose_s _target_pose{}; /**< precision landing target position */

	int _target_pose_sub{-1};
	bool _target_pose_valid{false}; /**< whether we have received a landing target position message / 타겟 위치 메시지를 수신 여부 */
	bool _target_pose_updated{false}; /**< wether the landing target position message is updated / 타겟 위치 메시지 업데이트 여부 */

	// projection을 위한 reference
	struct map_projection_reference_s _map_ref {}; /**< reference for local/global projections */

	uint64_t _state_start_time{0}; /**< time when we entered current state  /현재 상태로 진입한 시간 */
	uint64_t _last_slewrate_time{0}; /**< time when we last limited setpoint changes /제한된 sp가 지속된 시간 */
	uint64_t _target_acquired_time{0}; /**< time when we first saw the landing target during search /탐색시 처음 타겟을 발견한 시간 */
	uint64_t _point_reached_time{0}; /**< time when we reached a setpoint /sp에 도달했을 때 시간  */

	int _search_cnt{0}; /**< counter of how many times we had to search for the landing target /타겟 탐색 시도 횟수 */
	float _approach_alt{0.0f}; /**< altitude at which to stay during horizontal approach /수평 접근시 사용하는 고도 */

	matrix::Vector2f _sp_pev;
	matrix::Vector2f _sp_pev_prev;

	PrecLandState _state{PrecLandState::Start};

	PrecLandMode _mode{PrecLandMode::Opportunistic};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::PLD_BTOUT>) _param_timeout,
		(ParamFloat<px4::params::PLD_HACC_RAD>) _param_hacc_rad,
		(ParamFloat<px4::params::PLD_FAPPR_ALT>) _param_final_approach_alt,
		(ParamFloat<px4::params::PLD_SRCH_ALT>) _param_search_alt,
		(ParamFloat<px4::params::PLD_SRCH_TOUT>) _param_search_timeout,
		(ParamInt<px4::params::PLD_MAX_SRCH>) _param_max_searches,
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_acceleration_hor,
		(ParamFloat<px4::params::MPC_XY_CRUISE>) _param_xy_vel_cruise
	)

};
