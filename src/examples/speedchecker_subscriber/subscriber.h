/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file subscriber_example.h
 * Example subscriber for ros and px4
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */
#include <px4.h>
#include <uORB/uORB.h>
#include <uORB/topics/speedchecker_info.h>

using namespace px4;

void speedchecker_info_function(const speedchecker_info_s &msg);

class SpeedCheckerSubscriber
{
public:
	SpeedCheckerSubscriber();

	~SpeedCheckerSubscriber() {}

	void spin() {_n.spin();}

protected:
	px4::NodeHandle _n;
	// px4::ParameterInt _p_sub_interv;
	// px4::ParameterFloat _p_test_float;
	px4::Subscriber<speedchecker_info_s> *_sub_speedchecker_info;

	AppState _appState;

	void speedchecker_info_callback(const speedchecker_info_s &msg);

	// void rc_channels_callback(const px4_rc_channels &msg);
	// void vehicle_attitude_callback(const px4_vehicle_attitude &msg);
	// void parameter_update_callback(const px4_parameter_update &msg);

};
