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
 * @file subscriber_example.cpp
 *
 * @author Jeyong Shin <jeyong@subak.io>
 */

#include <px4_posix.h>
#include <controllib/blocks.hpp>
#include "subscriber.hpp"

/*
SpeedCheckerSubscriber::SpeedCheckerSubscriber()
{
	SpeedCheckerSubscriber(nullptr, "speedchecker_subscriber");
}
*/

SpeedCheckerSubscriber::SpeedCheckerSubscriber(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),
	// subscriptions
	_speedchecker_info(ORB_ID(speedchecker_info), 4, 0, &getSubscriptions()) //4ms
{
		_attPoll.fd = _speedchecker_info.getHandle();
		_attPoll.events = POLLIN;
}

void SpeedCheckerSubscriber::update()
{
		// wait for a sensor update, check for exit condition every 4 ms(250Hz)
	if (px4_poll(&_attPoll, 1, 4) < 0) { return; } // poll error

	uint64_t newTimeStamp = hrt_absolute_time();
	float dt = (newTimeStamp - _timeStamp) / 1.0e6f;
	_timeStamp = newTimeStamp;

	// check for sane values of dt
	// to prevent large control responses
	if (dt > 1.0f || dt < 0) { return; }

	// set dt for all child blocks
	setDt(dt);

	// check for new updates
//	if (_param_update.updated()) { updateParams(); }

	// get new information from subscriptions
	updateSubscriptions();

	_speedchecker_info.get().sequence;

}