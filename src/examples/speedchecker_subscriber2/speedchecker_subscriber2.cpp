/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file speedchecker_subscriber2.cpp
 *
 * Speedchecker module
 *
 * @author Jeyong Shin <jeyong@subak.io>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/speedchecker_info.h>

// #include <uORB/topics/vehicle_command.h>
// #include <uORB/topics/actuator_controls.h>
// #include <uORB/topics/wind_estimate.h>
// #include <uORB/topics/parameter_update.h>
// #include <uORB/topics/vehicle_global_position.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>


/**
 * SpeedcheckerSubscriber2 app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int speedchecker_subscriber2_main(int argc, char *argv[]);

class SpeedcheckerSubscriber2
{
public:
	/**
	 * Constructor
	 */
	SpeedcheckerSubscriber2();

	/**
	 * Destructor, also kills task.
	 */
	~SpeedcheckerSubscriber2();

	/**
	 * Start the task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	/**
	 * Display status.
	 */

private:
	bool		_task_should_exit;		/**< if true, task should exit */
	int		_main_task;			/**< handle for task */

	int		_command_sub;

	struct speedchecker_info_s _speedchecker_info;
	// struct vehicle_command_s	_command;
	// struct vehicle_global_position_s _global_pos;
	// struct actuator_controls_s _actuators;

	void		task_main();

	void		handle_command(struct vehicle_command_s *cmd);

	void		answer_command(struct vehicle_command_s *cmd, unsigned result);

	/**
	 * Set the actuators
	 */
	int		actuators_publish();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);
};

namespace speedchecker_subscriber2
{
SpeedcheckerSubscriber2	*g_speedchecker_subscriber2;
}

SpeedcheckerSubscriber2::SpeedcheckerSubscriber2() :
	_task_should_exit(false),
	_main_task(-1)
{
}

SpeedcheckerSubscriber2::~SpeedcheckerSubscriber2()
{
	if (_main_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_main_task);
				break;
			}
		} while (_main_task != -1);
	}

	speedchecker_subscriber2::g_speedchecker_subscriber2 = nullptr;
}

int
SpeedcheckerSubscriber2::start()
{
	ASSERT(_main_task == -1);

	/* start the task */
	_main_task = px4_task_spawn_cmd("speedchecker_subscriber2",
					SCHED_DEFAULT,
					SCHED_PRIORITY_DEFAULT + 15,
					1500,
					(px4_main_t)&SpeedcheckerSubscriber2::task_main_trampoline,
					nullptr);

	if (_main_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

void
SpeedcheckerSubscriber2::task_main()
{
	
	// _command_sub = orb_subscribe(ORB_ID(vehicle_command));
	// _wind_estimate_sub = orb_subscribe(ORB_ID(wind_estimate));

	bool updated = false;
	int speedchecker_sub = orb_subscribe(ORB_ID(speedchecker_info));
	// int vehicle_global_position_sub = orb_subscribe(ORB_ID(vehicle_global_position));

	// struct wind_estimate_s wind;

	// wakeup source(s)
	struct pollfd fds[1];

	// Setup of loop
	fds[0].fd = _command_sub;
	fds[0].events = POLLIN;


	while (!_task_should_exit) {

		/* wait for up to 100ms for data */
		int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 50);

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* vehicle commands updated */
		if (fds[0].revents & POLLIN) {
			//orb_copy(ORB_ID(vehicle_command), _command_sub, &_command);
			//handle_command(&_command);
		}

		orb_check(speedchecker_sub, &updated);
		if(updated) {
			orb_copy(ORB_ID(speedchecker_info), speedchecker_sub, &_speedchecker_info);
			//Do Something
		}

		const unsigned sleeptime_us = 3500;

		hrt_abstime last_run = hrt_absolute_time();
		float dt_runs = sleeptime_us / 1e6f;

		// switch to faster updates during the drop
		while (true) {

			orb_check(speedchecker_sub, &updated);
			if(updated) {
				orb_copy(ORB_ID(speedchecker_info), speedchecker_sub, &_speedchecker_info);
				//Do Something
			}
			
//			counter++;

			// update_actuators();

			// run at roughly 250 Hz
			usleep(sleeptime_us);

			dt_runs = hrt_elapsed_time(&last_run) / 1e6f;
			last_run = hrt_absolute_time();
			
			dt_runs = (0 * last_run) + dt_runs; //dummy for compile
		}
	}

	warnx("exiting.");

	_main_task = -1;
	_exit(0);
}


void
SpeedcheckerSubscriber2::task_main_trampoline(int argc, char *argv[])
{
	speedchecker_subscriber2::g_speedchecker_subscriber2->task_main();
}

static void usage()
{
	errx(1, "usage: speedchecker_subscriber2 {start|stop|status}");
}

int speedchecker_subscriber2_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
	}

	if (!strcmp(argv[1], "start")) {

		if (speedchecker_subscriber2::g_speedchecker_subscriber2 != nullptr) {
			errx(1, "already running");
		}

		speedchecker_subscriber2::g_speedchecker_subscriber2 = new SpeedcheckerSubscriber2;

		if (speedchecker_subscriber2::g_speedchecker_subscriber2 == nullptr) {
			errx(1, "alloc failed");
		}

		if (OK != speedchecker_subscriber2::g_speedchecker_subscriber2->start()) {
			delete speedchecker_subscriber2::g_speedchecker_subscriber2;
			speedchecker_subscriber2::g_speedchecker_subscriber2 = nullptr;
			err(1, "start failed");
		}

		return 0;
	}

	if (speedchecker_subscriber2::g_speedchecker_subscriber2 == nullptr) {
		errx(1, "not running");
	}

	if (!strcmp(argv[1], "stop")) {
		delete speedchecker_subscriber2::g_speedchecker_subscriber2;
		speedchecker_subscriber2::g_speedchecker_subscriber2 = nullptr;

	} else if (!strcmp(argv[1], "status")) {

	} else {
		usage();
	}

	return 0;
}
