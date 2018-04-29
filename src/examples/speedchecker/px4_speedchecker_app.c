/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file px4_simple_app.c
 * SpeedChecker test application for PX4 autopilot
 *
 * @author Jeyong Shin <jeyong@subak.io>
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/speedchecker_info.h>
#include <drivers/drv_hrt.h>

#define SCHED_PRIORITY_250HZ SCHED_PRIORITY_MAX-5

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */

/**
 * daemon management function.
 */
__EXPORT int speedchecker_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int speedchecker_thread_main(int argc, char *argv[]);


/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}


int speedchecker_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("speedchecker already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("speedchecker",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_250HZ-5,
						 2000,
						 speedchecker_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int speedchecker_thread_main(int argc, char *argv[])
{
	warnx("[speedchecker] starting\n");

	struct speedchecker_info_s speed_info;
	memset(&speed_info, 0, sizeof(speed_info));
	for(int i=0; i<88; i++)
	{
		speed_info.dummy_data[i] = i;
	}
	orb_advert_t speed_info_pub = orb_advertise(ORB_ID(speedchecker_info), &speed_info);

	thread_running = true;

	warnx("Hello speedchecker!\n");

	hrt_abstime last_run = hrt_absolute_time();
	

	double freq = 0;
	double dt = 0;
	int count = 0;
	while (!thread_should_exit) {
		
		speed_info.sequence++;
		speed_info.curtime = hrt_absolute_time();
		orb_publish(ORB_ID(speedchecker_info), speed_info_pub, &speed_info);

		speed_info.sequence = speed_info.sequence % 1000;
		if (speed_info.sequence == 0)
		{
			count++;
			dt = (double) (hrt_absolute_time()-last_run) / 1000000.0;
			freq = (double) (1000 / dt);

			warnx("About %1.2f freq/sec(Hz), deltat = %4.2f, running time : %d sec ", freq, dt, (count*4));

			last_run = hrt_absolute_time();
		}
		usleep(3000);
	}

	warnx("[speedchecker] exiting.\n");

	thread_running = false;

	return 0;
}

