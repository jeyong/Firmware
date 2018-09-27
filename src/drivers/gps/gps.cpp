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
 * @file gps.cpp
 * Driver for the GPS on a serial/spi port
 */

#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif


#include <termios.h>

#ifndef __PX4_QURT
#include <poll.h>
#endif


#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_config.h>
#include <px4_getopt.h>
#include <px4_module.h>
#include <px4_tasks.h>
#include <px4_time.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <systemlib/err.h>
#include <parameters/param.h>
#include <drivers/drv_gps.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>
#include <uORB/topics/gps_inject_data.h>
#include <uORB/topics/gps_dump.h>

#include <board_config.h>

#include "devices/src/ubx.h"
#include "devices/src/mtk.h"
#include "devices/src/ashtech.h"


#define TIMEOUT_5HZ 500 // 5Hz로 동작 
#define RATE_MEASUREMENT_PERIOD 5000000 // 5초
#define GPS_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls // read() 호출이 정상적으로 동작하기 위해서 읽기 동작 전에 20ms 대기


/* struct for dynamic allocation of satellite info data */
struct GPS_Sat_Info {
	struct satellite_info_s 	_data;
};


class GPS : public ModuleBase<GPS>
{
public:

	/** The GPS allows to run multiple instances */
	enum class Instance : uint8_t {
		Main = 0,
		Secondary,

		Count
	};

	GPS(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface, bool fake_gps, bool enable_sat_info,
	    Instance instance);
	virtual ~GPS();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** spawn task and select the instance */
	static int task_spawn(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static GPS *instantiate(int argc, char *argv[]);

	static GPS *instantiate(int argc, char *argv[], Instance instance);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/**
	 * task spawn trampoline for the secondary GPS
	 */
	static int run_trampoline_secondary(int argc, char *argv[]);

	/** @see ModuleBase::run() */
	void run() override;

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	int print_status() override;

private:

	int				_serial_fd;					///< serial interface to GPS // GPS serial fd 
	unsigned			_baudrate;					///< current baudrate // 현재 baudrate
	char				_port[20];					///< device / serial port path // serial 포트 path
	bool				_healthy;					///< flag to signal if the GPS is ok // GPS가 정상인지 여부
	bool				_baudrate_changed;				///< flag to signal that the baudrate with the GPS has changed // GPS의 baudrate가 변했는지 여부를 나타내는 flag
	bool				_mode_changed;					///< flag that the GPS mode has changed // GPS 모드가 변경되었는지 여부를 나타내는 flag
	bool        			_mode_auto;					///< if true, auto-detect which GPS is attached // GPS 장착여부 자동 감지 
	gps_driver_mode_t		_mode;						///< current mode // 현재 모드 (uBlox 모드 사용)
	GPSHelper::Interface  _interface;   						///< interface // 인터페이스
	GPSHelper			*_helper;					///< instance of GPS parser // GPS 파서 
	GPS_Sat_Info			*_sat_info;					///< instance of GPS sat info data object // GPS sattelite 정보를 담고 있는 객체
	struct vehicle_gps_position_s	_report_gps_pos;				///< uORB topic for gps position // gps position을 위한 uORB topic
	orb_advert_t			_report_gps_pos_pub;				///< uORB pub for gps position // gps position을 uORB pub
	int					_gps_orb_instance;				///< uORB multi-topic instance
	struct satellite_info_s		*_p_report_sat_info;				///< pointer to uORB topic for satellite info // GPS satellite 정보를 위한 uORB topic에 대한 pointer
	int					_gps_sat_orb_instance;				///< uORB multi-topic instance for satellite info // GPS satellite 정보를 위한 uORB instance
	orb_advert_t			_report_sat_info_pub;				///< uORB pub for satellite info // GPS satellite 정보를 위한 uORB pub
	float				_rate;						///< position update rate // position 정보를 업데이트하는 rate
	float				_rate_rtcm_injection;				///< RTCM message injection rate //  RTCM 메시지 추가 rate
	unsigned			_last_rate_rtcm_injection_count; 		///< counter for number of RTCM messages // RTCM 메시지의 횟수
	bool				_fake_gps;					///< fake gps output
	Instance 			_instance;

	int _orb_inject_data_fd;

	orb_advert_t _dump_communication_pub;			///< if non-null, dump communication // null이 아닌 경우 dump communication
	gps_dump_s *_dump_to_device;   	// GPS로 보내는 데이터
	gps_dump_s *_dump_from_device;  // GPS로부터 받은 데이터

	static volatile bool _is_gps_main_advertised; ///< for the second gps we want to make sure that it gets instance 1
	/// and thus we wait until the first one publishes at least one message.
	// 2번째 gps는 instance 1을 가져야하므로 첫번째 gps가 최소한 하나의 메시지를 publish할때까지 기다리기
	static volatile GPS *_secondary_instance;


	/**
	 * GPS 설정, GPS로 가는 통신 처리
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void config();

	/**
	 * UART의 baudrate 설정 (GPS와 통신 속도)
	 * Set the baudrate of the UART to the GPS
	 */
	int set_baudrate(unsigned baud);

	/**
	 * GPS에게 reset 명령 보내기 
	 * Send a reset command to the GPS
	 */
	void cmd_reset();

	/**
	 * gps 정보를 publish
	 * Publish the gps struct
	 */
	void 				publish();

	/**
	 * staelliteInfo를 publish
	 * Publish the satellite info
	 */
	void 				publishSatelliteInfo();

	/**
	 * 시리얼 장치에서 사용하는 poll에 대한 추상화
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param buf: pointer to read buffer // read 버퍼의 포인터
	 * @param buf_length: size of read buffer // read 버퍼의 크기
	 * @param timeout: timeout in ms // ms단위 timeout 시간
	 * @return: 0 for nothing read, or poll timed out // 0 : 읽을 것이 없거나 timeout, 음수 : error, 양수 : 읽은 byte 수
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);

	/**
	 * 새로운 inject data topic이 있는지 검사하고 이를 처리
	 * check for new messages on the inject data topic & handle them
	 */
	void handleInjectDataTopic();

	/**
	 * data를 gps에 보내기 (RTCM stream)
	 * send data to the device, such as an RTCM stream
	 * @param data
	 * @param len
	 */
	inline bool injectData(uint8_t *data, size_t len);

	/**
	 * baudrate 설정
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);

	/**
	 * callback from the driver for the platform specific stuff
	 */
	static int callback(GPSCallbackType type, void *data1, int data2, void *user);

	/**
	 * msg_to_gps_device가 true이면 gps 장치로 메시지 보내고 false이면 gps에서 메시지를 받음
	 * Dump gps communication.
	 * @param data message
	 * @param len length of the message
	 * @param msg_to_gps_device if true, this is a message sent to the gps device, otherwise it's from the device
	 */
	void dumpGpsData(uint8_t *data, size_t len, bool msg_to_gps_device);

	void initializeCommunicationDump();
};

volatile bool GPS::_is_gps_main_advertised = false;
volatile GPS *GPS::_secondary_instance = nullptr;

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gps_main(int argc, char *argv[]);


GPS::GPS(const char *path, gps_driver_mode_t mode, GPSHelper::Interface interface, bool fake_gps,
	 bool enable_sat_info, Instance instance) :
	_serial_fd(-1),
	_healthy(false),
	_mode_changed(false),
	_mode(mode),
	_interface(interface),
	_helper(nullptr),
	_sat_info(nullptr),
	_report_gps_pos{},
	_report_gps_pos_pub(nullptr),
	_gps_orb_instance(-1),
	_p_report_sat_info(nullptr),
	_report_sat_info_pub(nullptr),
	_rate(0.0f),
	_rate_rtcm_injection(0.0f),
	_last_rate_rtcm_injection_count(0),
	_fake_gps(fake_gps),
	_instance(instance),
	_orb_inject_data_fd(-1),
	_dump_communication_pub(nullptr),
	_dump_to_device(nullptr),
	_dump_from_device(nullptr)
{
	/* store port name */
	strncpy(_port, path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* create satellite info data object if requested */
	if (enable_sat_info) {
		_sat_info = new GPS_Sat_Info();
		_p_report_sat_info = &_sat_info->_data;
		memset(_p_report_sat_info, 0, sizeof(*_p_report_sat_info));
	}

	if (mode == GPS_DRIVER_MODE_NONE) {
		_mode_auto = true;
	}
}

GPS::~GPS()
{
	GPS *secondary_instance = (GPS *) _secondary_instance;

	if (_instance == Instance::Main && secondary_instance) {
		secondary_instance->request_stop();

		// wait for it to exit
		unsigned int i = 0;

		do {
			usleep(20000); // 20 ms
			++i;
		} while (_secondary_instance && i < 100);
	}

	if (_sat_info) {
		delete (_sat_info);
	}

	if (_dump_to_device) {
		delete (_dump_to_device);
	}

	if (_dump_from_device) {
		delete (_dump_from_device);
	}

}

int GPS::callback(GPSCallbackType type, void *data1, int data2, void *user)
{
	GPS *gps = (GPS *)user;

	switch (type) {
	case GPSCallbackType::readDeviceData: { // gps에서 읽기고 size 반환
			int num_read = gps->pollOrRead((uint8_t *)data1, data2, *((int *)data1));

			if (num_read > 0) {
				gps->dumpGpsData((uint8_t *)data1, (size_t)num_read, false);
			}

			return num_read;
		}

	case GPSCallbackType::writeDeviceData: // gps에 쓰기
		gps->dumpGpsData((uint8_t *)data1, (size_t)data2, true);

		return write(gps->_serial_fd, data1, (size_t)data2);

	case GPSCallbackType::setBaudrate: //Baud rate 속도 설정
		return gps->setBaudrate(data2);

	case GPSCallbackType::gotRTCMMessage:
		/* not used */
		break;

	case GPSCallbackType::surveyInStatus:
		/* not used */
		break;

	case GPSCallbackType::setClock: // 시간 설정
		px4_clock_settime(CLOCK_REALTIME, (timespec *)data1);
		break;
	}

	return 0;
}

// 
int GPS::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
	handleInjectDataTopic();

#if !defined(__PX4_QURT)

	/* For non QURT, use the usual polling. */

	// serial data를 polling. 하나의 thread에서 orb messaage도 처리해야 하므로, 동시에 시리얼과 orb로부터 polling을 수행.
	// 이 2가지 polling은 서로 다른 매커니즘을 사용. (orb subscribe와 uart에서 수신)
	// 최대 polling 구간을 두고 주기적으로 orb 메시지를 체크하는 방식 사용
	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	//FIXME: add a unified poll() API
	const int max_timeout = 50;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	// uart로 polling 하기
	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	if (ret > 0) {
		// GPS에서 새로운 data가 있으면 이를 처리
		/* if we have new data from GPS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * 여기 진입한 경우 polling을 통해서 데이터가 있는 상태,
			 * 따라서 read() 호출에 시간이 많이 걸리므로 1-2 byte 정도 들어온 경우 바로 읽어들이지 않고 delay를 둔다.
			 * 요청한 data가 온전히 온 경우에는 delay 없이 바로 읽기
			 * 
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If we have all requested data available, read it without waiting.
			 * If more bytes are available, we'll go back to poll() again.
			 */
#ifdef __PX4_NUTTX
			int err = 0;
			int bytesAvailable = 0;
			err = ioctl(_serial_fd, FIONREAD, (unsigned long)&bytesAvailable);

			if ((err != 0) || (bytesAvailable < (int)buf_length)) {
				usleep(GPS_WAIT_BEFORE_READ * 1000);
			}

#else
			// read() 호출 하기전에 시간 dealy를 둔다. read() 호출에 cost가 많이 드니까
			usleep(GPS_WAIT_BEFORE_READ * 1000);
#endif

			ret = ::read(_serial_fd, buf, buf_length);

		} else {
			ret = -1;
		}
	}

	return ret;

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	usleep(10000);
	return ::read(_serial_fd, buf, buf_length);
#endif
}
// gps_inject_data를 subscribeㅏ여  
void GPS::handleInjectDataTopic()
{
	if (_orb_inject_data_fd == -1) {
		return;
	}

	bool updated = false;

	do {
		orb_check(_orb_inject_data_fd, &updated);

		if (updated) {
			struct gps_inject_data_s msg;
			orb_copy(ORB_ID(gps_inject_data), _orb_inject_data_fd, &msg);

			/* 
			 * gps 장치로 메시지를 쓰기. 메시지가 조각으로 나뉘어져 있더라도 동작 중에는 그냥 gps 장치로 보내기.
			 * 별도로 message를 온전한 상태로 만들 필요 없음. 
			 * 즉 받은 조각 data 그대로 쓰기
			 * Write the message to the gps device. Note that the message could be fragmented.
			 * But as we don't write anywhere else to the device during operation, we don't
			 * need to assemble the message first.
			 */
			injectData(msg.data, msg.len);

			++_last_rate_rtcm_injection_count;
		}
	} while (updated);
}

bool GPS::injectData(uint8_t *data, size_t len)
{
	dumpGpsData(data, len, true);

	size_t written = ::write(_serial_fd, data, len);
	::fsync(_serial_fd);
	return written == len;
}

int GPS::setBaudrate(unsigned baud)
{
	/* process baud rate */
	int speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit, disable flow control */
	uart_config.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		GPS_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		GPS_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		GPS_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

	return 0;
}

// param에서 dump 정보에 따라 dump -> device, file -> dump, GPS로부터 받은 dump를 publish 준비
void GPS::initializeCommunicationDump()
{
	//  1로 설정된 경우 모든 GPS 통신 데이터를 uORB로 publish하고 log 파일로 저정할 수 있음
	param_t gps_dump_comm_ph = param_find("GPS_DUMP_COMM");
	int32_t param_dump_comm;

	if (gps_dump_comm_ph == PARAM_INVALID || param_get(gps_dump_comm_ph, &param_dump_comm) != 0) {
		return;
	}

	if (param_dump_comm != 1) {
		return; //dumping disabled
	}

	_dump_from_device = new gps_dump_s();
	_dump_to_device = new gps_dump_s();

	if (!_dump_from_device || !_dump_to_device) {
		PX4_ERR("failed to allocated dump data");
		return;
	}

	memset(_dump_to_device, 0, sizeof(gps_dump_s));
	memset(_dump_from_device, 0, sizeof(gps_dump_s));

	int instance;
	// 충분히 큰 queue를 사용해야 메시지를 잃어버리는 일이 없으며 다른 방법으로는 logger rate 속도를 빨리하는 방법이 있음
	// _dump_from_device는 uORB로 보내는 dump
	//make sure to use a large enough queue size, so that we don't lose messages. You may also want
	//to increase the logger rate for that.
	_dump_communication_pub = orb_advertise_multi_queue(ORB_ID(gps_dump), _dump_from_device, &instance,
				  ORB_PRIO_DEFAULT, 8);
}

// GPS 데이터 dump를 publish
void GPS::dumpGpsData(uint8_t *data, size_t len, bool msg_to_gps_device)
{
	// dump 관련 param 설정이 안되어 있는 겨우 dump 데이터를 모으지 않음.
	if (!_dump_communication_pub) {
		return;
	}

	// dump할 데이터 선택 (gps로부터 받은 data or gps로 보낼 data 선택하기)
	gps_dump_s *dump_data = msg_to_gps_device ? _dump_to_device : _dump_from_device;

	while (len > 0) {
		size_t write_len = len;

		// 쓸려고 하는 크기 > data 크기 - 길이
		if (write_len > sizeof(dump_data->data) - dump_data->len) {
			write_len = sizeof(dump_data->data) - dump_data->len;
		}

		// 
		memcpy(dump_data->data + dump_data->len, data, write_len);
		data += write_len;
		dump_data->len += write_len;
		len -= write_len;

		if (dump_data->len >= sizeof(dump_data->data)) {
			if (msg_to_gps_device) { // gps로 보내는 data인 경우 len의 맨 앞 비트를 '1'로 설정
				dump_data->len |= 1 << 7;
			}

			// dump 데이터 publish 하기 
			dump_data->timestamp = hrt_absolute_time();
			orb_publish(ORB_ID(gps_dump), _dump_communication_pub, dump_data);
			dump_data->len = 0;
		}
	}
}

void
GPS::run()
{
	if (!_fake_gps) {
		// serial 포트 열기 
		/* open the serial port */
		_serial_fd = ::open(_port, O_RDWR | O_NOCTTY); // "/dev/ttyS3"

		if (_serial_fd < 0) {
			PX4_ERR("GPS: failed to open serial port: %s err: %d", _port, errno);
			return;
		}
	}

	_orb_inject_data_fd = orb_subscribe(ORB_ID(gps_inject_data));

	// 통신 dump 초기화
	initializeCommunicationDump();

	uint64_t last_rate_measurement = hrt_absolute_time();
	unsigned last_rate_count = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!should_exit()) {

		if (_fake_gps) {
			_report_gps_pos = {};
			_report_gps_pos.timestamp = hrt_absolute_time();
			_report_gps_pos.lat = (int32_t)47.378301e7f;
			_report_gps_pos.lon = (int32_t)8.538777e7f;
			_report_gps_pos.alt = (int32_t)1200e3f;
			_report_gps_pos.alt_ellipsoid = 10000;
			_report_gps_pos.s_variance_m_s = 0.5f;
			_report_gps_pos.c_variance_rad = 0.1f;
			_report_gps_pos.fix_type = 3;
			_report_gps_pos.eph = 0.8f;
			_report_gps_pos.epv = 1.2f;
			_report_gps_pos.hdop = 0.9f;
			_report_gps_pos.vdop = 0.9f;
			_report_gps_pos.vel_n_m_s = 0.0f;
			_report_gps_pos.vel_e_m_s = 0.0f;
			_report_gps_pos.vel_d_m_s = 0.0f;
			_report_gps_pos.vel_m_s = 0.0f;
			_report_gps_pos.cog_rad = 0.0f;
			_report_gps_pos.vel_ned_valid = true;
			_report_gps_pos.satellites_used = 10;

			/* no time and satellite information simulated */


			publish();

			usleep(200000);

		} else {

			if (_helper != nullptr) {
				delete (_helper);
				_helper = nullptr;
			}

			switch (_mode) {
			case GPS_DRIVER_MODE_NONE:
				_mode = GPS_DRIVER_MODE_UBX;

			/* FALLTHROUGH */
			case GPS_DRIVER_MODE_UBX: {
					int32_t param_gps_ubx_dynmodel = 7; // default to 7: airborne with <2g acceleration
					param_get(param_find("GPS_UBX_DYNMODEL"), &param_gps_ubx_dynmodel);

					_helper = new GPSDriverUBX(_interface, &GPS::callback, this, &_report_gps_pos, _p_report_sat_info,
								   param_gps_ubx_dynmodel);
				}
				break;

			case GPS_DRIVER_MODE_MTK:
				_helper = new GPSDriverMTK(&GPS::callback, this, &_report_gps_pos);
				break;

			case GPS_DRIVER_MODE_ASHTECH:
				_helper = new GPSDriverAshtech(&GPS::callback, this, &_report_gps_pos, _p_report_sat_info);
				break;

			default:
				break;
			}

			// Ashtech 드라이버는 실제 설정에 문제가 있어도 성공되었다고 나오는 문제 있음.
			// MTK 드라이버는 테스트 제대로 안되었음.
			// 현재 UBlox 드라이버만 제대로 테스트 되어 신뢰할 수 있음. 
			/* the Ashtech driver lies about successful configuration and the
			 * MTK driver is not well tested, so we really only trust the UBX
			 * driver for an advance publication
			 */
			// UBloxDriver의 configure() 호출
			if (_helper && _helper->configure(_baudrate, GPSHelper::OutputMode::GPS) == 0) {

				// report 초기화
				/* reset report */
				memset(&_report_gps_pos, 0, sizeof(_report_gps_pos));

				if (_mode == GPS_DRIVER_MODE_UBX) {
					// 여기 들어오는 경우 GPS가 정상적으로 검색된 상태로, 실제로 읽기 동작 전에 사용하는 변수들 초기화 수행
					/* GPS is obviously detected successfully, reset statistics */
					_helper->resetUpdateRates();
				}

				int helper_ret;
				// 5Hz timeout 전에 읽기 성공한 경우 // UBloxDriver의 receive() 호출
				while ((helper_ret = _helper->receive(TIMEOUT_5HZ)) > 0 && !should_exit()) {

					if (helper_ret & 1) { // 정상적으로 읽어온 경우 publish()를 호출하여 정보를 publish 수행
						publish();

						last_rate_count++;
					}

					if (_p_report_sat_info && (helper_ret & 2)) { // 2를 반환한 경우 satellite정보를 publish
						publishSatelliteInfo();
					}

					// 5초마다 update rate를 측정 (RATE_MEASUREMENT_PERIOD는 5초로 설정)
					/* measure update rate every 5 seconds */
					if (hrt_absolute_time() - last_rate_measurement > RATE_MEASUREMENT_PERIOD) {
						float dt = (float)((hrt_absolute_time() - last_rate_measurement)) / 1000000.0f;
						_rate = last_rate_count / dt;
						_rate_rtcm_injection = _last_rate_rtcm_injection_count / dt;
						last_rate_measurement = hrt_absolute_time();
						last_rate_count = 0; // 새로 측정을 위한 변수 초기화
						_last_rate_rtcm_injection_count = 0; // 새로 측정을 위한 변수 초기화
						_helper->storeUpdateRates(); // 속도, 위치의 update rate를 저장
						_helper->resetUpdateRates(); // 새로운 측정을 위한 사용변수들 초기화
					}

					if (!_healthy) { // 디버깅 목적으로 두었는데 현재는 처리 부분이 없음
						// Helpful for debugging, but too verbose for normal ops
						//						const char *mode_str = "unknown";
						//
						//						switch (_mode) {
						//						case GPS_DRIVER_MODE_UBX:
						//							mode_str = "UBX";
						//							break;
						//
						//						case GPS_DRIVER_MODE_MTK:
						//							mode_str = "MTK";
						//							break;
						//
						//						case GPS_DRIVER_MODE_ASHTECH:
						//							mode_str = "ASHTECH";
						//							break;
						//
						//						default:
						//							break;
						//						}
						//
						//						PX4_WARN("module found: %s", mode_str);
						_healthy = true;
					}
				}

				if (_healthy) { // health 관련 변수 초기화
					_healthy = false;
					_rate = 0.0f;
					_rate_rtcm_injection = 0.0f;
				}
			}

			// GPS의 경우 동적으로 연결 및 해제가 가능하고 다른 GPS 모델이 붙더라도 동작할 수 있게 매번 검사하도록 구현
			if (_mode_auto) { // auto모드로 설정된 경우 다른 드라이버에 대해서도 주기적으로 체크
				switch (_mode) {
				case GPS_DRIVER_MODE_UBX:
					_mode = GPS_DRIVER_MODE_MTK;
					break;

				case GPS_DRIVER_MODE_MTK:
					_mode = GPS_DRIVER_MODE_ASHTECH;
					break;

				case GPS_DRIVER_MODE_ASHTECH:
					_mode = GPS_DRIVER_MODE_UBX;
					usleep(500000); // tried all possible drivers. Wait a bit before next round
					break;

				default:
					break;
				}

			} else {
				usleep(500000);
			}

		}
	}

	PX4_INFO("exiting");

	orb_unsubscribe(_orb_inject_data_fd);

	if (_dump_communication_pub) {
		orb_unadvertise(_dump_communication_pub);
	}

	if (_serial_fd >= 0) {
		::close(_serial_fd);
		_serial_fd = -1;
	}

	orb_unadvertise(_report_gps_pos_pub);
}



void
GPS::cmd_reset()
{
#ifdef GPIO_GPS_NRESET
	PX4_WARN("Toggling GPS reset pin");
	px4_arch_configgpio(GPIO_GPS_NRESET);
	px4_arch_gpiowrite(GPIO_GPS_NRESET, 0);
	usleep(100);
	px4_arch_gpiowrite(GPIO_GPS_NRESET, 1);
	PX4_WARN("Toggled GPS reset pin");
#endif
}

int
GPS::print_status()
{
	switch (_instance) {
	case Instance::Main:
		PX4_INFO("Main GPS");
		break;

	case Instance::Secondary:
		PX4_INFO("");
		PX4_INFO("Secondary GPS");
		break;

	default:
		break;
	}

	//GPS Mode
	if (_fake_gps) {
		PX4_INFO("protocol: SIMULATED");

	} else {
		switch (_mode) {
		case GPS_DRIVER_MODE_UBX:
			PX4_INFO("protocol: UBX");
			break;

		case GPS_DRIVER_MODE_MTK:
			PX4_INFO("protocol: MTK");
			break;

		case GPS_DRIVER_MODE_ASHTECH:
			PX4_INFO("protocol: ASHTECH");
			break;

		default:
			break;
		}
	}

	PX4_INFO("port: %s, baudrate: %d, status: %s", _port, _baudrate, _healthy ? "OK" : "NOT OK");
	PX4_INFO("sat info: %s", (_p_report_sat_info != nullptr) ? "enabled" : "disabled");

	if (_report_gps_pos.timestamp != 0) {
		print_message(_report_gps_pos);

		if (_helper) {
			PX4_INFO("rate position: \t\t%6.2f Hz", (double)_helper->getPositionUpdateRate());
			PX4_INFO("rate velocity: \t\t%6.2f Hz", (double)_helper->getVelocityUpdateRate());
		}

		if (!_fake_gps) {
			PX4_INFO("rate publication:\t\t%6.2f Hz", (double)_rate);
			PX4_INFO("rate RTCM injection:\t%6.2f Hz", (double)_rate_rtcm_injection);
		}

	}

	if (_instance == Instance::Main && _secondary_instance) {
		GPS *secondary_instance = (GPS *)_secondary_instance;
		secondary_instance->print_status();
	}

	return 0;
}

void
GPS::publish()
{
	if (_instance == Instance::Main || _is_gps_main_advertised) {
		orb_publish_auto(ORB_ID(vehicle_gps_position), &_report_gps_pos_pub, &_report_gps_pos, &_gps_orb_instance,
				 ORB_PRIO_DEFAULT);
		_is_gps_main_advertised = true;
	}
}

void
GPS::publishSatelliteInfo()
{
	if (_instance == Instance::Main) {
		orb_publish_auto(ORB_ID(satellite_info), &_report_sat_info_pub, _p_report_sat_info, &_gps_sat_orb_instance,
				 ORB_PRIO_DEFAULT);

	} else {
		//we don't publish satellite info for the secondary gps
	}
}

int GPS::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GPS::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
GPS driver module that handles the communication with the device and publishes the position via uORB.
It supports multiple protocols (device vendors) and by default automatically selects the correct one.

The module supports a secondary GPS device, specified via `-e` parameter. The position will be published
on the second uORB topic instance, but it's currently not used by the rest of the system (however the
data will be logged, so that it can be used for comparisons).

### Implementation
There is a thread for each device polling for data. The GPS protocol classes are implemented with callbacks
so that they can be used in other projects as well (eg. QGroundControl uses them too).

### Examples
For testing it can be useful to fake a GPS signal (it will signal the system that it has a valid position):
$ gps stop
$ gps start -f
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gps", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "GPS device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('e', nullptr, "<file:dev>", "Optional secondary GPS device", true);

	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Fake a GPS signal (useful for testing)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('s', "Enable publication of satellite info", true);

	PRINT_MODULE_USAGE_PARAM_STRING('i', "uart", "spi|uart", "GPS interface", true);
	PRINT_MODULE_USAGE_PARAM_STRING('p', nullptr, "ubx|mtk|ash", "GPS Protocol (default=auto select)", true);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int GPS::task_spawn(int argc, char *argv[])
{
	return task_spawn(argc, argv, Instance::Main);
}

int GPS::task_spawn(int argc, char *argv[], Instance instance)
{
	px4_main_t entry_point;
	if (instance == Instance::Main) {
		entry_point = (px4_main_t)&run_trampoline;
	} else {
		entry_point = (px4_main_t)&run_trampoline_secondary;
	}

	int task_id = px4_task_spawn_cmd("gps", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1610,
				   entry_point, (char *const *)argv);

	if (task_id < 0) {
		task_id = -1;
		return -errno;
	}

	if (instance == Instance::Main) {
		_task_id = task_id;
	}

	return 0;
}

int GPS::run_trampoline_secondary(int argc, char *argv[])
{

#ifdef __PX4_NUTTX
	// on NuttX task_create() adds the task name as first argument
	argc -= 1;
	argv += 1;
#endif

	GPS *gps = instantiate(argc, argv, Instance::Secondary);
	if (gps) {
		_secondary_instance = gps;
		gps->run();

		_secondary_instance = nullptr;
		delete gps;
	}
	return 0;
}
GPS *GPS::instantiate(int argc, char *argv[])
{
	return instantiate(argc, argv, Instance::Main);
}

GPS *GPS::instantiate(int argc, char *argv[], Instance instance)
{
	const char *device_name = GPS_DEFAULT_UART_PORT;
	const char *device_name_secondary = nullptr;
	bool fake_gps = false;
	bool enable_sat_info = false;
	GPSHelper::Interface interface = GPSHelper::Interface::UART;
	gps_driver_mode_t mode = GPS_DRIVER_MODE_NONE;

	bool error_flag = false;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:e:fsi:p:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		case 'e':
			device_name_secondary = myoptarg;
			break;

		case 'f':
			fake_gps = true;
			break;

		case 's':
			enable_sat_info = true;
			break;

		case 'i':
			if (!strcmp(myoptarg, "spi")) {
				interface = GPSHelper::Interface::SPI;

			} else if (!strcmp(myoptarg, "uart")) {
				interface = GPSHelper::Interface::UART;

			} else {
				PX4_ERR("unknown interface: %s", myoptarg);
				error_flag = true;
			}
			break;

		case 'p':
			if (!strcmp(myoptarg, "ubx")) {
				mode = GPS_DRIVER_MODE_UBX;

			} else if (!strcmp(myoptarg, "mtk")) {
				mode = GPS_DRIVER_MODE_MTK;

			} else if (!strcmp(myoptarg, "ash")) {
				mode = GPS_DRIVER_MODE_ASHTECH;

			} else {
				PX4_ERR("unknown interface: %s", myoptarg);
				error_flag = true;
			}
			break;

		case '?':
			error_flag = true;
			break;

		default:
			PX4_WARN("unrecognized flag");
			error_flag = true;
			break;
		}
	}

	if (error_flag) {
		return nullptr;
	}

	GPS *gps;
	if (instance == Instance::Main) {
		gps = new GPS(device_name, mode, interface, fake_gps, enable_sat_info, instance);

		if (gps && device_name_secondary) { // main이 정상적으로 생성되어야 secondary도 task_spawn되도록...
			task_spawn(argc, argv, Instance::Secondary);
			// wait until running
			int i = 0;

			do { // secondary를 위해서 1초간 wait
				/* wait up to 1s */
				usleep(2500);

			} while (!_secondary_instance && ++i < 400);

			if (i == 400) { // 1초 전에 끝나야 하는데 1초가 지난 경우
				PX4_ERR("Timed out while waiting for thread to start");
			}
		}
	} else { // secondary instance
		gps = new GPS(device_name_secondary, mode, interface, fake_gps, enable_sat_info, instance);
	}

	return gps;
}

int
gps_main(int argc, char *argv[])
{
	return GPS::main(argc, argv);
}
