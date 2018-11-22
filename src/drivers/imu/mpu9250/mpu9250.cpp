/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file mpu9250.cpp
 *
 * Driver for the Invensense MPU9250 connected via I2C or SPI.
 *
 * @author Andrew Tridgell
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>
#include <ecl/geo/geo.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>
#include <systemlib/px4_macros.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "gyro.h"
#include "mpu9250.h"

/*
  timer interrupt를 설정할때 원하는 sample rate보다 약간더 빠르게 동작하도록 설정하고 accel 값 비교에 따른 중복을 제거.
  이렇게 해서 줄인 시간은 다른 timer로 인해 생길 수 있는 timing jitter에 의한 지연을 대처하는게 충분하다.
 */
#define MPU9250_TIMER_REDUCTION				200

// accel 범위 설정
#define ACCEL_RANGE_G  16
/*
  check_registers()에서 검사하는 register의 목록.
  MPUREG_PRODUCT_ID는 목록의 맨 앞에 둬야함.
 */
const uint8_t MPU9250::_checked_registers[MPU9250_NUM_CHECKED_REGISTERS] = { MPUREG_WHOAMI, // product ID
									     MPUREG_PWR_MGMT_1, // power management
									     MPUREG_PWR_MGMT_2,
									     MPUREG_USER_CTRL,
									     MPUREG_SMPLRT_DIV,
									     MPUREG_CONFIG,
									     MPUREG_GYRO_CONFIG, //gyro config
									     MPUREG_ACCEL_CONFIG, //accel config
									     MPUREG_ACCEL_CONFIG2,
									     MPUREG_INT_ENABLE, // interrupt enable
									     MPUREG_INT_PIN_CFG // interrupt pin config
									   };


MPU9250::MPU9250(device::Device *interface, device::Device *mag_interface, const char *path_accel,
		 const char *path_gyro, const char *path_mag,
		 enum Rotation rotation) :
	CDev("MPU9250", path_accel),
	_interface(interface),
	_gyro(new MPU9250_gyro(this, path_gyro)),
	_mag(new MPU9250_mag(this, mag_interface, path_mag)),
	_whoami(0),
#if defined(USE_I2C)
	_work {},
	_use_hrt(false),
#else
	_use_hrt(true),
#endif
	_call {},
	_call_interval(0),
	_accel_reports(nullptr),
	_accel_scale{},
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_accel_topic(nullptr),
	_accel_orb_class_instance(-1),
	_accel_class_instance(-1),
	_gyro_reports(nullptr),
	_gyro_scale{},
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_dlpf_freq(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_sample_rate(1000),
	_accel_reads(perf_alloc(PC_COUNT, "mpu9250_acc_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "mpu9250_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu9250_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu9250_bad_trans")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu9250_bad_reg")),
	_good_transfers(perf_alloc(PC_COUNT, "mpu9250_good_trans")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu9250_reset")),
	_duplicates(perf_alloc(PC_COUNT, "mpu9250_dupe")),
	_register_wait(0),
	_reset_wait(0),
	_accel_filter_x(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_int(1000000 / MPU9250_ACCEL_MAX_OUTPUT_RATE),
	_gyro_int(1000000 / MPU9250_GYRO_MAX_OUTPUT_RATE, true),
	_rotation(rotation),
	_checked_next(0),
	_last_temperature(0),
	_last_accel_data{},
	_got_duplicate(false)
{
	_debug_enabled = false;

	// 장치의 parameter를 설정하고 bus 장치의 parameter 사용
	/* Set device parameters and make sure parameters of the bus device are adopted */
	_device_id.devid_s.devtype = DRV_ACC_DEVTYPE_MPU9250;
	_device_id.devid_s.bus_type = (device::Device::DeviceBusType)_interface->get_device_bus_type();
	_device_id.devid_s.bus = _interface->get_device_bus();
	_device_id.devid_s.address = _interface->get_device_address();

	// 부모 device id로 _gyro 장치의 device id로 우선 설정
	/* Set device parameters and make sure parameters of the bus device are adopted */
	_gyro->_device_id.devid = _device_id.devid;
	_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_MPU9250;
	_gyro->_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_gyro->_device_id.devid_s.bus = _interface->get_device_bus();
	_gyro->_device_id.devid_s.address = _interface->get_device_address();

	// 부모 device id로 _mag 장치의 device id로 우선 설정
	_mag->_device_id.devid = _device_id.devid;
	_mag->_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_MPU9250;
	_mag->_device_id.devid_s.bus_type = _interface->get_device_bus_type();
	_mag->_device_id.devid_s.bus = _interface->get_device_bus();
	_mag->_device_id.devid_s.address = _interface->get_device_address();

	// 별도의 mag인 경우에 i2c 버스에 연결
	_interface->set_device_type(_device_id.devid_s.devtype);

	// 기본 accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// 기본 gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
#if defined(USE_I2C)
	memset(&_work, 0, sizeof(_work));
#endif
}

MPU9250::~MPU9250()
{
	//비활성화
	stop();

	orb_unadvertise(_accel_topic);
	orb_unadvertise(_gyro->_gyro_topic);

	/* gyro의 subdriver를 삭제 */
	delete _gyro;

	/* magnetometer subdriver 삭제 */
	delete _mag;

	/* 기본 report를 해제 */
	if (_accel_reports != nullptr) {
		delete _accel_reports;
	}

	if (_gyro_reports != nullptr) {
		delete _gyro_reports;
	}

	if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}

	/* 성능 counter 삭제 */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

int
MPU9250::init()
{

#if defined(USE_I2C)
	unsigned dummy;
	use_i2c(_interface->ioctl(MPUIOCGIS_I2C, dummy));
#endif

	/*
	 * MPU가 I2C르 사용하는 경우 sample rate를 200Hz까지 줄여야 함.
	*/
	if (is_i2c()) {
		_sample_rate = 200;
		_accel_int.set_autoreset_interval(1000000 / 1000);
		_gyro_int.set_autoreset_interval(1000000 / 1000);
	}

	int ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 probe failed");
		return ret;
	}

	// 초기화
	ret = CDev::init();

	// 초기화 실패하는 경우
	if (ret != OK) {
		DEVICE_DEBUG("CDev init failed");
		return ret;
	}

	// accel report 버퍼 할당
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));
	ret = -ENOMEM;

	if (_accel_reports == nullptr) {
		return ret;
	}

	// gyro report 버퍼 할당
	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == nullptr) {
		return ret;
	}

	// mpu 리셋
	if (reset_mpu() != OK) {
		PX4_ERR("Exiting! Device failed to take initialization");
		return ret;
	}

	// offset와 scale 초기화
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	// 제어기를 위해 sw low pass filter 설정. param에서 cutoff 참조
	param_t accel_cut_ph = param_find("IMU_ACCEL_CUTOFF");
	float accel_cut = MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ;

	if (accel_cut_ph != PARAM_INVALID && (param_get(accel_cut_ph, &accel_cut) == PX4_OK)) {
		PX4_INFO("accel cutoff set to %.2f Hz", double(accel_cut));

		_accel_filter_x.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, accel_cut);
		_accel_filter_y.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, accel_cut);
		_accel_filter_z.set_cutoff_frequency(MPU9250_ACCEL_DEFAULT_RATE, accel_cut);

	} else {
		PX4_ERR("IMU_ACCEL_CUTOFF param invalid");
	}

	param_t gyro_cut_ph = param_find("IMU_GYRO_CUTOFF");
	float gyro_cut = MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ;

	if (gyro_cut_ph != PARAM_INVALID && (param_get(gyro_cut_ph, &gyro_cut) == PX4_OK)) {
		PX4_INFO("gyro cutoff set to %.2f Hz", double(gyro_cut));

		_gyro_filter_x.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, gyro_cut);
		_gyro_filter_y.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, gyro_cut);
		_gyro_filter_z.set_cutoff_frequency(MPU9250_GYRO_DEFAULT_RATE, gyro_cut);

	} else {
		PX4_ERR("IMU_GYRO_CUTOFF param invalid");
	}

	// gyro device node 초기화
	ret = _gyro->init();

	// 초기화 실패하는 경우
	if (ret != OK) {
		DEVICE_DEBUG("gyro init failed");
		return ret;
	}

#ifdef USE_I2C

	if (!_mag->is_passthrough() && _mag->_interface->init() != PX4_OK) {
		PX4_ERR("failed to setup ak8963 interface");
	}

#endif /* USE_I2C */

	// 9250이며 mag 초기화
	if (_whoami == MPU_WHOAMI_9250) {
		ret = _mag->init();
	}

	// 실패하는 경우
	if (ret != OK) {
		DEVICE_DEBUG("mag init failed");
		return ret;
	}


	if (_whoami == MPU_WHOAMI_9250) {
		ret = _mag->ak8963_reset();
	}

	if (ret != OK) {
		DEVICE_DEBUG("mag reset failed");
		return ret;
	}

	// "/dev/accelx" 
	_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	// accel topic을 publish하기 위해서 accel report를 초기화
	struct accel_report arp;
	_accel_reports->get(&arp);

	// report를 생성해서 publish
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_accel_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
		return ret;
	}

	// gyro topic을 publish하기 위해서 accel report를 초기화
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	_gyro->_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro->_gyro_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_gyro->_gyro_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
		return ret;
	}

	return ret;
}

// startup time : 최대 100 ms. 
int MPU9250::reset()
{
	irqstate_t state;

	// mpu9250가 0V에서 시작하는 경우 시작하는데 필요한 시간 최대 100ms

	usleep(110000);

	// 준비될때까지(100ms) sampling을 연기
	state = px4_enter_critical_section();
	_reset_wait = hrt_absolute_time() + 100000;
	px4_leave_critical_section(state);

	int ret;

	ret = reset_mpu();

	if (ret == OK && _whoami == MPU_WHOAMI_9250) {
		ret = _mag->ak8963_reset();
	}


	state = px4_enter_critical_section();
	_reset_wait = hrt_absolute_time() + 10;
	px4_leave_critical_section(state);

	return ret;
}

int MPU9250::reset_mpu()
{
	write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_AUTO);
	write_checked_reg(MPUREG_PWR_MGMT_2, 0);


	usleep(1000);

	// Enable I2C bus or Disable I2C bus (recommended on data sheet)

	write_checked_reg(MPUREG_USER_CTRL, is_i2c() ? 0 : BIT_I2C_IF_DIS);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);

	_set_dlpf_filter(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ);

	// Gyro scale 2000 deg/s ()
	write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	set_accel_range(ACCEL_RANGE_G);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready

#ifdef USE_I2C
	bool bypass = !_mag->is_passthrough();
#else
	bool bypass = false;
#endif


	write_checked_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR | (bypass ? BIT_INT_BYPASS_EN : 0));

	write_checked_reg(MPUREG_ACCEL_CONFIG2, BITS_ACCEL_CONFIG2_41HZ);

	uint8_t retries = 3;
	bool all_ok = false;

	while (!all_ok && retries--) {
		all_ok = true;
		uint8_t reg;

		for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
			if ((reg = read_reg(_checked_registers[i])) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				PX4_ERR("Reg %d is:%d s/b:%d Tries:%d", _checked_registers[i], reg, _checked_values[i], retries);
				all_ok = false;
			}
		}
	}

	return all_ok ? OK : -EIO;
}

int
MPU9250::probe()
{
	// device ID 찾기
	_whoami = read_reg(MPUREG_WHOAMI);

	// product revision 검증
	switch (_whoami) {
	case MPU_WHOAMI_9250:
	case MPU_WHOAMI_6500:
		memset(_checked_values, 0, sizeof(_checked_values));
		memset(_checked_bad, 0, sizeof(_checked_bad));
		_checked_values[0] = _whoami;
		_checked_bad[0] = _whoami;
		return OK;
	}

	DEVICE_DEBUG("unexpected whoami 0x%02x", _whoami);
	return -EIO;
}

/*
  smaple rate를 설정 (1kHz ~ 5Hz). accel과 gyro에 대해서
*/
void
MPU9250::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	if (desired_sample_rate_hz == 0 ||
	    desired_sample_rate_hz == GYRO_SAMPLERATE_DEFAULT ||
	    desired_sample_rate_hz == ACCEL_SAMPLERATE_DEFAULT) {
		desired_sample_rate_hz = MPU9250_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
	_sample_rate = 1000 / div;
}

/*
  Low Pass Filter의 freq 설정. accel과 gyro에 대해서
 */
void
MPU9250::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter;

	// 다음으로 가장 높은 filter freq를 선택
	if (frequency_hz == 0) {
		_dlpf_freq = 0;
		filter = BITS_DLPF_CFG_3600HZ;

	} else if (frequency_hz <= 5) {
		_dlpf_freq = 5;
		filter = BITS_DLPF_CFG_5HZ;

	} else if (frequency_hz <= 10) {
		_dlpf_freq = 10;
		filter = BITS_DLPF_CFG_10HZ;

	} else if (frequency_hz <= 20) {
		_dlpf_freq = 20;
		filter = BITS_DLPF_CFG_20HZ;

	} else if (frequency_hz <= 41) {
		_dlpf_freq = 41;
		filter = BITS_DLPF_CFG_41HZ;

	} else if (frequency_hz <= 92) {
		_dlpf_freq = 92;
		filter = BITS_DLPF_CFG_92HZ;

	} else if (frequency_hz <= 184) {
		_dlpf_freq = 184;
		filter = BITS_DLPF_CFG_184HZ;

	} else if (frequency_hz <= 250) {
		_dlpf_freq = 250;
		filter = BITS_DLPF_CFG_250HZ;

	} else {
		_dlpf_freq = 0;
		filter = BITS_DLPF_CFG_3600HZ;
	}

	write_checked_reg(MPUREG_CONFIG, filter);
}

ssize_t
MPU9250::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	// buffer는 반드시 충분히 커야 
	if (count < 1) {
		return -ENOSPC;
	}

	// 자동 측정이 활성화되어 있지 않다면 새로운 측정값을 buffer로 넣어야
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	// 만약 data가 없는 경우, error 상황
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	// buffer에서 읽어서 report로 복사.
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	// 전달할 최대 byte 수를 반환
	return (transferred * sizeof(accel_report));
}

int
MPU9250::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
MPU9250::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	return 0;
}

int
MPU9250::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	/*
	 * 스펙 문서 위치 : 
	 * http://www.invensense.com/mems/gyro/documents/PS-MPU-9250A-00v3.4.pdf
	 */
	const float max_offset = 0.34f;
	
	const float max_scale = 0.3f;

	// gyro offsets
	if (fabsf(_gyro_scale.x_offset) > max_offset) {
		return 1;
	}

	// gyro scale
	if (fabsf(_gyro_scale.x_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > max_scale) {
		return 1;
	}

	return 0;
}

/*
  의도적으로 복구 동작이 시작되도록 센서에서 error를 구동시킴.
 */
void
MPU9250::test_error()
{
	// deliberately trigger an error. This was noticed during
	// development as a handy way to test the reset logic
	uint8_t data[16];
	memset(data, 0, sizeof(data));
	_interface->read(MPU9250_SET_SPEED(MPUREG_INT_STATUS, MPU9250_LOW_BUS_SPEED), data, sizeof(data));
	::printf("error triggered\n");
	print_registers();
}

ssize_t
MPU9250::gyro_read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(gyro_report);

	// 버퍼는 충분히 크게 
	if (count < 1) {
		return -ENOSPC;
	}

	// 자동 측정이 활성화되어 있는 경우, 새로운 측정값을 buf로 넣기
	if (_call_interval == 0) {
		_gyro_reports->flush();
		measure();
	}

	// data가 없으면 error
	if (_gyro_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_gyro_reads);

	// buffer를 report로 복사해서 caller에 전달
	gyro_report *grp = reinterpret_cast<gyro_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_gyro_reports->get(grp)) {
			break;
		}

		transferred++;
		grp++;
	}

	// 전송할 byte의 수 반환
	return (transferred * sizeof(gyro_report));
}

int
MPU9250::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET: { // 센서 reset
			return reset();
		}

	case SENSORIOCSPOLLRATE: { // 센서 poll rate 설정
			switch (arg) {

			// 수동 polling으로 전환
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			// 외부에서 신호 수신 지원 안함
			case SENSOR_POLLRATE_EXTERNAL:

			// 0이면 error 상황
			case 0:
				return -EINVAL;

			// 기본/최대 polling rate 설정
			case SENSOR_POLLRATE_MAX: // 최대 polling rate
				return ioctl(filp, SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT: // 기본 polling rate // MPU9250_ACCEL_DEFAULT_RATE도 1000. 1000 Hz로 설정
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU9250_ACCEL_DEFAULT_RATE);

			// polling interval을 조정 Hz 단위 
			default: {
					// 처음 시작하는 경우 내부 polling을 시작해야하는지 want_start를 true로 설정
					bool want_start = (_call_interval == 0);

					// 1000 us
					// hz를 us 단위의 hrt interval로 변환
					unsigned ticks = 1000000 / arg;

					// 최대 rate 대비 적정한 rate인지 검사
					if (ticks < 1000) { // 1ms보다 작은 경우 그냥 return
						return -EINVAL;
					}

					// accel, gyro에 대힌 low pass filter 초기화
					// filter 조정
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					// 다음 measurement에 대해서 interval을 조정. 
					_call_interval = ticks;

					/*
					  sample time보다 interval을 더 빠르게 호출하도록 설정. (시간 지연에 대비해서 딱맞게 )
					  중복 sample을 탐지하면 reject. stm32와 mpu9250 clock 사이에 beat때문에 aliasing을 방지. (800 us)
					 */ // 실제 system 동작 interval : _call.period, 기대하는 interval : _call_interval
					_call.period = _call_interval - MPU9250_TIMER_REDUCTION;

					// poll state machine을 시작해야한다면 start 호출
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE: // poll rate 
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: { // 
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_accel_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case ACCELIOCGSAMPLERATE: // accel sample rate 속도
		return _sample_rate;

	case ACCELIOCSSAMPLERATE: // accel sample rate 속도 설정
		_set_sample_rate(arg);
		return OK;

	case ACCELIOCSSCALE: { // accel scale set 
			struct accel_calibration_s *s = (struct accel_calibration_s *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE: // accel scale get
		memcpy((struct accel_calibration_s *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE: // accel range set
		return set_accel_range(arg);

	case ACCELIOCGRANGE: // accel range get 
		return (unsigned long)((_accel_range_m_s2) / CONSTANTS_ONE_G + 0.5f);

	case ACCELIOCSELFTEST: // accel self-test set
		return accel_self_test();

	default:
		/* 상위 class로 전달 */
		return CDev::ioctl(filp, cmd, arg);
	}
}

int
MPU9250::gyro_ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	/* these are shared with the accel side */
	case SENSORIOCSPOLLRATE:
	case SENSORIOCGPOLLRATE:
	case SENSORIOCRESET:
		return ioctl(filp, cmd, arg);

	case SENSORIOCSQUEUEDEPTH: {
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_gyro_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case GYROIOCGSAMPLERATE:
		return _sample_rate;

	case GYROIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case GYROIOCSSCALE:
		/* copy scale in */
		memcpy(&_gyro_scale, (struct gyro_calibration_s *) arg, sizeof(_gyro_scale));
		return OK;

	case GYROIOCGSCALE:
		/* copy scale out */
		memcpy((struct gyro_calibration_s *) arg, &_gyro_scale, sizeof(_gyro_scale));
		return OK;

	case GYROIOCSRANGE:
		return -EINVAL;

	case GYROIOCGRANGE:
		return (unsigned long)(_gyro_range_rad_s * 180.0f / M_PI_F + 0.5f);

	case GYROIOCSELFTEST:
		return gyro_self_test();

	default:
		/* 부모 class로 전달 */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uint8_t
MPU9250::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t buf;
	_interface->read(MPU9250_SET_SPEED(reg, speed), &buf, 1);
	return buf;
}

uint16_t
MPU9250::read_reg16(unsigned reg)
{
	uint8_t buf[2];

	// 낮은 clock 속도에서 general register transfer
	_interface->read(MPU9250_LOW_SPEED_OP(reg), &buf, arraySize(buf));
	return (uint16_t)(buf[0] << 8) | buf[1];
}

void
MPU9250::write_reg(unsigned reg, uint8_t value)
{
	// 낮은 clock 속도에서 general register transfer
	_interface->write(MPU9250_LOW_SPEED_OP(reg), &value, 1);
}

void
MPU9250::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU9250::modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_checked_reg(reg, val);
}

void
MPU9250::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
			break;
		}
	}
}

int
MPU9250::set_accel_range(unsigned max_g_in)
{
	uint8_t afs_sel;
	float lsb_per_g;
	float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		max_accel_g = 2;
	}

	write_checked_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
	_accel_range_scale = (CONSTANTS_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * CONSTANTS_ONE_G;

	return OK;
}

void
MPU9250::start()
{
	// 이미 시작하고 있는 경우 stop 시키기
	stop();

	// 버퍼에 남아 있던 data를 제거하기
	_accel_reports->flush();
	_gyro_reports->flush();
	_mag->_mag_reports->flush();

	if (_use_hrt) { //hrt 사용하는 경우
		// 지정한 rate로 polling을 시작
		hrt_call_every(&_call,
			       1000,
			       _call_interval - MPU9250_TIMER_REDUCTION, //800 us
			       (hrt_callout)&MPU9250::measure_trampoline, this);

	} else {
#ifdef USE_I2C
		/* schedule a cycle to start things */
		work_queue(HPWORK, &_work, (worker_t)&MPU9250::cycle_trampoline, this, 1);
#endif
	}

}

void
MPU9250::stop()
{
	if (_use_hrt) { // hrt로 동작하는 경우 hrt_cancle로 stop 시키기
		hrt_cancel(&_call);

	} else {
#ifdef USE_I2C
		work_cancel(HPWORK, &_work);
#endif
	}
}


#if defined(USE_I2C)
void
MPU9250::cycle_trampoline(void *arg)
{
	MPU9250 *dev = (MPU9250 *)arg;

	dev->cycle();
}

void
MPU9250::cycle()
{

	measure();

	if (_call_interval != 0) {
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&MPU9250::cycle_trampoline,
			   this,
			   USEC2TICK(_call_interval - MPU9250_TIMER_REDUCTION));
	}
}
#endif


void
MPU9250::measure_trampoline(void *arg)
{
	MPU9250 *dev = reinterpret_cast<MPU9250 *>(arg);

	/* make another measurement */
	dev->measure();
}

// 최대 속도로 register 읽기.(high speed register로 등록되어 있지 않더라도)
// 일부 register에 대해서 저속으로 읽기가 가능한 경우 센서 설정을 변경하는데 여러 register를 읽기 위해서 시간 지연이 누적. 단일 register를 읽을때는 이런 지연시간이 누적되는 경우는 없음.
// data register를 읽는 속도와 동일한 속도로 읽기는 것도 SPI bus 상태를 테스트하는 좋은 방법
void
MPU9250::check_registers(void)
{
	uint8_t v;

	if ((v = read_reg(_checked_registers[_checked_next], MPU9250_HIGH_BUS_SPEED)) !=
	    _checked_values[_checked_next]) { // 검사 같과 같아야 정상인데 다른 경우 문제가 있는 경우
		_checked_bad[_checked_next] = v; // bad 값을 저장하는 곳에 넣고

		/*
		  이상한 값을 가져오는 경우 SPI bus나 센서가 문제가 있다는 증거.
		  _register_wait을 20으로 설정하고 센서가 다시 OK상태로 되기 체크 
		 */
		perf_count(_bad_registers);

		/*
		  bad register 값을 고치도록 시도하기. 문제있는 센서가 bus를 망치는 않도록 loop마다 수정을 시도.
		  고장난 센서가 bus를 차지하는 것을 막기 위해서 loop마다 한 번 fix 작업 수행.
		 */
		if (_register_wait == 0 || _checked_next == 0) { // _register_wait은 20번 loop 돈 경우 혹은 처음부터 체크하는 경우 
			// product_id가 잘못된 값이면 센서를 완전히 reset시킨다.
			write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET); // 센서 완전 reset
			write_reg(MPUREG_PWR_MGMT_2, MPU_CLK_SEL_AUTO);
			// reset 후에 잠시 대기한다. 이후에 register 쓰기 혹은 이상한 상태에 있는 모든 register를 제대로 보정하고 mpu9250을 종료시킴. 
			_reset_wait = hrt_absolute_time() + 10000;
			_checked_next = 0;

		} else {
			// 정상값을 register에 쓰기
			write_reg(_checked_registers[_checked_next], _checked_values[_checked_next]);
			// 센서가 복구되는 기회를 주기 위해서 register에 쓰기 전에 3ms 정도 대기
			_reset_wait = hrt_absolute_time() + 3000;
		}

		_register_wait = 20;
	}

	_checked_next = (_checked_next + 1) % MPU9250_NUM_CHECKED_REGISTERS;
}

// null data 검사
bool MPU9250::check_null_data(uint32_t *data, uint8_t size)
{
	while (size--) {
		if (*data++) {
			perf_count(_good_transfers);
			return false;
		}
	}
	// 모두 zero 데이터인 경우 SPI 버스 error일 가능성 있음.
	perf_count(_bad_transfers);
	perf_end(_sample_perf);
	// 여기서 reset()을 호출하지 않음. reset()은 interrupt를 비활성화 시키는데 20ms 정도 기간이 걸림.
	// 이런 경우 mpu9250에 문제가 있으면 FMU failure 상태가 됨.(20ms 동안 imu 데이터가 들어가지 않으므로 다른 센서가 정상이더라도 failure 상태)
	return true;
}

// 중복 검사
bool MPU9250::check_duplicate(uint8_t *accel_data)
{
	/*
	   accel data 중복 여부 검사.
	   status register에 있는 data ready interrupt status bit을 사용할 수 없음(왜냐하면 새로운 gyro data가 들어와도 high로 되니까)
	   BITS_DLPF_CFG_256HZ_NOLPF2을 설정한 경우 gyro를 8kHz로 샘플링을 하므로 실제로는 중복 accel data가 들어왔는데 새로운 data가 들어왔다고 착각할 수 있음.
	*/
	// 2개가 같은 경우 memcmp()은 0을 반환
	if (!_got_duplicate && memcmp(accel_data, &_last_accel_data, sizeof(_last_accel_data)) == 0) {
		// 이 경우 새로운 data가 아니므로 다음 timer를 기다림
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;

	} else { // 이전 값과 다른 경우 복사
		memcpy(&_last_accel_data, accel_data, sizeof(_last_accel_data));
		_got_duplicate = false;
	}

	return _got_duplicate;
}

//gyro, accel, mag 읽어서 
void
MPU9250::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		return;
	}

	struct MPUReport mpu_report;

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
	} report;

	/* 측정 시작 */
	perf_begin(_sample_perf);

	/*
	 * MPU9250 SPI 인터페이스로 측정값 전체 set을 가져오기
	 */
	if (OK != _interface->read(MPU9250_SET_SPEED(MPUREG_INT_STATUS, MPU9250_HIGH_BUS_SPEED),
				   (uint8_t *)&mpu_report,
				   sizeof(mpu_report))) {
		perf_end(_sample_perf);
		return;
	}

	// register를 검사 제대로 읽히는지에 따라서 spi bus나 센서의 상태를 체크 가능
	check_registers();

	// 중복 검사
	if (check_duplicate(&mpu_report.accel_x[0])) {
		return;
	}

#ifdef USE_I2C

	if (_mag->is_passthrough()) {
#endif
		_mag->_measure(mpu_report.mag);
#ifdef USE_I2C

	} else {
		_mag->measure();
	}

#endif

	/*
	 * big endian에서 littel endian을 변환하기
	 */
	report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);
	report.temp    = int16_t_from_bytes(mpu_report.temp);
	report.gyro_x  = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y  = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z  = int16_t_from_bytes(mpu_report.gyro_z);

	// null data 여부 검사
	if (check_null_data((uint32_t *)&report, sizeof(report) / 4)) {
		return;
	}

	// 다시 sensor를 사용하기 전에 정상적인 전송을 위한 기다림. _good_transfers을 증가시키지만 아직 data를 반환하지 않음.
	if (_register_wait != 0) {
		_register_wait--;
		return;
	}

	/*
	 * 축을 바꾸기 (기체 body frame 기준 x, y, z (roll, pitch, yaw) 맞춤). mag와 같은 방향으로 맞추기
	 * http://www.plclive.com/uploads/allimg/160323/110R2G42-0.png 참고
	 * x, y축 바꾸고 y방향 -로 설정
	 */
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * swap을 적용
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;

	/*
	 * buf를 report하기
	 * Report buffers.
	 */
	accel_report		arb;
	gyro_report		grb;

	/*
	 * 결과를 m/s^2 하기 위해 조정 및 scale
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();

	// error count = bad transfer의 횟수 + bad register가 읽은 합.
	// 상위 코드에서 sensor가 문제가 있는지를 결정하는데 사용.
	grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) raw 값을 스케일링해서 SI 단위로 만들기
	 * 2) 고정 offset 값을 빼기 (SI 단위)
	 * 3) 선형 동적 factor로 칼리브레이션한 값을 scale
	 * Note: 고정 sensor offset은 'zero' 입력에서 sensor 출력되는 수. 따라서 offset을 빼줘야 함.
	 *   예제: gyro 출력은 0 각속도에서 74의 값을 출력. 따라서 offset은 74이므로 측정한 값에서 74를 빼준다.
	 */

	// NOTE: 축은 보드와 매치시키기 위해서 바꿔치기한다. 

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// 사용자 지정 rotation 적용
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	matrix::Vector3f aval(x_in_new, y_in_new, z_in_new);
	matrix::Vector3f aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	_last_temperature = (report.temp) / 361.0f + 35.0f; // 도씨

	arb.temperature_raw = report.temp;
	arb.temperature = _last_temperature;

	// device ID를 반환
	arb.device_id = _device_id.devid;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// 사용자 지정 roation을 적용
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	matrix::Vector3f gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	matrix::Vector3f gval_integrated;

	bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = _last_temperature;

	// device ID를 반환
	grb.device_id = _gyro->_device_id.devid;

	_accel_reports->force(&arb); // arb를 ringbuffer에 넣기
	_gyro_reports->force(&grb); // grb를 ringbuffer에 넣기

	// data를 기다리는 쪽에다가 notify
	if (accel_notify) {
		poll_notify(POLLIN);
	}

	if (gyro_notify) {
		_gyro->parent_poll_notify();
	}

	if (accel_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro->_gyro_topic, &grb);
	}

	/* stop measuring */
	perf_end(_sample_perf);
}

void
MPU9250::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_accel_reads);
	perf_print_counter(_gyro_reads);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_bad_registers);
	perf_print_counter(_good_transfers);
	perf_print_counter(_reset_retries);
	perf_print_counter(_duplicates);
	_accel_reports->print_info("accel queue");
	_gyro_reports->print_info("gyro queue");
	_mag->_mag_reports->print_info("mag queue");
	::printf("checked_next: %u\n", _checked_next);

	for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
		uint8_t v = read_reg(_checked_registers[i], MPU9250_HIGH_BUS_SPEED);

		if (v != _checked_values[i]) {
			::printf("reg %02x:%02x should be %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_values[i]);
		}

		if (v != _checked_bad[i]) {
			::printf("reg %02x:%02x was bad %02x\n",
				 (unsigned)_checked_registers[i],
				 (unsigned)v,
				 (unsigned)_checked_bad[i]);
		}
	}

	::printf("temperature: %.1f\n", (double)_last_temperature);
	float accel_cut = _accel_filter_x.get_cutoff_freq();
	::printf("accel cutoff set to %10.2f Hz\n", double(accel_cut));
	float gyro_cut = _gyro_filter_x.get_cutoff_freq();
	::printf("gyro cutoff set to %10.2f Hz\n", double(gyro_cut));
}

void
MPU9250::print_registers()
{
	printf("MPU9250 registers\n");

	for (uint8_t reg = 0; reg <= 126; reg++) {
		uint8_t v = read_reg(reg);
		printf("%02x:%02x ", (unsigned)reg, (unsigned)v);

		if (reg % 13 == 0) {
			printf("\n");
		}
	}

	printf("\n");
}
