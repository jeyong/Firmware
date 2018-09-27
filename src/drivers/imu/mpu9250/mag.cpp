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
 * @file mag.cpp
 *
 * Driver for the ak8963 magnetometer within the Invensense mpu9250
 *
 * @author Robert Dickenson
 *
 */

#include <px4_config.h>
#include <px4_log.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>

#include <perf/perf_counter.h>

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
#include "mpu9250.h"

// interface가 null이 아닌 경우 해당 장치와 interface를 이용해서 통신
// interface가 null인 경우 reg값을 통해서 통신
// If interface is non-null, then it will used for interacting with the device.
// Otherwise, it will passthrough the parent MPU9250
MPU9250_mag::MPU9250_mag(MPU9250 *parent, device::Device *interface, const char *path) :
	CDev("MPU9250_mag", path),
	_interface(interface),
	_parent(parent),
	_mag_topic(nullptr),
	_mag_orb_class_instance(-1),
	_mag_class_instance(-1),
	_mag_reading_data(false),
	_mag_reports(nullptr),
	_mag_scale{},
	_mag_range_scale(),
	_mag_reads(perf_alloc(PC_COUNT, "mpu9250_mag_reads")),
	_mag_errors(perf_alloc(PC_COUNT, "mpu9250_mag_errors")),
	_mag_overruns(perf_alloc(PC_COUNT, "mpu9250_mag_overruns")),
	_mag_overflows(perf_alloc(PC_COUNT, "mpu9250_mag_overflows")),
	_mag_duplicates(perf_alloc(PC_COUNT, "mpu9250_mag_duplicates")),
	_mag_asa_x(1.0), //각 축에 대한 sensitivity adjustment value
	_mag_asa_y(1.0),
	_mag_asa_z(1.0),
	_last_mag_data{}
{
	// default mag scale factors
	_mag_scale.x_offset = 0;
	_mag_scale.x_scale  = 1.0f;
	_mag_scale.y_offset = 0;
	_mag_scale.y_scale  = 1.0f;
	_mag_scale.z_offset = 0;
	_mag_scale.z_scale  = 1.0f;

	_mag_range_scale = MPU9250_MAG_RANGE_GA;
}

MPU9250_mag::~MPU9250_mag()
{
	if (_mag_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _mag_class_instance);
	}

	if (_mag_reports != nullptr) {
		delete _mag_reports;
	}

	orb_unadvertise(_mag_topic);

	perf_free(_mag_reads);
	perf_free(_mag_errors);
	perf_free(_mag_overruns);
	perf_free(_mag_overflows);
	perf_free(_mag_duplicates);
}

int
MPU9250_mag::init()
{
	int ret = CDev::init();

	/* if cdev init failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("MPU9250 mag init failed");
		return ret;
	}

	// 2개 담을 수 있는 ringbuffer
	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == nullptr) {
		return -ENOMEM;;
	}

	_mag_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH); //"/dev/mag"


	// sensor_mag id로 publish 준비
	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrp;
	_mag_reports->get(&mrp);

	_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
					 &_mag_orb_class_instance, ORB_PRIO_LOW);
	//			   &_mag_orb_class_instance, (is_external()) ? ORB_PRIO_MAX - 1 : ORB_PRIO_HIGH - 1);

	if (_mag_topic == nullptr) {
		PX4_ERR("ADVERT FAIL");
		return -ENOMEM;
	}

	return OK;
}

// data 중복 검사
bool MPU9250_mag::check_duplicate(uint8_t *mag_data)
{
	// 이전 data와 현재 data가 같으면 true를 반환(다음 timer까지 대기)
	if (memcmp(mag_data, &_last_mag_data, sizeof(_last_mag_data)) == 0) {
		// it isn't new data - wait for next timer
		return true;

	} else { // 중복이 아님. 현재 data를 이전 data에 복사
		memcpy(&_last_mag_data, mag_data, sizeof(_last_mag_data));
		return false;
	}
}

void
MPU9250_mag::measure()
{
	struct ak8963_regs data;

	if (OK == _interface->read(AK8963REG_ST1, &data, sizeof(struct ak8963_regs))) {
		_measure(data);
	}
}

void
MPU9250_mag::_measure(struct ak8963_regs data)
{
	bool mag_notify = true;

	if (check_duplicate((uint8_t *)&data.x) && !(data.st1 & 0x02)) {
		perf_count(_mag_duplicates);
		return;
	}

	//overrun flag가 설정되어 있는지 감시
	/* monitor for if data overrun flag is ever set */
	if (data.st1 & 0x02) {
		perf_count(_mag_overruns);
	}

	// mag 센서 overflow flag 감시
	/* monitor for if magnetic sensor overflow flag is ever set noting that st2
	 * is usually not even refreshed, but will always be in the same place in the
	 * mpu's buffers regardless, hence the actual count would be bogus
	 */
	if (data.st2 & 0x08) {
		perf_count(_mag_overflows);
	}

	mag_report	mrb;
	mrb.timestamp = hrt_absolute_time();
	mrb.is_external = false;

	/* http://www.plclive.com/uploads/allimg/160323/110R2G42-0.png 그림 참고
	 * Align axes - note the accel & gryo are also re-aligned so this
	 *              doesn't look obvious with the datasheet
	 */
	mrb.x_raw =  data.x;
	mrb.y_raw = -data.y; //
	mrb.z_raw = -data.z; //

	float xraw_f =  data.x;
	float yraw_f = -data.y;
	float zraw_f = -data.z;

	// rotate 적용
	/* apply user specified rotation */
	rotate_3f(_parent->_rotation, xraw_f, yraw_f, zraw_f);

	mrb.x = ((xraw_f * _mag_range_scale * _mag_asa_x) - _mag_scale.x_offset) * _mag_scale.x_scale;
	mrb.y = ((yraw_f * _mag_range_scale * _mag_asa_y) - _mag_scale.y_offset) * _mag_scale.y_scale;
	mrb.z = ((zraw_f * _mag_range_scale * _mag_asa_z) - _mag_scale.z_offset) * _mag_scale.z_scale;
	mrb.range_ga = 48.0f;
	mrb.scaling = _mag_range_scale;
	mrb.temperature = _parent->_last_temperature;
	mrb.device_id = _parent->_mag->_device_id.devid;

	mrb.error_count = perf_event_count(_mag_errors);

	_mag_reports->force(&mrb); // ringbuffer에 넣기 

	/* notify anyone waiting for data */
	if (mag_notify) { // mag data를 polling하고 있는 모듈에게 noti
		poll_notify(POLLIN);
	}

	// sensor_mag를 publish
	if (mag_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _mag_topic, &mrb);
	}
}

// 읽어서 buffer에 넣기
ssize_t
MPU9250_mag::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report); // data 갯수

	// count가 최소한 1보다 크거나 같아야함. 
	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	// 자동 측정이 활성화되어 있지 않은 경우, _parent->measure()로 측정 값을 buffer로 가져온다.
	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_parent->_call_interval == 0) {
		_mag_reports->flush();
		// 최소한 1번 이상 cycle이 돌아야 정상적인 mag 값을 얻을 수 있다.
		/* TODO: this won't work as getting valid magnetometer
		 *       data requires more than one measure cycle
		 */
		_parent->measure();
	}

	// data가 없으면 error 반환
	/* if no data, error (we could block here) */
	if (_mag_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_mag_reads);

	// buffer에 담기 위한 준비
	/* copy reports out of our buffer to the caller */
	mag_report *mrp = reinterpret_cast<mag_report *>(buffer); // mrp = buffer
	int transferred = 0;

	while (count--) { // count 갯수만큼 얻기
		if (!_mag_reports->get(mrp)) { //mrp에 넣기 (buffer에 담기)
			break;
		}

		transferred++;
		mrp++;
	}
	// buffer에 담겨진 크기 반환
	/* return the number of bytes transferred */
	return (transferred * sizeof(mag_report));
}

int
MPU9250_mag::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET: // reset
		return ak8963_reset();

	case SENSORIOCSPOLLRATE: { // poll rate 속도 설정
			switch (arg) {

			/* switching to manual polling */ // 메뉴얼 polling으로 전환
			case SENSOR_POLLRATE_MANUAL:
				/* 아직 구현 없음
				 * TODO: investigate being able to stop
				 *       the continuous sampling
				 */
				//stop();
				return OK;

			/* external signalling not supported */ // 지원 안함
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */ // 100Hz로 설정
			case SENSOR_POLLRATE_MAX:
				return ioctl(filp, SENSORIOCSPOLLRATE, 100);

			case SENSOR_POLLRATE_DEFAULT: // 100Hz로 설정
				return ioctl(filp, SENSORIOCSPOLLRATE, MPU9250_AK8963_SAMPLE_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					if (MPU9250_AK8963_SAMPLE_RATE != arg) {
						return -EINVAL;
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE: // poll rate get (100Hz 반환)
		return MPU9250_AK8963_SAMPLE_RATE;

	case SENSORIOCSQUEUEDEPTH: { // internal queue depth. 저장공간 크기. 1 < size < 100 사이로 지정.
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_mag_reports->resize(arg)) { // queue의 크기를 조정
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case MAGIOCGSAMPLERATE: // sample rate get. 100Hz
		return MPU9250_AK8963_SAMPLE_RATE;

	case MAGIOCSSAMPLERATE:

		/* 현재 지원하지 않음.
		 * We don't currently support any means of changing
		 * the sampling rate of the mag
		 */
		if (MPU9250_AK8963_SAMPLE_RATE != arg) {
			return -EINVAL;
		}

		return OK;

	case MAGIOCSSCALE: // scale 설정
		/* copy scale in */
		memcpy(&_mag_scale, (struct mag_scale *) arg, sizeof(_mag_scale));
		return OK;

	case MAGIOCGSCALE: // scale 얻기
		/* copy scale out */
		memcpy((struct mag_scale *) arg, &_mag_scale, sizeof(_mag_scale));
		return OK;

	case MAGIOCSRANGE:
		return -EINVAL;

	case MAGIOCGRANGE: // 48 Gauss 값으로 고정
		return 48; // fixed full scale measurement range of +/- 4800 uT == 48 Gauss

	case MAGIOCSELFTEST: // self-test
		return self_test();

#ifdef MAGIOCSHWLOWPASS

	case MAGIOCSHWLOWPASS:
		return -EINVAL;
#endif

#ifdef MAGIOCGHWLOWPASS

	case MAGIOCGHWLOWPASS:
		return -EINVAL;
#endif

	default:
		return (int)CDev::ioctl(filp, cmd, arg);
	}
}

int
MPU9250_mag::self_test(void)
{
	return 0;
}

// pass-through 설정. 
void
MPU9250_mag::set_passthrough(uint8_t reg, uint8_t size, uint8_t *out)
{
	uint8_t addr;

	// register 변경 전에 slave의 read/write 를 비활성화 시키기.
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // ensure slave r/w is disabled before changing the registers

	if (out) { // out 데이터를 쓰기
		_parent->write_reg(MPUREG_I2C_SLV0_D0, *out);
		addr = AK8963_I2C_ADDR;

	} else { // 읽기
		addr = AK8963_I2C_ADDR | BIT_I2C_READ_FLAG;
	}

	_parent->write_reg(MPUREG_I2C_SLV0_ADDR, addr);
	_parent->write_reg(MPUREG_I2C_SLV0_REG,  reg);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, size | BIT_I2C_SLV0_EN);
}

//지정한 size만큼 읽어서 buf에 넣기
void
MPU9250_mag::read_block(uint8_t reg, uint8_t *val, uint8_t count)
{
	_parent->_interface->read(reg, val, count);
}

// 지정한 size만큼 읽어서 buf에 넣기
// i2c 통신에서 FMU가 master가 되고 imu9250은 slave로 동작하는 mode.
// Pass-Through Mode: The MPU-9250 directly connects the primary and auxiliary I2C buses together, allowing the system processor to directly communicate with any external sensors.
void
MPU9250_mag::passthrough_read(uint8_t reg, uint8_t *buf, uint8_t size)
{
	set_passthrough(reg, size);
	usleep(25 + 25 * size); // wait for the value to be read from slave //slave로부터 읽기 모드로 대기
	read_block(MPUREG_EXT_SENS_DATA_00, buf, size);
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // disable new reads // 읽고나면 새로 read 동작 안하도록
}

// 1byte 읽어서 반환
uint8_t
MPU9250_mag::read_reg(unsigned int reg)
{
	uint8_t buf;

	if (_interface == nullptr) {
		passthrough_read(reg, &buf, 0x01);

	} else {
		_interface->read(reg, &buf, 1);
	}

	return buf;
}


// ak8963 device id 검사
bool
MPU9250_mag::ak8963_check_id(uint8_t &deviceid)
{
	deviceid = read_reg(AK8963REG_WIA);

	return (AK8963_DEVICE_ID == deviceid);
}

/*
 * pass-through mode로 1byte 쓰기 동작
 * 400kHz I2C bus speed = 2.5us per bit = 25us per byte
 */
void
MPU9250_mag::passthrough_write(uint8_t reg, uint8_t val)
{
	set_passthrough(reg, 1, &val);
	usleep(50); // wait for the value to be written to slave
	_parent->write_reg(MPUREG_I2C_SLV0_CTRL, 0); // disable new writes
}


// 낮은 clock speed인 경우 register의 값을 전송
void
MPU9250_mag::write_reg(unsigned reg, uint8_t value)
{
	// general register transfer at low clock speed
	if (_interface == nullptr) {
		passthrough_write(reg, value);

	} else {
		_interface->write(MPU9250_LOW_SPEED_OP(reg), &value, 1);
	}
}



// ak8963 reset
int
MPU9250_mag::ak8963_reset(void)
{
	// First initialize it to use the bus

	int rv = ak8963_setup();

	if (rv == OK) {

		// Now reset the mag
		write_reg(AK8963REG_CNTL2, AK8963_RESET);
		// Then re-initialize the bus/mag
		rv = ak8963_setup();
	}

	return rv;

}

// ak8963 asa값 읽기
bool
MPU9250_mag::ak8963_read_adjustments(void)
{
	uint8_t response[3];
	float ak8963_ASA[3];

	write_reg(AK8963REG_CNTL1, AK8963_FUZE_MODE | AK8963_16BIT_ADC);
	usleep(50);

	if (_interface != nullptr) {
		_interface->read(AK8963REG_ASAX, response, 3);

	} else {
		passthrough_read(AK8963REG_ASAX, response, 3);
	}

	write_reg(AK8963REG_CNTL1, AK8963_POWERDOWN_MODE);

	// 계산식 적용
	for (int i = 0; i < 3; i++) {
		if (0 != response[i] && 0xff != response[i]) {
			ak8963_ASA[i] = ((float)(response[i] - 128) / 256.0f) + 1.0f;

		} else {
			return false;
		}
	}

	_mag_asa_x = ak8963_ASA[0];
	_mag_asa_y = ak8963_ASA[1];
	_mag_asa_z = ak8963_ASA[2];

	return true;
}

// ak8963이 master i2c로 동작하도록 reg 설정
int
MPU9250_mag::ak8963_setup_master_i2c(void)
{
	//mag의 interface가 null인 경우, SPI를 사용하고 있는 중이며 MPU9250(_parent)를 이용하여 reg로 읽고 쓰기 통신
	/* When _interface is null we are using SPI and must
	 * use the parent interface to configure the device to act
	 * in master mode (SPI to I2C bridge)
	 */
	if (_interface == nullptr) {
		_parent->modify_checked_reg(MPUREG_USER_CTRL, 0, BIT_I2C_MST_EN);
		_parent->write_reg(MPUREG_I2C_MST_CTRL, BIT_I2C_MST_P_NSR | BIT_I2C_MST_WAIT_FOR_ES | BITS_I2C_MST_CLOCK_400HZ);

	} else {
		_parent->modify_checked_reg(MPUREG_USER_CTRL, BIT_I2C_MST_EN, 0);
	}

	return OK;
}
// ak8963 setup
int
MPU9250_mag::ak8963_setup(void)
{
	int retries = 10;

	do {

		ak8963_setup_master_i2c(); //master i2c로 서렂ㅇ
		write_reg(AK8963REG_CNTL2, AK8963_RESET); //reset

		uint8_t id = 0;

		if (ak8963_check_id(id)) { //device id 검사하여 같으면 
			break;
		}

		retries--;
		PX4_ERR("AK8963: bad id %d retries %d", id, retries);
		_parent->modify_reg(MPUREG_USER_CTRL, 0, BIT_I2C_MST_RST);
		up_udelay(100);
	} while (retries > 0);

	if (retries > 0) {
		retries = 10;

		while (!ak8963_read_adjustments() && retries) { //asa 읽기. 10번까지 재시도
			retries--;
			PX4_ERR("AK8963: failed to read adjustment data. Retries %d", retries);

			_parent->modify_reg(MPUREG_USER_CTRL, 0, BIT_I2C_MST_RST);
			up_udelay(100);
			ak8963_setup_master_i2c();
			write_reg(AK8963REG_CNTL2, AK8963_RESET);
		}
	}

	if (retries == 0) {
		PX4_ERR("AK8963: failed to initialize, disabled!");
		_parent->modify_checked_reg(MPUREG_USER_CTRL, BIT_I2C_MST_EN, 0);
		_parent->write_reg(MPUREG_I2C_MST_CTRL, 0);
		return -EIO;
	}

	write_reg(AK8963REG_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);

	// interface가 null인 경우 mpu의 i2c master interface를 설정하여 ak8963 data를 읽도록
	if (_interface == NULL) {
		/* Configure mpu' I2c Master interface to read ak8963 data
		 * Into to fifo
		 */
		set_passthrough(AK8963REG_ST1, sizeof(struct ak8963_regs));
	}

	return OK;
}
