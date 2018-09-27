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
 * @file mpu9250_spi.cpp
 *
 * Driver for the Invensense MPU9250 connected via SPI.
 *
 * @author Andrew Tridgell
 * @author Pat Hickey
 * @author David sidrane
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/spi.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_device.h>

#include "mpu9250.h"
#include <board_config.h>

#define DIR_READ			0x80
#define DIR_WRITE			0x00

/*
 * The MPU9250 can only handle high SPI bus speeds of 20Mhz on the sensor and
 * interrupt status registers. All other registers have a maximum 1MHz
 * SPI speed
 *
 * 실제 동작은 10.5 Mhz, 11.250Mhz 정도로 동작 (HIGH Speed인 경우)
 * The Actual Value will be rounded down by the spi driver.
 * for a 168Mhz CPU this will be 10.5 Mhz and for a 180 Mhz CPU
 * it will be 11.250 Mhz
 */
#define MPU9250_LOW_SPI_BUS_SPEED	1000*1000    //  1MHz
#define MPU9250_HIGH_SPI_BUS_SPEED	20*1000*1000 // 20Mhz 


device::Device *MPU9250_SPI_interface(int bus, uint32_t cs, bool external_bus);


class MPU9250_SPI : public device::SPI
{
public:
	MPU9250_SPI(int bus, uint32_t device);
	virtual ~MPU9250_SPI() = default;

	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);
protected:
	virtual int probe();

private:

	/* Helper to set the desired speed and isolate the register on return */

	void set_bus_frequency(unsigned &reg_speed_reg_out);
};

device::Device *
MPU9250_SPI_interface(int bus, uint32_t cs, bool external_bus)
{
	device::Device *interface = nullptr;

	if (external_bus) {
#if !(defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT_MPU))
		errx(0, "External SPI not available");
#endif
	}

	if (cs != SPIDEV_NONE(0)) { // device id가 none이 아닌 경우 
		interface = new MPU9250_SPI(bus, cs);
	}

	return interface;
}

MPU9250_SPI::MPU9250_SPI(int bus, uint32_t device) :
	SPI("MPU9250", nullptr, bus, device, SPIDEV_MODE3, MPU9250_LOW_SPI_BUS_SPEED)
{
	_device_id.devid_s.devtype =  DRV_ACC_DEVTYPE_MPU9250;
}

int
MPU9250_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case ACCELIOCGEXTERNAL: //외부 센서에서 accel get
		external();

	/* FALLTHROUGH */

	case DEVIOCGDEVICEID: // base class의 ioctl호출.  device id get
		return CDev::ioctl(nullptr, operation, arg);

	case MPUIOCGIS_I2C:
		return 0;

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

//freq 설정
void
MPU9250_SPI::set_bus_frequency(unsigned &reg_speed)
{
	/* Set the desired speed */

	set_frequency(MPU9250_IS_HIGH_SPEED(reg_speed) ? MPU9250_HIGH_SPI_BUS_SPEED : MPU9250_LOW_SPI_BUS_SPEED);

	/* Isoolate the register on return */

	reg_speed = MPU9250_REG(reg_speed);
}


int
MPU9250_SPI::write(unsigned reg_speed, void *data, unsigned count)
{
	uint8_t cmd[MPU_MAX_WRITE_BUFFER_SIZE]; //write buffer. size = 2

	// 버퍼보다 큰 경우 error
	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	// freq 설정 및 
	/* Set the desired speed and isolate the register */

	set_bus_frequency(reg_speed);

	cmd[0] = reg_speed | DIR_WRITE; // 첫번째 buffer에는 WRITE 명령
	cmd[1] = *(uint8_t *)data;     // 두번째 buffer에 data

	return transfer(&cmd[0], &cmd[0], count + 1);
}

// MPUReport의 data를 읽을지 cmd를 읽을지 buf를 결정. data를 읽기에 충분한 버퍼를 제공하도록.
int
MPU9250_SPI::read(unsigned reg_speed, void *data, unsigned count)
{
	/* We want to avoid copying the data of MPUReport: So if the caller
	 * supplies a buffer not MPUReport in size, it is assume to be a reg or reg 16 read
	 * and we need to provied the buffer large enough for the callers data
	 * and our command.
	 */
	uint8_t cmd[3] = {0, 0, 0};

	// 읽을 내용은 cmd인지 data 인지를 결정(MPUReport보다 작으면 cmd라고 판단)
	uint8_t *pbuff  =  count < sizeof(MPUReport) ? cmd : (uint8_t *) data ;


	if (count < sizeof(MPUReport))  {
		// 명령 추가
		/* add command */

		count++;
	}

	// bus freq를 설정
	set_bus_frequency(reg_speed);

	// 명령 설정
	/* Set command */

	pbuff[0] = reg_speed | DIR_READ ; // 첫번째 버퍼에 READ 명령 설정

	// 명령을 전달하면 data를 얻는다.
	/* Transfer the command and get the data */

	int ret = transfer(pbuff, pbuff, count);

	if (ret == OK && pbuff == &cmd[0]) { //pbuff의 주소가 바뀌지 않은 경우 cmd를 그대로 사용

		// count를 조정
		/* Adjust the count back */

		count--;

		// data 반환
		/* Return the data */

		memcpy(data, &cmd[1], count); //

	}

	return ret;
}

// 장치가 연결되어 있는지 여부 확인
int
MPU9250_SPI::probe()
{
	uint8_t whoami = 0;

	int ret = read(MPUREG_WHOAMI, &whoami, 1); // 9250 attach 여부 확인

	if (ret != OK) {
		return -EIO;
	}

	switch (whoami) {
	case MPU_WHOAMI_9250:  // 9250이 있는 경우 0을 리턴
	case MPU_WHOAMI_6500:
		ret = 0;
		break;

	default:
		PX4_WARN("probe failed! %u", whoami);
		ret = -EIO;
	}

	return ret;
}
