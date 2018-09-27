/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file spi.cpp
 *
 * Base class for devices connected via SPI.
 *
 * @todo Work out if caching the mode/frequency would save any time.
 *
 * @todo A separate bus/device abstraction would allow for mixed interrupt-mode
 * and non-interrupt-mode clients to arbitrate for the bus.  As things stand,
 * a bus shared between clients of both kinds is vulnerable to races between
 * the two, where an interrupt-mode client will ignore the lock held by the
 * non-interrupt-mode client.
 */

#include "SPI.hpp"

#include <px4_config.h>
#include <nuttx/arch.h>

#ifndef CONFIG_SPI_EXCHANGE
# error This driver requires CONFIG_SPI_EXCHANGE
#endif

namespace device
{

SPI::SPI(const char *name,
	 const char *devname,
	 int bus,
	 uint32_t device,
	 enum spi_mode_e mode, //clock mode
	 uint32_t frequency) :
	// base class
	CDev(name, devname),
	// public
	// protected
	locking_mode(LOCK_PREEMPTION),
	// private
	_device(device),
	_mode(mode),
	_frequency(frequency),
	_dev(nullptr)
{
	// SPI 장치 관련해서 device id 관련 필드 초기화
	// fill in _device_id fields for a SPI device
	_device_id.devid_s.bus_type = DeviceBusType_SPI;
	_device_id.devid_s.bus = bus;
	_device_id.devid_s.address = (uint8_t)device;
	// devicetype은 driver에서 채운다.
	// devtype needs to be filled in by the driver
	_device_id.devid_s.devtype = 0;
}

SPI::~SPI()
{
	// XXX no way to let go of the bus...
}

int
SPI::init()
{
	int ret = OK;

	// spi bus에 attach를 위한 초기화
	/* attach to the spi bus */
	if (_dev == nullptr) {
		_dev = px4_spibus_initialize(get_device_bus());
	}

	if (_dev == nullptr) {
		DEVICE_DEBUG("failed to init SPI");
		ret = -ENOENT;
		goto out;
	}

	// pin selection의 high -> low 전환이 가능하도록 device를 풀어 놓기. unselect 상태로 초기화
	/* deselect device to ensure high to low transition of pin select */
	SPI_SELECT(_dev, _device, false);

	// probe()는 해당 device가 존재하는지 여부를 검사
	/* call the probe function to check whether the device is present */
	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("probe failed");
		goto out;
	}

	// device node 생성
	/* do base class init, which will create the device node, etc. */
	ret = CDev::init();

	if (ret != OK) {
		DEVICE_DEBUG("cdev init failed");
		goto out;
	}

	/* tell the workd where we are */
	DEVICE_LOG("on SPI bus %d at %d (%u KHz)", get_device_bus(), PX4_SPI_DEV_ID(_device), _frequency / 1000);

out:
	return ret;
}

// 전송의 실제 처리는 _transfer()에서 처리. 전송하기전에 lock 처리 설정.
int
SPI::transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = up_interrupt_context() ? LOCK_NONE : locking_mode;

	// 필요에 따라서 bus를 lock
	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: { // 시스템 차원에서 lock
			irqstate_t state = px4_enter_critical_section();
			result = _transfer(send, recv, len);
			px4_leave_critical_section(state);
		}
		break;

	case LOCK_THREADS: // spi에서 lock
		SPI_LOCK(_dev, true);
		result = _transfer(send, recv, len);
		SPI_LOCK(_dev, false);
		break;

	case LOCK_NONE: // lock 없이 전송
		result = _transfer(send, recv, len);
		break;
	}

	return result;
}

// 실제 전송 처리 부분. 
int
SPI::_transfer(uint8_t *send, uint8_t *recv, unsigned len)
{
	SPI_SETFREQUENCY(_dev, _frequency); //freq 설정
	SPI_SETMODE(_dev, _mode);  // mode 설정
	SPI_SETBITS(_dev, 8);  
	SPI_SELECT(_dev, _device, true); // 전송을 위한 select true

	/* do the transfer */ // //send, recv 전송처리
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */ // 전송을 마치고 false로 설정
	SPI_SELECT(_dev, _device, false);

	return OK;
}

// _transferhword 처리. 즉 half-word(16bit) 전용 전송
int
SPI::transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	int result;

	if ((send == nullptr) && (recv == nullptr)) {
		return -EINVAL;
	}

	LockMode mode = up_interrupt_context() ? LOCK_NONE : locking_mode;

	/* lock the bus as required */
	switch (mode) {
	default:
	case LOCK_PREEMPTION: {
			irqstate_t state = px4_enter_critical_section();
			result = _transferhword(send, recv, len);
			px4_leave_critical_section(state);
		}
		break;

	case LOCK_THREADS:
		SPI_LOCK(_dev, true);
		result = _transferhword(send, recv, len);
		SPI_LOCK(_dev, false);
		break;

	case LOCK_NONE:
		result = _transferhword(send, recv, len);
		break;
	}

	return result;
}

int
SPI::_transferhword(uint16_t *send, uint16_t *recv, unsigned len)
{
	SPI_SETFREQUENCY(_dev, _frequency);
	SPI_SETMODE(_dev, _mode);
	SPI_SETBITS(_dev, 16);							/* 16 bit transfer */
	SPI_SELECT(_dev, _device, true);

	/* do the transfer */
	SPI_EXCHANGE(_dev, send, recv, len);

	/* and clean up */
	SPI_SELECT(_dev, _device, false);

	return OK;
}

} // namespace device
