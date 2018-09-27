/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
// https://www.geeksforgeeks.org/friend-class-function-cpp/

#include <stdint.h>

#include <perf/perf_counter.h>
#include <systemlib/conversions.h>

#include <nuttx/wqueue.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>

#include "mag.h"
#include "gyro.h"



#if defined(PX4_I2C_OBDEV_MPU9250)
#  define USE_I2C
#endif


// MPU 9250 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BITS_ACCEL_CONFIG2_41HZ		0x03

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10
#define BIT_INT_BYPASS_EN		0x02

#define BIT_I2C_READ_FLAG           0x80

#define BIT_I2C_SLV0_NACK           0x01
#define BIT_I2C_FIFO_EN             0x40
#define BIT_I2C_MST_EN              0x20
#define BIT_I2C_IF_DIS              0x10
#define BIT_FIFO_RST                0x04
#define BIT_I2C_MST_RST             0x02
#define BIT_SIG_COND_RST            0x01

#define BIT_I2C_SLV0_EN             0x80
#define BIT_I2C_SLV0_BYTE_SW        0x40
#define BIT_I2C_SLV0_REG_DIS        0x20
#define BIT_I2C_SLV0_REG_GRP        0x10

#define BIT_I2C_MST_MULT_MST_EN     0x80
#define BIT_I2C_MST_WAIT_FOR_ES     0x40
#define BIT_I2C_MST_SLV_3_FIFO_EN   0x20
#define BIT_I2C_MST_P_NSR           0x10
#define BITS_I2C_MST_CLOCK_258HZ    0x08
#define BITS_I2C_MST_CLOCK_400HZ    0x0D

#define BIT_I2C_SLV0_DLY_EN         0x01
#define BIT_I2C_SLV1_DLY_EN         0x02
#define BIT_I2C_SLV2_DLY_EN         0x04
#define BIT_I2C_SLV3_DLY_EN         0x08

#define MPU_WHOAMI_9250			0x71
#define MPU_WHOAMI_6500			0x70

#define MPU9250_ACCEL_DEFAULT_RATE	1000
#define MPU9250_ACCEL_MAX_OUTPUT_RATE			280
#define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define MPU9250_GYRO_DEFAULT_RATE	1000
/* rates need to be the same between accel and gyro */
#define MPU9250_GYRO_MAX_OUTPUT_RATE			MPU9250_ACCEL_MAX_OUTPUT_RATE
#define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define MPU9250_DEFAULT_ONCHIP_FILTER_FREQ	92

#define MPUIOCGIS_I2C	(unsigned)(DEVIOCGDEVICEID+100)


#pragma pack(push, 1)
/**
 * Report conversation within the mpu, including command byte and
 * interrupt status.
 */
struct MPUReport {
	uint8_t		cmd;
	uint8_t		status;
	uint8_t		accel_x[2];
	uint8_t		accel_y[2];
	uint8_t		accel_z[2];
	uint8_t		temp[2];
	uint8_t		gyro_x[2];
	uint8_t		gyro_y[2];
	uint8_t		gyro_z[2];
	struct ak8963_regs mag;
};
#pragma pack(pop)

#define MPU_MAX_WRITE_BUFFER_SIZE (2)


/*
  MPU9250은 고속 버스와 interrupt status 레지스터를 처리할 수 있음.
  다른 모든 register는 최대 1MHz로 통신. I2C는 400kHz, SPI는 1MHz.
  더 빠른 통신이 필요한 경우, 센서와 interrupt register는 SPI로 20MHz로 읽기가 가능.
  The MPU9250 can only handle high bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  Communication with all registers of the device is performed using either
  I2C at 400kHz or SPI at 1MHz. For applications requiring faster communications,
  the sensor and interrupt registers may be read using SPI at 20MHz
 */
#define MPU9250_LOW_BUS_SPEED				0
#define MPU9250_HIGH_BUS_SPEED				0x8000
#  define MPU9250_IS_HIGH_SPEED(r) 			((r) & MPU9250_HIGH_BUS_SPEED)
#  define MPU9250_REG(r) 					((r) &~MPU9250_HIGH_BUS_SPEED)
#  define MPU9250_SET_SPEED(r, s) 			((r)|(s))
#  define MPU9250_HIGH_SPEED_OP(r) 			MPU9250_SET_SPEED((r), MPU9250_HIGH_BUS_SPEED)
#  define MPU9250_LOW_SPEED_OP(r)			MPU9250_REG((r))

/* interface factories */
extern device::Device *MPU9250_SPI_interface(int bus, uint32_t cs, bool external_bus);
extern device::Device *MPU9250_I2C_interface(int bus, uint32_t address, bool external_bus);
extern int MPU9250_probe(device::Device *dev, int device_type);

typedef device::Device *(*MPU9250_constructor)(int, uint32_t, bool);

class MPU9250_mag;
class MPU9250_gyro;

class MPU9250 : public device::CDev
{
public:
	MPU9250(device::Device *interface, device::Device *mag_interface, const char *path_accel, const char *path_gyro,
		const char *path_mag,
		enum Rotation rotation);
	virtual ~MPU9250();

	virtual int		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int		ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void			print_info();

	void			print_registers();

	// 테스트 목적으로 센서 에러 발생
	// deliberately cause a sensor error
	void 			test_error();

protected:
	Device			*_interface;

	virtual int		probe();

	friend class MPU9250_mag; // MPU9250_mag class는 MPU9250 class의 private에 접근 가능
	friend class MPU9250_gyro;

	virtual ssize_t		gyro_read(struct file *filp, char *buffer, size_t buflen);
	virtual int		gyro_ioctl(struct file *filp, int cmd, unsigned long arg);

private:
	MPU9250_gyro	*_gyro;
	MPU9250_mag     *_mag;
	uint8_t			_whoami;	/** whoami result */

#if defined(USE_I2C)
	/*
	 * SPI bus based device use hrt
	 * I2C bus needs to use work queue
	 */
	work_s			_work;
#endif
	// SPI의 경우 hrt 사용
	bool 			_use_hrt;

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_accel_reports; //accel ring 버퍼

	struct accel_calibration_s	_accel_scale; // accel scale
	float			_accel_range_scale;  // range scale 
	float			_accel_range_m_s2;	// m/s^2
	orb_advert_t		_accel_topic;	// accel topic
	int			_accel_orb_class_instance;	// orb class instance
	int			_accel_class_instance;	// class instance

	ringbuffer::RingBuffer	*_gyro_reports; // gyro 버퍼

	struct gyro_calibration_s	_gyro_scale; //gyro scale
	float			_gyro_range_scale;	// range scale
	float			_gyro_range_rad_s;	// rad/s

	unsigned		_dlpf_freq;	// 

	unsigned		_sample_rate;	//sample_rate
	perf_counter_t		_accel_reads;	// accel read 횟수
	perf_counter_t		_gyro_reads;	// gyro read 횟수
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;	// bad transfer 횟수
	perf_counter_t		_bad_registers;	// bad register 횟수
	perf_counter_t		_good_transfers;	// good transfer 횟수
	perf_counter_t		_reset_retries;	// reset 횟수
	perf_counter_t		_duplicates;	// 중복 횟수

	uint8_t			_register_wait; 	// register wait
	uint64_t		_reset_wait;	//reset wait

	math::LowPassFilter2p	_accel_filter_x; //accel X축 필터
	math::LowPassFilter2p	_accel_filter_y; //accel Y축 필터
	math::LowPassFilter2p	_accel_filter_z; //accel Z축 필터
	math::LowPassFilter2p	_gyro_filter_x; //gyro X축 필터
	math::LowPassFilter2p	_gyro_filter_y; //gyro Y축 필터
	math::LowPassFilter2p	_gyro_filter_z; //gyro Z축 필터

	Integrator		_accel_int;	// accel 통합
	Integrator		_gyro_int;	// gyro 통합

	enum Rotation		_rotation;

	// 실시간으로 주요 설정 register를 검사 지원. SPI 버스 에러를 탐지하고 센서를 reset하기 위함.
	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MPU9250_NUM_CHECKED_REGISTERS 11
	static const uint8_t	_checked_registers[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_bad[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// last temperature reading for print_info()
	float			_last_temperature;

	// 데이터가 null인지 중복인지 체크
	bool check_null_data(uint32_t *data, uint8_t size);
	bool check_duplicate(uint8_t *accel_data);
	// keep last accel reading for duplicate detection
	uint8_t			_last_accel_data[6];
	bool			_got_duplicate;

	/**
	 * 측정 시작
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * 측정 중지
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * chip과 측정 범위를 reset하며 이때 scale과 offset은 변경하지 않음.
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();


	/**
	 * main chip을 reset (mag를 해당없음)
	 * Resets the main chip (excluding the magnetometer if any).
	 */
	int			reset_mpu();


#if defined(USE_I2C)
	/**
	 * When the I2C interfase is on
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
		 * previous measurement.
		 *
		 * When the interval between measurements is greater than the minimum
		 * measurement interval, a gap is inserted between collection
		 * and measurement to provide the most recent measurement possible
		 * at the next interval.
		 */
	void			cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		cycle_trampoline(void *arg);

	void use_i2c(bool on_true) { _use_hrt = !on_true; }

#endif

	bool is_i2c(void) { return !_use_hrt; }




	/**
	 * hrt wrapper를 아직 만들지 않아서 static으로 선언.
	 * 자동으로 polling하도록 설정한 경우 지정한 rate로 interrupt 형태로 HRT에서 호출.
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * 센서로부터 측정 값을 가져와서 버퍼를 업데이트한다.
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	/**
	 * mpu로부터 register를 읽는다.
	 * Read a register from the mpu
	 *
	 * @param		The register to read.
	 * @return		The value that was read.
	 */
	uint8_t			read_reg(unsigned reg, uint32_t speed = MPU9250_LOW_BUS_SPEED);
	uint16_t		read_reg16(unsigned reg);

	/**
	 * mpu의 register에 쓰기
	 * Write a register in the mpu
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * mpu에 있는 register 수정. register bits를 설정하기 전에 clear하기.
	 * Modify a register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * mpu에서 register에 쓰고 _checked_values를 업데이트하기
	 * Write a register in the mpu, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/**
	 * mpu에 있는 checked register를 수정
	 * Modify a checked register in the mpu
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_checked_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * mpu 측정 범위를 설정
	 * Set the mpu measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	/**
	 * mpu에서 읽은 16-bit 값을 native byte order로 변경
	 * Swap a 16-bit value read from the mpu to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	bool			is_external()
	{
		unsigned dummy;
		return _interface->ioctl(ACCELIOCGEXTERNAL, dummy);
	}

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			gyro_self_test();

	/*
	  로우패스필터의 freq를 설정
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  sample rate를 설정 (1KHz ~ 5Hz)
	  set sample rate (approximate) - 1kHz to 5Hz
	*/
	void _set_sample_rate(unsigned desired_sample_rate_hz);

	/*
	  주요 register들이 여전히 동일한 값을 가지고 있는지 검사
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/* do not allow to copy this class due to pointer data members */
	MPU9250(const MPU9250 &);
	MPU9250 operator=(const MPU9250 &);
};
