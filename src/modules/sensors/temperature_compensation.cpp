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

/**
 * @file temperature_compensation.cpp
 *
 * Sensors temperature compensation methods
 *
 * @author Paul Riseborough <gncsolns@gmail.com>
 */

#include "temperature_compensation.h"
#include <parameters/param.h>
#include <px4_defines.h>
#include <px4_log.h>

namespace sensors
{
// parameter_handles 초기화 : gyro, accel, baro 칼리브레이션 관련
int TemperatureCompensation::initialize_parameter_handles(ParameterHandles &parameter_handles)
{
	char nbuf[16];
	int ret = PX4_ERROR;

	// gyro 온도 칼리브레이션 enable 파라미터 읽어오기
	/* rate gyro calibration parameters */
	parameter_handles.gyro_tc_enable = param_find("TC_G_ENABLE");
	int32_t gyro_tc_enabled = 0;
	ret = param_get(parameter_handles.gyro_tc_enable, &gyro_tc_enabled);

	if (ret == PX4_OK && gyro_tc_enabled) {
		for (unsigned j = 0; j < GYRO_COUNT_MAX; j++) {
			sprintf(nbuf, "TC_G%d_ID", j);
			parameter_handles.gyro_cal_handles[j].ID = param_find(nbuf);

			for (unsigned i = 0; i < 3; i++) {
				sprintf(nbuf, "TC_G%d_X3_%d", j, i);
				parameter_handles.gyro_cal_handles[j].x3[i] = param_find(nbuf);
				sprintf(nbuf, "TC_G%d_X2_%d", j, i);
				parameter_handles.gyro_cal_handles[j].x2[i] = param_find(nbuf);
				sprintf(nbuf, "TC_G%d_X1_%d", j, i);
				parameter_handles.gyro_cal_handles[j].x1[i] = param_find(nbuf);
				sprintf(nbuf, "TC_G%d_X0_%d", j, i);
				parameter_handles.gyro_cal_handles[j].x0[i] = param_find(nbuf);
				sprintf(nbuf, "TC_G%d_SCL_%d", j, i);
				parameter_handles.gyro_cal_handles[j].scale[i] = param_find(nbuf);
			}

			sprintf(nbuf, "TC_G%d_TREF", j);
			parameter_handles.gyro_cal_handles[j].ref_temp = param_find(nbuf);
			sprintf(nbuf, "TC_G%d_TMIN", j);
			parameter_handles.gyro_cal_handles[j].min_temp = param_find(nbuf);
			sprintf(nbuf, "TC_G%d_TMAX", j);
			parameter_handles.gyro_cal_handles[j].max_temp = param_find(nbuf);
		}
	}

	// accel 온도 칼리브레이션 enable 파라미터 읽어오기
	/* accelerometer calibration parameters */
	parameter_handles.accel_tc_enable = param_find("TC_A_ENABLE");
	int32_t accel_tc_enabled = 0;
	ret = param_get(parameter_handles.accel_tc_enable, &accel_tc_enabled);

	if (ret == PX4_OK && accel_tc_enabled) {
		for (unsigned j = 0; j < ACCEL_COUNT_MAX; j++) {
			sprintf(nbuf, "TC_A%d_ID", j);
			parameter_handles.accel_cal_handles[j].ID = param_find(nbuf);

			for (unsigned i = 0; i < 3; i++) {
				sprintf(nbuf, "TC_A%d_X3_%d", j, i);
				parameter_handles.accel_cal_handles[j].x3[i] = param_find(nbuf);
				sprintf(nbuf, "TC_A%d_X2_%d", j, i);
				parameter_handles.accel_cal_handles[j].x2[i] = param_find(nbuf);
				sprintf(nbuf, "TC_A%d_X1_%d", j, i);
				parameter_handles.accel_cal_handles[j].x1[i] = param_find(nbuf);
				sprintf(nbuf, "TC_A%d_X0_%d", j, i);
				parameter_handles.accel_cal_handles[j].x0[i] = param_find(nbuf);
				sprintf(nbuf, "TC_A%d_SCL_%d", j, i);
				parameter_handles.accel_cal_handles[j].scale[i] = param_find(nbuf);
			}

			sprintf(nbuf, "TC_A%d_TREF", j);
			parameter_handles.accel_cal_handles[j].ref_temp = param_find(nbuf);
			sprintf(nbuf, "TC_A%d_TMIN", j);
			parameter_handles.accel_cal_handles[j].min_temp = param_find(nbuf);
			sprintf(nbuf, "TC_A%d_TMAX", j);
			parameter_handles.accel_cal_handles[j].max_temp = param_find(nbuf);
		}
	}

	// baro 온도 칼리브레이션 enable 파라미터 읽어오기
	/* barometer calibration parameters */
	parameter_handles.baro_tc_enable = param_find("TC_B_ENABLE");
	int32_t baro_tc_enabled = 0;
	ret = param_get(parameter_handles.baro_tc_enable, &baro_tc_enabled);

	if (ret == PX4_OK && baro_tc_enabled) {
		for (unsigned j = 0; j < BARO_COUNT_MAX; j++) {
			sprintf(nbuf, "TC_B%d_ID", j);
			parameter_handles.baro_cal_handles[j].ID = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_X5", j);
			parameter_handles.baro_cal_handles[j].x5 = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_X4", j);
			parameter_handles.baro_cal_handles[j].x4 = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_X3", j);
			parameter_handles.baro_cal_handles[j].x3 = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_X2", j);
			parameter_handles.baro_cal_handles[j].x2 = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_X1", j);
			parameter_handles.baro_cal_handles[j].x1 = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_X0", j);
			parameter_handles.baro_cal_handles[j].x0 = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_SCL", j);
			parameter_handles.baro_cal_handles[j].scale = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_TREF", j);
			parameter_handles.baro_cal_handles[j].ref_temp = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_TMIN", j);
			parameter_handles.baro_cal_handles[j].min_temp = param_find(nbuf);
			sprintf(nbuf, "TC_B%d_TMAX", j);
			parameter_handles.baro_cal_handles[j].max_temp = param_find(nbuf);
		}
	}

	return PX4_OK;
}

// parameter_handles을 이용해서 _parameters를 업데이트. accel, gyro, baro 칼리브레이션에 필요한 값을 param에서 읽기
int TemperatureCompensation::parameters_update()
{
	int ret = 0;

	ParameterHandles parameter_handles;
	ret = initialize_parameter_handles(parameter_handles);

	if (ret != 0) {
		return ret;
	}

	// gyro의 온도 칼리브레이션이 enable 여부 확인
	/* rate gyro calibration parameters */
	param_get(parameter_handles.gyro_tc_enable, &(_parameters.gyro_tc_enable));

	if (_parameters.gyro_tc_enable == 1) {
		for (unsigned j = 0; j < GYRO_COUNT_MAX; j++) {
			if (param_get(parameter_handles.gyro_cal_handles[j].ID, &(_parameters.gyro_cal_data[j].ID)) == PX4_OK) {
				param_get(parameter_handles.gyro_cal_handles[j].ref_temp, &(_parameters.gyro_cal_data[j].ref_temp));
				param_get(parameter_handles.gyro_cal_handles[j].min_temp, &(_parameters.gyro_cal_data[j].min_temp));
				param_get(parameter_handles.gyro_cal_handles[j].max_temp, &(_parameters.gyro_cal_data[j].max_temp));

				for (unsigned int i = 0; i < 3; i++) {
					param_get(parameter_handles.gyro_cal_handles[j].x3[i], &(_parameters.gyro_cal_data[j].x3[i]));
					param_get(parameter_handles.gyro_cal_handles[j].x2[i], &(_parameters.gyro_cal_data[j].x2[i]));
					param_get(parameter_handles.gyro_cal_handles[j].x1[i], &(_parameters.gyro_cal_data[j].x1[i]));
					param_get(parameter_handles.gyro_cal_handles[j].x0[i], &(_parameters.gyro_cal_data[j].x0[i]));
					param_get(parameter_handles.gyro_cal_handles[j].scale[i], &(_parameters.gyro_cal_data[j].scale[i]));
				}

			} else { // gyro 칼리브레이션 ID를 param에서 읽기 실패한 경우, 관련 값을 초기값으로 설정
				// Set all cal values to zero and scale factor to unity
				memset(&_parameters.gyro_cal_data[j], 0, sizeof(_parameters.gyro_cal_data[j]));

				// Set the scale factor to unity
				for (unsigned int i = 0; i < 3; i++) {
					_parameters.gyro_cal_data[j].scale[i] = 1.0f;
				}

				PX4_WARN("FAIL GYRO %d CAL PARAM LOAD - USING DEFAULTS", j);
				ret = PX4_ERROR;
			}
		}
	}

	// accel의 온도 칼리브레이션이 enable 여부 확인
	/* accelerometer calibration parameters */
	param_get(parameter_handles.accel_tc_enable, &(_parameters.accel_tc_enable));

	if (_parameters.accel_tc_enable == 1) {
		for (unsigned j = 0; j < ACCEL_COUNT_MAX; j++) {
			if (param_get(parameter_handles.accel_cal_handles[j].ID, &(_parameters.accel_cal_data[j].ID)) == PX4_OK) {
				param_get(parameter_handles.accel_cal_handles[j].ref_temp, &(_parameters.accel_cal_data[j].ref_temp));
				param_get(parameter_handles.accel_cal_handles[j].min_temp, &(_parameters.accel_cal_data[j].min_temp));
				param_get(parameter_handles.accel_cal_handles[j].max_temp, &(_parameters.accel_cal_data[j].max_temp));

				for (unsigned int i = 0; i < 3; i++) {
					param_get(parameter_handles.accel_cal_handles[j].x3[i], &(_parameters.accel_cal_data[j].x3[i]));
					param_get(parameter_handles.accel_cal_handles[j].x2[i], &(_parameters.accel_cal_data[j].x2[i]));
					param_get(parameter_handles.accel_cal_handles[j].x1[i], &(_parameters.accel_cal_data[j].x1[i]));
					param_get(parameter_handles.accel_cal_handles[j].x0[i], &(_parameters.accel_cal_data[j].x0[i]));
					param_get(parameter_handles.accel_cal_handles[j].scale[i], &(_parameters.accel_cal_data[j].scale[i]));
				}

			} else { // accel 칼리브레이션 ID를 param에서 읽기 실패한 경우, 관련 값을 초기값으로 설정
				// Set all cal values to zero and scale factor to unity
				memset(&_parameters.accel_cal_data[j], 0, sizeof(_parameters.accel_cal_data[j]));

				// Set the scale factor to unity
				for (unsigned int i = 0; i < 3; i++) {
					_parameters.accel_cal_data[j].scale[i] = 1.0f;
				}

				PX4_WARN("FAIL ACCEL %d CAL PARAM LOAD - USING DEFAULTS", j);
				ret = PX4_ERROR;
			}
		}
	}

	// baro의 온도 칼리브레이션이 enable 여부 확인
	/* barometer calibration parameters */
	param_get(parameter_handles.baro_tc_enable, &(_parameters.baro_tc_enable));

	if (_parameters.baro_tc_enable == 1) {
		for (unsigned j = 0; j < BARO_COUNT_MAX; j++) {
			if (param_get(parameter_handles.baro_cal_handles[j].ID, &(_parameters.baro_cal_data[j].ID)) == PX4_OK) {
				param_get(parameter_handles.baro_cal_handles[j].ref_temp, &(_parameters.baro_cal_data[j].ref_temp));
				param_get(parameter_handles.baro_cal_handles[j].min_temp, &(_parameters.baro_cal_data[j].min_temp));
				param_get(parameter_handles.baro_cal_handles[j].max_temp, &(_parameters.baro_cal_data[j].max_temp));
				param_get(parameter_handles.baro_cal_handles[j].x5, &(_parameters.baro_cal_data[j].x5));
				param_get(parameter_handles.baro_cal_handles[j].x4, &(_parameters.baro_cal_data[j].x4));
				param_get(parameter_handles.baro_cal_handles[j].x3, &(_parameters.baro_cal_data[j].x3));
				param_get(parameter_handles.baro_cal_handles[j].x2, &(_parameters.baro_cal_data[j].x2));
				param_get(parameter_handles.baro_cal_handles[j].x1, &(_parameters.baro_cal_data[j].x1));
				param_get(parameter_handles.baro_cal_handles[j].x0, &(_parameters.baro_cal_data[j].x0));
				param_get(parameter_handles.baro_cal_handles[j].scale, &(_parameters.baro_cal_data[j].scale));

			} else { // baro 칼리브레이션 ID를 param에서 읽기 실패한 경우, 관련 값을 초기값으로 설정
				// Set all cal values to zero and scale factor to unity
				memset(&_parameters.baro_cal_data[j], 0, sizeof(_parameters.baro_cal_data[j]));

				// Set the scale factor to unity
				_parameters.baro_cal_data[j].scale = 1.0f;

				PX4_WARN("FAIL BARO %d CAL PARAM LOAD - USING DEFAULTS", j);
				ret = PX4_ERROR;
			}
		}
	}

	// param으로부터 offset과 scale 변경값으로 업데이트 될 수 있으므로, 이 값이 다음 보정에 사용하기 위해서 아래 data를 reset
	/* the offsets & scales might have changed, so make sure to report that change later when applying the
	 * next corrections
	 */
	_gyro_data.reset_temperature();
	_accel_data.reset_temperature();
	_baro_data.reset_temperature();

	return ret;
}

// 1차원 온도 offset 계산
bool TemperatureCompensation::calc_thermal_offsets_1D(SensorCalData1D &coef, float measured_temp, float &offset)
{
	bool ret = true;

	// 측정한 온도를 클립해서 칼리브레이션 범위 내에 있도록...
	// clip the measured temperature to remain within the calibration range
	float delta_temp;

	if (measured_temp > coef.max_temp) { // 측정한 온도가 coef.max를 넘는 경우
		delta_temp = coef.max_temp - coef.ref_temp;
		ret = false;

	} else if (measured_temp < coef.min_temp) { //측정한 온도가 coef.min보다 작은 경우
		delta_temp = coef.min_temp - coef.ref_temp;
		ret = false;

	} else { //정상인 경우
		delta_temp = measured_temp - coef.ref_temp;

	}

	// offset 계산
	// calulate the offset
	float temp_var = delta_temp;
	offset = coef.x0 + coef.x1 * temp_var;
	temp_var *= delta_temp;
	offset += coef.x2 * temp_var;
	temp_var *= delta_temp;
	offset += coef.x3 * temp_var;
	temp_var *= delta_temp;
	offset += coef.x4 * temp_var;
	temp_var *= delta_temp;
	offset += coef.x5 * temp_var;

	return ret;

}

//3차원(XYZ) 온도 offset 계산하기 (상관계수, 측정한 온도, offset인 인자로 사용)
bool TemperatureCompensation::calc_thermal_offsets_3D(const SensorCalData3D &coef, float measured_temp, float offset[])
{
	bool ret = true;

	// 측정한 온도를 칼리브레이션 범위 내에 남아 있도록 
	// clip the measured temperature to remain within the calibration range
	float delta_temp;

	if (measured_temp > coef.max_temp) { // 측정한 온도가 coef.max를 넘는 경우
		delta_temp = coef.max_temp - coef.ref_temp;
		ret = false;

	} else if (measured_temp < coef.min_temp) { // 측정한 온도가 coef.min보다 작은 경우
		delta_temp = coef.min_temp - coef.ref_temp;
		ret = false;

	} else { // 정상인 경우
		delta_temp = measured_temp - coef.ref_temp;

	}

	// offset을 계산
	// calulate the offsets
	float delta_temp_2 = delta_temp * delta_temp;
	float delta_temp_3 = delta_temp_2 * delta_temp;

	for (uint8_t i = 0; i < 3; i++) {
		offset[i] = coef.x0[i] + coef.x1[i] * delta_temp + coef.x2[i] * delta_temp_2 + coef.x3[i] * delta_temp_3;
	}

	return ret;

}

// tc_enalbe인 경우에 한해서 gyro device_id와 topic_instance를 매핑
int TemperatureCompensation::set_sensor_id_gyro(uint32_t device_id, int topic_instance)
{
	if (_parameters.gyro_tc_enable != 1) {
		return 0;
	}

	return set_sensor_id(device_id, topic_instance, _gyro_data, _parameters.gyro_cal_data, GYRO_COUNT_MAX);
}

// tc_enalbe인 경우에 한해서 accel device_id와 topic_instance를 매핑
int TemperatureCompensation::set_sensor_id_accel(uint32_t device_id, int topic_instance)
{
	if (_parameters.accel_tc_enable != 1) {
		return 0;
	}

	return set_sensor_id(device_id, topic_instance, _accel_data, _parameters.accel_cal_data, ACCEL_COUNT_MAX);
}

// tc_enalbe인 경우에 한해서 baro device_id와 topic_instance를 매핑
int TemperatureCompensation::set_sensor_id_baro(uint32_t device_id, int topic_instance)
{
	if (_parameters.baro_tc_enable != 1) {
		return 0;
	}

	return set_sensor_id(device_id, topic_instance, _baro_data, _parameters.baro_cal_data, BARO_COUNT_MAX);
}

// 센서 device id로 해당 센서에 대한 칼리브레이션 정보를 가지고 있는 i 인덱스를 sensor_data에 설정하도록. 
template<typename T>
int TemperatureCompensation::set_sensor_id(uint32_t device_id, int topic_instance, PerSensorData &sensor_data,
		const T *sensor_cal_data, uint8_t sensor_count_max)
{
	for (int i = 0; i < sensor_count_max; ++i) {
		if (device_id == sensor_cal_data[i].ID) {
			sensor_data.device_mapping[topic_instance] = i;
			return i;
		}
	}

	return -1;
}

int TemperatureCompensation::apply_corrections_gyro(int topic_instance, matrix::Vector3f &sensor_data,
		float temperature, float *offsets, float *scales)
{
	if (_parameters.gyro_tc_enable != 1) {
		return 0;
	}

	uint8_t mapping = _gyro_data.device_mapping[topic_instance];

	if (mapping == 255) { //255인 경우 mapping이 없는 값
		return -1;
	}

	// tc offset 계산
	calc_thermal_offsets_3D(_parameters.gyro_cal_data[mapping], temperature, offsets);

	// offset과 scales로 센서 data 계산
	// get the sensor scale factors and correct the data
	for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
		scales[axis_index] = _parameters.gyro_cal_data[mapping].scale[axis_index];
		sensor_data(axis_index) = (sensor_data(axis_index) - offsets[axis_index]) * scales[axis_index];
	}

	// 온도 차이가 1도 이상 나는 경우, 최신 온도 update
	if (fabsf(temperature - _gyro_data.last_temperature[topic_instance]) > 1.0f) {
		_gyro_data.last_temperature[topic_instance] = temperature;
		return 2;
	}

	return 1;
}

int TemperatureCompensation::apply_corrections_accel(int topic_instance, matrix::Vector3f &sensor_data,
		float temperature, float *offsets, float *scales)
{
	// 열보정 enable 되어 있지 않으면 return
	if (_parameters.accel_tc_enable != 1) {
		return 0;
	}

	// topic에 따른 param에서 해당 index 가져오기
	uint8_t mapping = _accel_data.device_mapping[topic_instance];

	if (mapping == 255) {
		return -1;
	}

	// tc offset 계산
	calc_thermal_offsets_3D(_parameters.accel_cal_data[mapping], temperature, offsets);

	// offset과 scales로 센서 data 계산
	// get the sensor scale factors and correct the data
	for (unsigned axis_index = 0; axis_index < 3; axis_index++) {
		scales[axis_index] = _parameters.accel_cal_data[mapping].scale[axis_index];
		sensor_data(axis_index) = (sensor_data(axis_index) - offsets[axis_index]) * scales[axis_index];
	}

	// 온도 차이가 1도 이상인 경우 baro 최신 온도를 업데이트. 
	if (fabsf(temperature - _accel_data.last_temperature[topic_instance]) > 1.0f) {
		_accel_data.last_temperature[topic_instance] = temperature;
		return 2;
	}

	return 1;
}

// baro의 sensor 데이터 구하기.(offset 계산, 최신 온도 업데이트)
int TemperatureCompensation::apply_corrections_baro(int topic_instance, float &sensor_data, float temperature,
		float *offsets, float *scales)
{
	// 열보정 enable 되어 있지 않으면 return
	if (_parameters.baro_tc_enable != 1) {
		return 0;
	}

	// topic에 따른 param에서 해당 index 가져오기
	uint8_t mapping = _baro_data.device_mapping[topic_instance];

	if (mapping == 255) {
		return -1;
	}

	// tc offset 계산
	calc_thermal_offsets_1D(_parameters.baro_cal_data[mapping], temperature, *offsets);

	// offset, scale로 센서 data 계산
	// get the sensor scale factors and correct the data
	*scales = _parameters.baro_cal_data[mapping].scale;
	sensor_data = (sensor_data - *offsets) * *scales;

	// 온도 차이가 1도 이상인 경우 baro 최신 온도를 업데이트. 
	if (fabsf(temperature - _baro_data.last_temperature[topic_instance]) > 1.0f) {
		_baro_data.last_temperature[topic_instance] = temperature;
		return 2;
	}

	return 1;
}

// gyro, accel, baro의 device id 출력
void TemperatureCompensation::print_status()
{
	PX4_INFO("Temperature Compensation:");
	PX4_INFO(" gyro: enabled: %i", _parameters.gyro_tc_enable);

	//gyro tc가 enable인 경우, device id 출력
	if (_parameters.gyro_tc_enable == 1) {
		for (int i = 0; i < GYRO_COUNT_MAX; ++i) {
			uint8_t mapping = _gyro_data.device_mapping[i];

			if (_gyro_data.device_mapping[i] != 255) {
				PX4_INFO("  using device ID %i for topic instance %i", _parameters.gyro_cal_data[mapping].ID, i);
			}
		}
	}

	PX4_INFO(" accel: enabled: %i", _parameters.accel_tc_enable);

	//accel tc가 enable인 경우, device id 출력
	if (_parameters.accel_tc_enable == 1) {
		for (int i = 0; i < ACCEL_COUNT_MAX; ++i) {
			uint8_t mapping = _accel_data.device_mapping[i];

			if (_accel_data.device_mapping[i] != 255) {
				PX4_INFO("  using device ID %i for topic instance %i", _parameters.accel_cal_data[mapping].ID, i);
			}
		}
	}

	PX4_INFO(" baro: enabled: %i", _parameters.baro_tc_enable);

	//baro tc가 enable인 경우, device id 출력
	if (_parameters.baro_tc_enable == 1) {
		for (int i = 0; i < BARO_COUNT_MAX; ++i) {
			uint8_t mapping = _baro_data.device_mapping[i];

			if (_baro_data.device_mapping[i] != 255) {
				PX4_INFO("  using device ID %i for topic instance %i", _parameters.baro_cal_data[mapping].ID, i);
			}
		}
	}
}

}
