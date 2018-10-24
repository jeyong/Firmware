#
# Sensor corrections in SI-unit form for the voted sensor
#

# Corrections for gyro angular rate outputs where corrected_rate = raw_rate * gyro_scale + gyro_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
# corrections for uORB instance 0
float32[3] gyro_offset_0	# 센서 프레임 기준 gyro XYZ 오프셋 값(rad/s)
float32[3] gyro_scale_0		# gyro XYZ 스케일 팩터

# corrections for uORB instance 1
float32[3] gyro_offset_1	# 센서 프레임 기준 gyro XYZ 오프셋 값(rad/s)
float32[3] gyro_scale_1		# gyro XYZ 스케일 팩터

# corrections for uORB instance 2
float32[3] gyro_offset_2	# 센서 프레임 기준 gyro XYZ 오프셋 값(rad/s)
float32[3] gyro_scale_2		# gyro XYZ 스케일 팩터

# Corrections for acceleromter acceleration outputs where corrected_accel = raw_accel * accel_scale + accel_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
# corrections for uORB instance 0
float32[3] accel_offset_0	# 센서 프레임 기준 가속도 XYZ 오프셋 값 (m/s/s)
float32[3] accel_scale_0	# 가속도 XYZ 스케일 팩터

# corrections for uORB instance 1
float32[3] accel_offset_1	# 센서 프레임 기준 가속도 XYZ 오프셋 값 (m/s/s)
float32[3] accel_scale_1	# 가속도 XYZ 스케일 팩터

# corrections for uORB instance 2
float32[3] accel_offset_2	# 센서 프레임 기준 가속도 XYZ 오프셋 값 (m/s/s)
float32[3] accel_scale_2	# 가속도 XYZ 스케일 팩터

# Corrections for barometric pressure outputs where corrected_pressure = raw_pressure * pressure_scale + pressure_offset
# Note the corrections are in the sensor frame and must be applied before the sensor data is rotated into body frame
# corrections for uORB instance 0
float32 baro_offset_0		# 센서 프레임 기준 barometric pressure 오프셋 값 (m/s/s)
float32 baro_scale_0		# barometric pressure 스케일 팩터

# corrections for uORB instance 1
float32 baro_offset_1		# 센서 프레임 기준 barometric pressure 오프셋 값 (m/s/s)
float32 baro_scale_1		# barometric pressure 스케일 팩터

# corrections for uORB instance 2
float32 baro_offset_2		# 센서 프레임 기준 barometric pressure 오프셋 값 (m/s/s)
float32 baro_scale_2		# barometric pressure 스케일 팩터

uint8 selected_gyro_instance	# 선출된 센서에 대해서 gyro uORB topic 인스턴스
uint8 selected_accel_instance	# 선출된 센서에 대해서 accelerometer uORB topic 인스턴스
uint8 selected_baro_instance	# 선출된 센서에 대해서 barometric pressure uORB topic 인스턴스

# 각 센서 타입에 대해서 uORB 인덱스와 parameter 인덱스를 매핑
uint8[3] gyro_mapping
uint8[3] accel_mapping
uint8[3] baro_mapping
