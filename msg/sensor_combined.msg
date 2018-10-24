#
# Sensor readings in SI-unit form.
#
# These fields are scaled and offset-compensated where possible and do not
# change with board revisions and sensor updates.
#

int32 RELATIVE_TIMESTAMP_INVALID = 2147483647 # (0x7fffffff) timestamp가 이 값으로 설정된 경우 센서값이 유효하지 않음을 뜻하는 고정값

# gyro timstamp is equal to the timestamp of the message
float32[3] gyro_rad			# 평균 각속도로 XYZ 바디 프레임 기준
uint32 gyro_integral_dt		# gyro 측정 샘플링 주기(단위 us)

int32 accelerometer_timestamp_relative	# timestamp + accelerometer_timestamp_relative = Accelerometer timestamp
float32[3] accelerometer_m_s2		# XYZ 바디 프레임 기준 측정 가속도 평균값 (m/s/s)
uint32 accelerometer_integral_dt	# 가속도 측정 샘플링 주기(단위 us)
