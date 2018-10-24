uint64 integral_dt	# 적분 시간
uint64 error_count
float32 x		# NED X 보드 축에 대한 가속(m/s^2)
float32 y		# NED Y 보드 축에 대한 가속(m/s^2)
float32 z		# NED Z 보드 축에 대한 가속(m/s^2)
float32 x_integral	# NED X 보드 축에 대한 속도(m/s)
float32 y_integral	# NED Y 보드 축에 대한 속도(m/s)
float32 z_integral	# NED Z 보드 축에 대한 속도(m/s)
float32 temperature	# 온도
float32 range_m_s2	# 범위 (m/s^2)
float32 scaling

int16 x_raw
int16 y_raw
int16 z_raw
int16 temperature_raw

uint32 device_id	# 변경되지 않는 센서에 대해서 고유 장치 ID
