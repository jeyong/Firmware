uint64 integral_dt	# integration time
uint64 error_count
float32 x		# 각속도 NED X 보드 축(rad/s)
float32 y		# 각속도 NED Y 보드 축(rad/s)
float32 z		# 각속도 NED Z 보드 축(rad/s)
float32 x_integral	# 델타 각속도 (NED X 보드 축)
float32 y_integral	# 델타 각속도 (NED Y 보드 축)
float32 z_integral	# 델타 각속도 (NED Z 보드 축)
float32 temperature	# 온도(단위 도씨)
float32 range_rad_s
float32 scaling

int16 x_raw
int16 y_raw
int16 z_raw
int16 temperature_raw

uint32 device_id	# 센서마다 고유 device ID
