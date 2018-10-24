# DISTANCE_SENSOR message data


float32 min_distance		# 센서가 측정하는 최소 거리 (m 단위)
float32 max_distance		# 센서가 측정하는 최대 거리 (m 단위)
float32 current_distance	# 현재 측정한 거리 (m 단위)
float32 covariance		# 측정 공분산 (m 단위)

uint8 type			# MAV_DISTANCE_SENSOR enum으로 쓰이는 타입
uint8 MAV_DISTANCE_SENSOR_LASER = 0
uint8 MAV_DISTANCE_SENSOR_ULTRASOUND = 1
uint8 MAV_DISTANCE_SENSOR_INFRARED = 2
uint8 MAV_DISTANCE_SENSOR_RADAR = 3

uint8 id			# 센서의 온보드 ID

uint8 orientation		# 센서 부탁 방향
uint8 ROTATION_DOWNWARD_FACING = 25 # MAV_SENSOR_ROTATION_PITCH_270
uint8 ROTATION_UPWARD_FACING   = 24 # MAV_SENSOR_ROTATION_PITCH_90
uint8 ROTATION_BACKWARD_FACING = 12 # MAV_SENSOR_ROTATION_PITCH_180
uint8 ROTATION_FORWARD_FACING  = 0  # MAV_SENSOR_ROTATION_NONE
uint8 ROTATION_LEFT_FACING     = 6  # MAV_SENSOR_ROTATION_YAW_270
uint8 ROTATION_RIGHT_FACING    = 2  # MAV_SENSOR_ROTATION_YAW_90
