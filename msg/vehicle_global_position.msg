# Fused global position in WGS84.
# This struct contains global position estimation. It is not the raw GPS
# measurement (@see vehicle_gps_position). This topic is usually published by the position
# estimator, which will take more sources of information into account than just GPS,
# e.g. control inputs of the vehicle in a Kalman-filter implementation.
#

float64 lat			# 위도
float64 lon			# 경도
float32 alt			# 해수면 고도

float32 delta_alt 	# 고도에 대한 델타 reset
uint8 lat_lon_reset_counter	# 수평 위치 좌표에 대한 reset 이벤트 카운터
uint8 alt_reset_counter 	# 고도에 대한 reset 이벤트 카운터

float32 vel_n			# NED earth-fixed 프레임에서 North velocity (metres/sec)
float32 vel_e			# NED earth-fixed 프레임에서 East velocity (metres/sec)
float32 vel_d			# NED earth-fixed 프레임에서 Down velocity (metres/sec)

float32 yaw 			# NED earth-fixed 프레임에 대한 Euler yaw 상대 각도 , -PI..+PI, (radians)

float32 eph			# 수평 위치 에러에 대한 표준편차 (metres)
float32 epv			# 수직 위치 에러에 대한 표준편차 (metres)

float32 terrain_alt		# 지형 고도 WGS84, (metres)
bool terrain_alt_valid		# 지형 고도 estimate가 유효

bool dead_reckoning		# dead-reckoning을 통한 위치 estimation인 경우 True

# TOPICS vehicle_global_position vehicle_global_position_groundtruth
