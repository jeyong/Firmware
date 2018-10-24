# Fused local position in NED.

bool xy_valid				# x와 y가 유효한 경우 true
bool z_valid				# z가 유효한 경우 true
bool v_xy_valid				# vx와 vy가 유효한 경우 true
bool v_z_valid				# vz가 유효한 경우 true

# Position in local NED frame
float32 x				# NED earth-fixed 프레임에 대해서 North position (metres)
float32 y				# NED earth-fixed 프레임에 대해서 East position (metres)
float32 z				# NED earth-fixed 프레임에 대해서 Down position (metres)

# Position reset delta
float32[2] delta_xy
uint8 xy_reset_counter

float32 delta_z
uint8 z_reset_counter

# Velocity in NED frame
float32 vx 				# NED earth-fixed 프레임에서 North velocity (metres/sec)
float32 vy				# NED earth-fixed 프레임에서 East velocity (metres/sec)
float32 vz				# NED earth-fixed 프레임에서 Down velocity (metres/sec)
float32 z_deriv				# NED earth-fixed 프레임에서 Down position time derivative

# Velocity reset delta
float32[2] delta_vxy
uint8 vxy_reset_counter

float32 delta_vz
uint8 vz_reset_counter
# NED 프레임에서 가속도
float32 ax        # NED earth-fixed 프레임에서 North velocity derivative (metres/sec^2)
float32 ay        # NED earth-fixed 프레임에서 East velocity derivative (metres/sec^2)
float32 az        # NED earth-fixed 프레임에서 Down velocity derivative (metres/sec^2)

# Heading
float32 yaw				# 오일러 yaw 각 Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians) 

# global (GPS / WGS84) 프레임에서 참조 지점의 위치 (local NED frame origin)
bool xy_global				# 위치 (x, y)가 유효한 global reference인 경우에 true
bool z_global				# z가 유효한 global reference인 경우 true
uint64 ref_timestamp			# 시스템이 시작한 이후로 reference position이 설정된 때의 시간
float64 ref_lat				# Reference point 위도
float64 ref_lon				# Reference point 경도
float32 ref_alt				# Reference 해수면 고도

# 표면과의 거리
float32 dist_bottom			# 바닥 표면에서 지상까지의 거리 (metres)
float32 dist_bottom_rate		# 바닥 표면에서 지상까지의 거리의 변화율 (metres/sec)
bool dist_bottom_valid			# 바닥 표면까지의 거리가 유효한 경우 true

float32 eph				# 수평 위치 에러에 대한 표준편차 (metres)
float32 epv				# 수직 위치 에러에 대한 표준편차 (metres)
float32 evh				# 수평 속도 에러에 대한 표준편차 (metres/sec)
float32 evv				# 수직 속도 에러에 대한 표준편차 (metres/sec)

# estimator specified vehicle limits
float32 vxy_max				# 최대 수평 속도 - 제한이 필요없는 경우 0 (meters/sec)
float32 vz_max				# 최대 수직 속도 - 제한이 필요없는 경우 0 (meters/sec)
float32 hagl_min			# 지상 위 최소 높이 - 제한이 필요없는 경우 0 (meters)
float32 hagl_max			# 지상 위 최대 높이 - 제한이 필요없는 경우 0 (meters)

# TOPICS vehicle_local_position vehicle_local_position_groundtruth vehicle_vision_position
