# GPS home position in WGS84 coordinates.

float64 lat				# 위도
float64 lon				# 경도
float32 alt				# 해수면 고도

float32 x				# X 좌포(m단위)
float32 y				# Y 좌포(m단위)
float32 z				# Z 좌포(m단위)

float32 yaw				# Yaw 각도(radian)

bool valid_alt		# 고도가 설정된 경우 true
bool valid_hpos		# 위도와 경도가 설정된 경우 true

bool manual_home	# home 위치가 수동으로 설정된 경우 true
