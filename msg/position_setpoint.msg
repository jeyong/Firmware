# this file is only used in the position_setpoint triple as a dependency

uint8 SETPOINT_TYPE_POSITION=0	
uint8 SETPOINT_TYPE_VELOCITY=1	
uint8 SETPOINT_TYPE_LOITER=2	
uint8 SETPOINT_TYPE_TAKEOFF=3	
uint8 SETPOINT_TYPE_LAND=4	
uint8 SETPOINT_TYPE_IDLE=5	
uint8 SETPOINT_TYPE_OFFBOARD=6 	
uint8 SETPOINT_TYPE_FOLLOW_TARGET=7

uint8 VELOCITY_FRAME_LOCAL_NED = 1 # MAV_FRAME_LOCAL_NED
uint8 VELOCITY_FRAME_BODY_NED = 8 # MAV_FRAME_BODY_NED

bool valid			# setpoint가 유효하면 true
uint8 type			# position 컨트롤러의 동작을 조정하기 위한 setpoint 타입

float32 x			# 로컬 position setpoint NED
float32 y			# 로컬 position setpoint NED
float32 z			# 로컬 position setpoint NED
bool position_valid		# 로컬 position setpoint가 유효하면 true

float32 vx			# 로컬 속도 setpoint NED (m/s)
float32 vy			# 로컬 속도 setpoint NED (m/s)
float32 vz			# 로컬 속도 setpoint NED (m/s)
bool velocity_valid		# 로컬 속도 setpoint가 유효하면 true
uint8 velocity_frame		# 속도 setpoints를 NED 혹은 body로 설정
bool alt_valid		# 3D position 컨트롤을 설정하지 않기. vx,vy 속도 제어하는 동안 z-position 제어를 원하는 경우 true

float64 lat			# 위도
float64 lon			# 경도
float32 alt			# 해수면 고도

float32 yaw			# yaw
bool yaw_valid			# yaw setpoint가 유효하면 true

float32 yawspeed		# yaw speed
bool yawspeed_valid		# yaw speed setpoind가 유효하면 true

float32 loiter_radius		# loiter radius 
int8 loiter_direction		# loiter 방향: 1 = CW, -1 = CCW
float32 pitch_min		# 고정익 이륙 waypoint를 위해서 최소 pitch 각도

float32 a_x			# 가속도 x setpoint
float32 a_y			# 가속도 y setpoint
float32 a_z			# 가속도 z setpoint
bool acceleration_valid		# 가속도 setpoint가 유효하면 true
bool acceleration_is_force	# 가속도를 힘으로 해석

float32 acceptance_radius   # waypoint navigation을 하는 경우 navigation 수용 각도

float32 cruising_speed		# 일반적인 크루징 속도
float32 cruising_throttle	# 일반적인 크루징 throttle
