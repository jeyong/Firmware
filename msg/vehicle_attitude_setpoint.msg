int8 LANDING_GEAR_UP = 1	# 랜딩기어 up
int8 LANDING_GEAR_DOWN = -1	# 랜딩기어 down

float32 roll_body				# NED 프레임에서 body 각도
float32 pitch_body				# NED 프레임에서 body 각도
float32 yaw_body				# NED 프레임에서 body 각도

float32 yaw_sp_move_rate		# yaw 각속도 rad/s

# For 쿼터니언 기반 자세 제어
float32[4] q_d					# 쿼터니언 제어를 위한 Desired quaternion
bool q_d_valid					# 쿼터니언 벡터가 유효한 경우 true

float32 thrust					

bool roll_reset_integral			# roll integral 부분을 reset
bool pitch_reset_integral			# pitch integral 부분을 reset
bool yaw_reset_integral				# yaw integral 부분을 reset

bool fw_control_yaw					# rudder로 헤딩 제어
bool disable_mc_yaw_control			# 멀티로터를 위한 yaw 제어

bool apply_flaps

float32 landing_gear

# TOPICS vehicle_attitude_setpoint mc_virtual_attitude_setpoint fw_virtual_attitude_setpoint
