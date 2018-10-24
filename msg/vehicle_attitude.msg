# This is similar to the mavlink message ATTITUDE_QUATERNION, but for onboard use

float32 rollspeed	# X 바디 축에 대해서 보정된 각속도 (rad/s)
float32 pitchspeed	# Y 바디 축에 대해서 보정된 각속도 (rad/s)
float32 yawspeed	# Z 바디 축에 대해서 보정된 각속도 (rad/s)

float32[4] q		# NED earth 프레임에서 XYZ 바디 프레임으로 쿼터니언 회전
float32[4] delta_q_reset 	# 마지막 reset 동안 Amount by which quaternion has changed during last reset
uint8 quat_reset_counter	# 쿼터니언 리셋 카운터

# TOPICS vehicle_attitude vehicle_attitude_groundtruth vehicle_vision_attitude
