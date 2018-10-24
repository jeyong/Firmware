# Local position setpoint in NED frame
# setting something to NaN means the state should not be controlled

float32 x			# NED에서 x 방향
float32 y			# NED에서 y 방향
float32 z			# NED에서 z 방향
float32 yaw			# NED에서 radians 값으로
float32 yawspeed	# radians/sec
float32 vx			# meters/sec
float32 vy			# meters/sec
float32 vz			# meters/sec
float32 acc_x		# meters/(sec*sec)
float32 acc_y		# meters/(sec*sec)
float32 acc_z		# meters/(sec*sec)
float32[3] thrust	# NED에서 thrust 벡터를 정규화
