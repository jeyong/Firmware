
# rates used by the controller
float32 rollspeed		# X 바디 축에 대한 각속도 (rad/s)
float32 pitchspeed		# Y 바디 축에 대한 각속도 (rad/s)
float32 yawspeed		# Z 바디 축에 대한 각속도 (rad/s)

# rate controller integrator status
float32 rollspeed_integ
float32 pitchspeed_integ
float32 yawspeed_integ
float32 additional_integ1	# 고정익: wheel rate integrator (옵션)
