# Relative position of precision land target in navigation (body fixed, north aligned, NED) and inertial (world fixed, north aligned, NED) frames

bool is_static			# 착륙 타겟이 고정된 상태인지 움직이는 상태인지 나타내는 flag

bool rel_pos_valid		# 상대 위치가 유효한지 여부를 나타내는 flag
bool rel_vel_valid		# 상대 속도가 유효한지 여부를 나타내는 flag

float32 x_rel			# 비행체에 대해서 타겟의 X/north 위치 [meters]
float32 y_rel			# 비행체에 대해서 타겟의 Y/east 위치 [meters]
float32 z_rel			# 비행체에 대해서 타겟의 Z/down 위치 [meters]

float32 vx_rel			# 비행체에 대해서 타겟의 X/north 속도 [meters/second]
float32 vy_rel			# 비행체에 대해서 타겟의 Y/east 속도 [meters/second]

float32 cov_x_rel		# X/north 위치 변화량 [meters^2]
float32 cov_y_rel		# Y/east 위치 변화량 [meters^2]

float32 cov_vx_rel		# X/north 속도 변화량 [(meters/second)^2]
float32 cov_vy_rel		# Y/east 속도 변화량 [(meters/second)^2]

bool abs_pos_valid		# 절대 위치가 유효한지 여부를 나타내는 flag
float32 x_abs			# 기준점에 대해서 타겟의 X/north 위치 [meters]
float32 y_abs			# 기준점에 대해서 타겟의 Y/east 위치 [meters]
float32 z_abs			# 기준점에 대해서 타겟의 Z/down 위치 [meters]
