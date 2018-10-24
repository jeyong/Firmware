uint8 MISSION_EXECUTION_MODE_NORMAL = 0	
uint8 MISSION_EXECUTION_MODE_REVERSE = 1	
uint8 MISSION_EXECUTION_MODE_FAST_FORWARD = 2	

uint32 instance_count		# 미션의 instance 카운트로 mission이 수정되는 경우 증가

int32 seq_reached		# 도달한 mission item의 순서
uint16 seq_current		# 현재 mission item의 순서
uint16 seq_total		# mission item의 전체 갯수

bool valid			# mission이 유효하면 true
bool warning			# mission이 유효하면 true하지만 문제가 있는 item인 경우 warning 발생
bool finished			# mission이 완료되면 true
bool failure			# mission이 연속으로 진행되지 않거나 완료된 경우 true

bool stay_in_failsafe		# commander가 failsafe 모드를 유지하면 경우 true
bool flight_termination		# navigator가 commander app으로부터 비행종료를 요청하면 true

bool item_do_jump_changed	# do jumps의 남은 갯수가 변경되면 true
uint16 item_changed_index	# item이 변경되었는지를 표시
uint16 item_do_jump_remaining	# do jumps의 남은 갯수를 설정

uint8 execution_mode	# mission이 실행되는 모드를 표시
