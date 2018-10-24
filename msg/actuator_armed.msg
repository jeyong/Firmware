
uint32 armed_time_ms	# Arming timestamp
bool armed		# arming 여부 설정
bool prearmed		# 액츄레이터 safety가 비활성화되어 있고 모터는 arming 상태가 아닌 경우 true로 설정
bool ready_to_arm	# arming 상태로 진입 가능한 상태인 경우 true
bool lockdown		# 액츄레이터가 강제로 비활성화되어 있는 경우 true로 설정 (비상상황 or HIL인 경우)
bool manual_lockdown    # 메뉴얼 쓰로틀 킬 스위치 사용이 가능한 경우 true
bool force_failsafe	# 액츄레이터가 강제로 failsafe 위치로 간 경우 true
bool in_esc_calibration_mode # IO/FMU는 액츄레이터 control topic과 관련된 msg를 무시
bool soft_stop # ESC가 idle 상태 제약을 제거해야할 필요가 있는 경우 true로 설정
