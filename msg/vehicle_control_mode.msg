
						# is set whenever the writing thread stores new data

bool flag_armed

bool flag_external_manual_override_ok		# 외부에서 시스템이 정상상태라고 flag값 설정. 고정익에서만 사용

# XXX needs yet to be set by state machine helper
bool flag_system_hil_enabled

bool flag_control_manual_enabled		# manual 입력이 믹스된 경우 true
bool flag_control_auto_enabled			# 온보드 autopilot이 동작하는 경우 true
bool flag_control_offboard_enabled		# offboard 제어를 사용하는 경우 true
bool flag_control_rates_enabled			# rate가 안정화된 경우 true
bool flag_control_attitude_enabled		# attitude 모드를 사용하는 경우 true
bool flag_control_rattitude_enabled		# rate/attitude 모드를 사용하는 경우 true
bool flag_control_force_enabled			# force 제어를 사용하는 경우 true
bool flag_control_acceleration_enabled			# 가속도를 제어하는 경우 true
bool flag_control_velocity_enabled		# 수평 속도를 제어하는 경우 true
bool flag_control_position_enabled		# 위치를 제어하는 경우 true
bool flag_control_altitude_enabled		# 자세를 제어하는 경우 true
bool flag_control_climb_rate_enabled		# climb 속도를 제어하는 경우 true
bool flag_control_termination_enabled		# 비행종료를 활성화시키는 경우 true
bool flag_control_fixed_hdg_enabled			# 고정된 헤딩을 사용하는 경우 true
