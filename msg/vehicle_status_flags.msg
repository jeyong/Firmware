# This is a struct used by the commander internally.

bool condition_calibration_enabled
bool condition_system_sensors_initialized
bool condition_system_hotplug_timeout		# 핫플러그 센서 검색이 끝나면 true
bool condition_system_returned_to_home
bool condition_auto_mission_available
bool condition_global_position_valid		# global position estimate의 결과가 navigation에 사용할 정도의 품질인 경우 commander app에서 true로 설정
bool condition_home_position_valid			# home 위치가 유효한지 표시
bool condition_local_position_valid		# local position estimate의 결과가 navigation에 사용할 정도의 품질인 경우 commander app에서 true로 설정
bool condition_local_velocity_valid		# local horizontal 속도 데이터의 결과가 navigation에 사용할 정도의 품질인 경우 commander app에서 true로 설정
bool condition_local_altitude_valid
bool condition_power_input_valid                # 입력 전원이 유효한 경우 true 설정

bool circuit_breaker_engaged_power_check
bool circuit_breaker_engaged_airspd_check
bool circuit_breaker_engaged_enginefailure_check
bool circuit_breaker_engaged_gpsfailure_check
bool circuit_breaker_flight_termination_disabled
bool circuit_breaker_engaged_usb_check
bool circuit_breaker_engaged_posfailure_check	# 위치 유효 검사가 비활성화된 경우 true로 설정

bool offboard_control_signal_found_once
bool offboard_control_signal_lost
bool offboard_control_set_by_command                # offboard 모드가 mavlink 명령으로 설정된 경우 true
bool offboard_control_loss_timeout                # offboard가 특정 시간 동안 lost된 경우 true

bool rc_signal_found_once
bool rc_input_blocked                                # RC 입력이 임시로 block하는 경우 설정
bool rc_calibration_valid                            # RC 칼리브레이션이 유효한 경우 설정
bool vtol_transition_failure                        # vtol 전환이 실패한 경우 true로 설정
bool usb_connected                                # USB 전원 공급의 상태
