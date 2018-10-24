# Main state, i.e. what user wants. Controlled by RC or from ground station via telemetry link.
uint8 MAIN_STATE_MANUAL = 0  # 메뉴얼 상태
uint8 MAIN_STATE_ALTCTL = 1  # 고도 컨트롤 상태
uint8 MAIN_STATE_POSCTL = 2  # 포지션 컨트롤 상태
uint8 MAIN_STATE_AUTO_MISSION = 3 # 미션 상태
uint8 MAIN_STATE_AUTO_LOITER = 4   # 로이터 상태
uint8 MAIN_STATE_AUTO_RTL = 5      # Return-To-Launch 상태
uint8 MAIN_STATE_ACRO = 6          # Acro 상태
uint8 MAIN_STATE_OFFBOARD = 7      # offboard 상태
uint8 MAIN_STATE_STAB = 8          # stable 상태
uint8 MAIN_STATE_RATTITUDE = 9     # RAttitude 상태
uint8 MAIN_STATE_AUTO_TAKEOFF = 10  # Takeoff 상태
uint8 MAIN_STATE_AUTO_LAND = 11    # Land 상태
uint8 MAIN_STATE_AUTO_FOLLOW_TARGET = 12  # Follow Target 상태
uint8 MAIN_STATE_AUTO_PRECLAND = 13       # 정밀 착륙 상태
uint8 MAIN_STATE_MAX = 14


uint8 main_state		    	# main 상태 머신
