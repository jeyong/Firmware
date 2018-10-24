# If you change the order, add or remove arming_state_t states make sure to update the arrays
# in state_machine_helper.cpp as well.
uint8 ARMING_STATE_INIT = 0
uint8 ARMING_STATE_STANDBY = 1
uint8 ARMING_STATE_ARMED = 2
uint8 ARMING_STATE_STANDBY_ERROR = 3
uint8 ARMING_STATE_REBOOT = 4
uint8 ARMING_STATE_IN_AIR_RESTORE = 5
uint8 ARMING_STATE_MAX = 6

uint8 HIL_STATE_OFF = 0
uint8 HIL_STATE_ON = 1

# Navigation state, i.e. "what should vehicle do".
uint8 NAVIGATION_STATE_MANUAL = 0		# Manual 모드
uint8 NAVIGATION_STATE_ALTCTL = 1		# Altitude 제어 모드
uint8 NAVIGATION_STATE_POSCTL = 2		# Position 제어 모드
uint8 NAVIGATION_STATE_AUTO_MISSION = 3		# Auto mission 모드
uint8 NAVIGATION_STATE_AUTO_LOITER = 4		# Auto loiter 모드
uint8 NAVIGATION_STATE_AUTO_RTL = 5		# Auto RTL 모드
uint8 NAVIGATION_STATE_AUTO_RCRECOVER = 6	# RC recover 모드
uint8 NAVIGATION_STATE_AUTO_RTGS = 7		# data link loss인 경우 자동 착륙
uint8 NAVIGATION_STATE_AUTO_LANDENGFAIL = 8 	# 엔진 고장시 자동 착륙
uint8 NAVIGATION_STATE_AUTO_LANDGPSFAIL = 9	# GPS 고장시 자동 착륙
uint8 NAVIGATION_STATE_ACRO = 10		# Acro 모드
uint8 NAVIGATION_STATE_UNUSED = 11		# Free slot
uint8 NAVIGATION_STATE_DESCEND = 12		# Descend 모드 (position control이 없는 경우)
uint8 NAVIGATION_STATE_TERMINATION = 13		# Termination 모드
uint8 NAVIGATION_STATE_OFFBOARD = 14
uint8 NAVIGATION_STATE_STAB = 15		# Stabilized 모드
uint8 NAVIGATION_STATE_RATTITUDE = 16		# Rattitude (aka "flip") 모드
uint8 NAVIGATION_STATE_AUTO_TAKEOFF = 17	# Takeoff
uint8 NAVIGATION_STATE_AUTO_LAND = 18		# Land
uint8 NAVIGATION_STATE_AUTO_FOLLOW_TARGET = 19	# Auto Follow
uint8 NAVIGATION_STATE_AUTO_PRECLAND = 20	# 정밀 착륙
uint8 NAVIGATION_STATE_MAX = 21

uint8 RC_IN_MODE_DEFAULT = 0
uint8 RC_IN_MODE_OFF = 1
uint8 RC_IN_MODE_GENERATED = 2

# 상태 머신 / 비행체의 상태
# Encodes the complete system state and is set by the commander app.

uint8 nav_state				# navigation 상태 머신을 특정 값으로 설정
uint8 arming_state			# 현재 arming 상태
uint8 hil_state				# 현재 hil 상태
bool failsafe				# 시스템이 failsafe 상태면 true

uint8 system_type			# 시스템 타입(MAV_TYPE)
uint8 system_id			# system id
uint8 component_id			# subsystem / component id

bool is_rotary_wing			# 시스템이 rotary wing 설정인 경우 true
bool is_vtol				# 시스템이 vtol인 경우 true
bool vtol_fw_permanent_stab		# vtol이 manual 모드에서 자세 안정 모드를 사용하는 경우 true 
bool in_transition_mode			# vtol이 전환하는 경우 true
bool in_transition_to_fw		# vtol이 MC에서 FW로 전환하는 경우 true

bool rc_signal_lost				# RC 수신을 잃은 경우 true
uint8 rc_input_mode				# RC 입력을 비활성화 시키는 경우 1로 설정하며 manual 제어를 RC 매핑으로 활성화하는 경우 2로 설정

bool data_link_lost				# GCS와 datalink를 잃는 경우
bool high_latency_data_link_active 		# GCS los에 대해서 low latency datlaink 허용
uint8 data_link_lost_counter			# counts unique data link lost events
bool engine_failure				# 엔진이 고장을 탐지한 경우 true로 설정
bool mission_failure				# 미션을 계속 진행할 수 없는 경우 true로 설정

# see SYS_STATUS mavlink message for the following
uint32 onboard_control_sensors_present
uint32 onboard_control_sensors_enabled
uint32 onboard_control_sensors_health
