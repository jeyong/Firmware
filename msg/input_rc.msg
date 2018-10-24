uint8 RC_INPUT_SOURCE_UNKNOWN = 0
uint8 RC_INPUT_SOURCE_PX4FMU_PPM = 1
uint8 RC_INPUT_SOURCE_PX4IO_PPM = 2
uint8 RC_INPUT_SOURCE_PX4IO_SPEKTRUM = 3
uint8 RC_INPUT_SOURCE_PX4IO_SBUS = 4
uint8 RC_INPUT_SOURCE_PX4IO_ST24 = 5
uint8 RC_INPUT_SOURCE_MAVLINK = 6
uint8 RC_INPUT_SOURCE_QURT = 7
uint8 RC_INPUT_SOURCE_PX4FMU_SPEKTRUM = 8
uint8 RC_INPUT_SOURCE_PX4FMU_SBUS = 9
uint8 RC_INPUT_SOURCE_PX4FMU_ST24 = 10
uint8 RC_INPUT_SOURCE_PX4FMU_SUMD = 11
uint8 RC_INPUT_SOURCE_PX4FMU_DSM = 12
uint8 RC_INPUT_SOURCE_PX4IO_SUMD = 13

uint8 RC_INPUT_MAX_CHANNELS = 18 	# RC 입력 채널의 최대 갯수

uint64 timestamp_last_signal		# 최종 유효 수신 시간
uint32 channel_count			# 실제 사용하는 채널의 갯수
int32 rssi				# 수신 신호 세기 (RSSI)
bool rc_failsafe			# 명시적인 failsafe flag로 TX가 failure인 경우 true
bool rc_lost				# RC 수신기 연결 상태
uint16 rc_lost_frame_count		# 수신하지 못한 RC frame의 갯수
uint16 rc_total_frame_count		# 전체 RC 프레임의 갯수
uint16 rc_ppm_frame_length		# 단일 PPM 프레임의 갯수
uint8 input_source			# 입력 소스
uint16[18] values			# 지원하는 채널 각각에 대한 펄스의 폭을 측정
