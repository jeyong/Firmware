uint32 ICAO_address 	# ICAO address
float64 lat 		# 위도
float64 lon 		# 경도
uint8 altitude_type	# ADSB_ALTITUDE_TYPE enum으로 표현되는 타입값
float32 altitude 	# 고도 (단위 m)
float32 heading 	# 지상에서의 헤딩 (단위 radian)
float32 hor_velocity	# 수평 속도(m/s)
float32 ver_velocity 	# 수직 속도(m/s)
char[9] callsign	# callsign
uint8 emitter_type 	# ADSB_EMITTER_TYPE enum으로 표현되는 타입값
uint8 tslc 		# 마지막 통신 이후 시간 (단위 s)
uint16 flags 		# 유효 데이터 필드를 포함한 여러 상태를 표시하는 flag
uint16 squawk 		# Squawk 코드

# ADSB flags
uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1
uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2
uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4
uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8
uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16
uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32
uint16 PX4_ADSB_FLAGS_RETRANSLATE = 256

uint32 ORB_QUEUE_LENGTH = 10
