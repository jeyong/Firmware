uint8 SAT_INFO_MAX_SATELLITES = 20

uint8 count			# satellite info에 있는 위성의 갯수
uint8[20] svid	 		# 위성 ID [1..255]
uint8[20] used			# 0: 위성 사용하지 않음, 1: 위성 사용
uint8[20] elevation		# 위성의 앙각
uint8[20] azimuth		# 위성이 방향
uint8[20] snr			# dBHz, 위성 신호의 노이즈 비율
