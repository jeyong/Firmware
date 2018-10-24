uint8 len			# 데이터의 길이
uint8 flags         		# LSB가 1인 경우 : fragmented
uint8[182] data		# GPS 장치로 데이터 쓰기 (RTCM 메시지)
