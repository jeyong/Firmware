# This message is used to dump the raw gps communication to the log.
# Set the parameter GPS_DUMP_COMM to 1 to use this.

uint8 len			# 데이터의 길이. MSB bit이 설정된 경우 gps 장치로 메시지 전송. 0인 경우 gps 장치로부터 받기
uint8[79] data			# log에 데이터 쓰기
