float32 voltage5V_v		# 5V rail 전압
float32 voltage3V3_v	# 센서 3V3 rail 전압
uint8 v3v3_valid		# 센서 3V3 rail 전압 유효
uint8 usb_connected		# 1이면 USB가 연결
uint8 brick_valid		# 1인 경우 brick bits power가 양호한 상태
uint8 usb_valid 		# USB가 유효하면 1
uint8 servo_valid		# servo power가 양호하면 1
uint8 periph_5V_OC		# 과전류인 경우 1
uint8 hipower_5V_OC		# hi power에 과전류인 경우 1

uint8 BRICK1_VALID_SHIFTS=0
uint8 BRICK1_VALID_MASK=1
uint8 BRICK2_VALID_SHIFTS=1
uint8 BRICK2_VALID_MASK=2
uint8 BRICK3_VALID_SHIFTS=2
uint8 BRICK3_VALID_MASK=4
uint8 BRICK4_VALID_SHIFTS=3
uint8 BRICK4_VALID_MASK=8

