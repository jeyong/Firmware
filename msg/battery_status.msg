float32 voltage_v		# 배터리 전압
float32 voltage_filtered_v	# 필터링된 배터리 전압
float32 current_a		# 배터리 전류
float32 current_filtered_a	# 필터링된 배터리 전류
float32 average_current_a		# 배터리 전류 평균
float32 discharged_mah		# mAh 단위의 방전된 양
float32 remaining		# 1에서 0까지의 남은양
float32 scale		# 전원 스케일링 팩터
float32 temperature         # 배터리 온도
int32 cell_count		# 셀의 수
bool connected			# 배터리가 연결되었는지 여부
bool system_source		# VDD_5V_IN에 대한 활성화 전원 소스가 배터리인지 여부
uint8 priority        	# 전원 컨트롤러에 연결된 우선순위
uint16 capacity         # 배터리 실제 용량
uint16 cycle_count         # 배터리의 방전 사이클 횟수
uint16 run_time_to_empty   # 남은 배터리 용량 예측
uint16 average_time_to_empty   # 남은 배터리 용량 예측(분당 방전되는 평균 비율 기반)
uint16 serial_number       # 배터리 팩의 시리얼 넘버


bool is_powering_off		# Power off 이벤트 긴급 표시 여부


uint8 BATTERY_WARNING_NONE = 0       # no battery low voltage warning active
uint8 BATTERY_WARNING_LOW = 1        # warning of low voltage
uint8 BATTERY_WARNING_CRITICAL = 2   # critical voltage, return / abort immediately
uint8 BATTERY_WARNING_EMERGENCY = 3   # immediate landing required
uint8 BATTERY_WARNING_FAILED = 4   # the battery has failed completely

uint8 warning    # 배터리 경고
