uint8 NUM_ACTUATOR_CONTROLS = 8   // 액츄레이터 제어 갯수는 총 8개
uint8 NUM_ACTUATOR_CONTROL_GROUPS = 4 // 액츄레이터 제어 그룹의 갯수는 4개
uint8 INDEX_ROLL = 0
uint8 INDEX_PITCH = 1
uint8 INDEX_YAW = 2
uint8 INDEX_THROTTLE = 3
uint8 INDEX_FLAPS = 4
uint8 INDEX_SPOILERS = 5
uint8 INDEX_AIRBRAKES = 6
uint8 INDEX_LANDING_GEAR = 7
uint8 GROUP_INDEX_ATTITUDE = 0
uint8 GROUP_INDEX_ATTITUDE_ALTERNATE = 1
uint64 timestamp_sample	    # 제어 응답 기반 데이터가 샘플링되는 timestamp
float32[8] control          # 제어 데이터

# TOPICS actuator_controls actuator_controls_0 actuator_controls_1 actuator_controls_2 actuator_controls_3
# TOPICS actuator_controls_virtual_fw actuator_controls_virtual_mc

