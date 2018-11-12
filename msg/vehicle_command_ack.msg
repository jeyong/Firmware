uint8 VEHICLE_RESULT_ACCEPTED = 0
uint8 VEHICLE_RESULT_TEMPORARILY_REJECTED = 1
uint8 VEHICLE_RESULT_DENIED = 2
uint8 VEHICLE_RESULT_UNSUPPORTED = 3
uint8 VEHICLE_RESULT_FAILED = 4
uint8 VEHICLE_RESULT_IN_PROGRESS = 5

uint16 ARM_AUTH_DENIED_REASON_GENERIC = 0
uint16 ARM_AUTH_DENIED_REASON_NONE = 1
uint16 ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT = 2
uint16 ARM_AUTH_DENIED_REASON_TIMEOUT = 3
uint16 ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE = 4
uint16 ARM_AUTH_DENIED_REASON_BAD_WEATHER = 5

uint32 ORB_QUEUE_LENGTH = 3

uint16 command # 명령
uint8 result  # 결과
bool from_external  #외부에서 온 명령인지 여부
uint8 result_param1   #결과 param1
int32 result_param2   #결과 param2
uint8 target_system   #target system
uint8 target_component #target component
