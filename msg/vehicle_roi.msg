# Vehicle Region Of Interest (ROI)

uint8 ROI_NONE = 0                         # No region of interest
uint8 ROI_WPNEXT = 1                       # 다음 MISSION으로
uint8 ROI_WPINDEX = 2                      # index로 주어진 MISSION으로 
uint8 ROI_LOCATION = 3                     # 고정 위치로
uint8 ROI_TARGET = 4                       # 타겟으로
uint8 ROI_ENUM_END = 5

uint8 mode          # ROI 모드

float64 lat			    # 위도
float64 lon			    # 경도
float32 alt			    # 고도

# 다음 waypoint에 대한 추가 각도 오프셋
float32 roll_offset		# 각도 오프셋 (rad 단위)
float32 pitch_offset		# 각도 오프셋 (rad 단위)
float32 yaw_offset		# 각도 오프셋 (rad 단위)

