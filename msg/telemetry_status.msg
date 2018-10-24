uint8 TELEMETRY_STATUS_RADIO_TYPE_GENERIC = 0
uint8 TELEMETRY_STATUS_RADIO_TYPE_3DR_RADIO = 1
uint8 TELEMETRY_STATUS_RADIO_TYPE_UBIQUITY_BULLET = 2
uint8 TELEMETRY_STATUS_RADIO_TYPE_WIRE = 3
uint8 TELEMETRY_STATUS_RADIO_TYPE_USB = 4
uint8 TELEMETRY_STATUS_RADIO_TYPE_IRIDIUM = 5

uint64 heartbeat_time			# 마지막 수신한 heartbeat의 시간
uint64 telem_time			# 마지막 수신한 status 패킷의 시간
uint8 type				# radio 하드웨어의 타입
uint8 rssi				# 로컬 신호 세기
uint8 remote_rssi			# 리모트 신호 세기
uint16 rxerrors				# 수신 에러
uint16 fixed				# 에러 보정 패킷 카운트
uint8 noise				# 백그라운드 노이즈 레벨
uint8 remote_noise			# 리모트 백그라운드 노이즈 레벨
uint8 txbuf				# tx 버터 사용률
uint8 system_id				# 리모트 시스템의 system id
uint8 component_id			# 리모트 시스템의 component id
