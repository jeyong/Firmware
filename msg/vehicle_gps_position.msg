# GPS position in WGS84 coordinates.
# the auto-generated field 'timestamp' is for the position & velocity (microseconds)
int32 lat			# 위도
int32 lon			# 경도
int32 alt			# 평균해수면 고도 (millimetres)
int32 alt_ellipsoid 		# 지구 표준위 고도 (millimetres)

float32 s_variance_m_s		# GPS 속도 정확도 estimate (metres/sec)
float32 c_variance_rad		# GPS 코스 정확도 estimate (radians) 
uint8 fix_type # 0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.   

float32 eph			# GPS 수평 위치 정확도 (metres)
float32 epv			# GPS 수직 위치 정확도 (metres)

float32 hdop			# 수평 정확도
float32 vdop			# 수평 정확도

int32 noise_per_ms		# ms 시간 당 GPS 노이즈
int32 jamming_indicator		# jamming 발생 표시

float32 vel_m_s			# GPS 그라운드 속도 (metres/sec) 
float32 vel_n_m_s		# GPS North 속도 (metres/sec) 
float32 vel_e_m_s		# GPS East 속도 (metres/sec)
float32 vel_d_m_s		# GPS Down 속도 (metres/sec) 
float32 cog_rad			# 지상위 코스(이동 방향) -PI..PI, (radians) 
bool vel_ned_valid		# NED 속도가 유효하면 True

int32 timestamp_time_relative	# timestamp + timestamp_time_relative = 시스템이 시작되고 난 후에 UTC timestamp의 시간 (microseconds)
uint64 time_utc_usec		# GPS 모듈에서 수신한 Timestamp (microseconds, UTC)

uint8 satellites_used		# 위성의 갯수
