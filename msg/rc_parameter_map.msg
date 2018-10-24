uint8 RC_PARAM_MAP_NCHAN = 3 # 매핑 채널의 갯수를 3개로 제한
uint8 PARAM_ID_LEN = 16 

bool[3] valid		# RC-Param 채널이 param으로 매핑되는 경우 true
int32[3] param_index	# 관련 param 인덱스로 이 필드는 -1로 설정된 경우 무시
char[51] param_id	# MAP_NCHAN * (ID_LEN + 1) chars, corresponding param id, null terminated
float32[3] scale		# RC 입력[-1, 1]을 parameter 값으로 스케일링
float32[3] value0		# 초기값
float32[3] value_min	# 최소 parameter 값
float32[3] value_max	# 최대 parameter 값
