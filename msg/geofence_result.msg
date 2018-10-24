uint8 GF_ACTION_NONE = 0                    # no action on geofence violation
uint8 GF_ACTION_WARN = 1                    # critical mavlink message
uint8 GF_ACTION_LOITER = 2                  # switch to AUTO|LOITER
uint8 GF_ACTION_RTL = 3                     # switch to AUTO|RTL
uint8 GF_ACTION_TERMINATE = 4               # flight termination

bool geofence_violated		# geofence 위반한 경우 true
uint8 geofence_action       	# geofence 위반시 동작

bool home_required		#geofence가 유효한 home 위치가 필요한 경우 true
