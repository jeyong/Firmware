#
# Pre-flight sensor check metrics. These will be zero if the vehicle only has one sensor.
# The topic will not be updated when the vehicle is armed
#
float32 accel_inconsistency_m_s_s  # IMU 인스턴스 간 최대 가속도 차이 (m/s/s)
float32 gyro_inconsistency_rad_s   # IMU 인스턴스 간 최대 각속도 차이 (rad/s)
float32 mag_inconsistency_ga       # Mag 인스턴스 간 최대 차이 (Gauss)
