uint8 INVALID=0
uint8 VALID=1
uint8 DEGRADED=2

# Время, к которому относится прогноз положения.
std_msgs/Header header

uint8 vehicle_id

# Положение и скорость в общей системе ENU.
geometry_msgs/Point position
geometry_msgs/Vector3 velocity

# Матрицы covariance 3x3, записанные по строкам.
float64[9] position_covariance
float64[9] velocity_covariance

uint8 status
float32 quality
uint8 used_station_count

# Возраст последнего принятого набора измерений.
float32 measurement_age_s

uint32 reset_counter