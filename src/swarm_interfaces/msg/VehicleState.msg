# Время формирования сообщения по часам ROS.
# frame_id — общая система координат, например swarm_map.
std_msgs/Header header

# PX4 system_id. Значение 0 означает неизвестный идентификатор.
uint8 vehicle_id

# Положение в общей ENU, метры:
# x — восток, y — север, z — вверх от общего начала координат.
geometry_msgs/Point position

# Скорость в общей ENU, м/с.
# Компоненты выражены в мировой системе координат.
geometry_msgs/Vector3 velocity

# Поворот из системы самолёта FLU в общую ENU.
# FLU: x вперёд, y влево, z вверх.
# Компоненты quaternion: x, y, z, w.
geometry_msgs/Quaternion orientation

# Высота над средним уровнем моря, метры.
# Имеет отдельный отсчёт от position.z.
float32 altitude_amsl_m

# Значения nav_state из PX4.
uint8 NAV_STATE_UNKNOWN=255
uint8 nav_state 255
uint8 nav_state_user_intention 255

# Интерпретировать только при status_valid=true.
bool armed
bool failsafe

# Достоверность соответствующих групп данных.
# При false значения группы нельзя использовать для управления.
bool position_valid
bool velocity_valid
bool attitude_valid
bool altitude_valid
bool status_valid

# Возраст последнего полученного исходного сообщения, секунды.
# Считается публикатором по steady_clock.
# При отсутствии данных публикатор устанавливает +inf.
float32 position_age_s
float32 velocity_age_s
float32 attitude_age_s
float32 altitude_age_s
float32 status_age_s
