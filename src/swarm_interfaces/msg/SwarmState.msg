# Собственная оценка PX4 аппарата, переведённая в ROS ENU/FLU.
uint8 uav_id
# Система координат позы и линейной скорости: map, ENU.
string frame_id
# Курс от оси X (восток) против часовой стрелки, рад; скорость курса, рад/с.
float32 yaw
float32 yaw_rate
# Позиция, м; ориентация base_link FLU относительно map.
geometry_msgs/Pose pose
# linear — м/с в map; angular — рад/с в map.
geometry_msgs/Twist velocity
# Номер nav_state PX4 строкой, например 14 — Offboard.
string mode
string health
# Время публикации мостом (sim time).
builtin_interfaces/Time last_seen
# Время измерения одометрии PX4 (sim time).
builtin_interfaces/Time timestamp
bool armed
bool valid
