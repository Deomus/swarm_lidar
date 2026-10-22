# Команда скорости одному аппарату.
uint8 uav_id
# Система координат команды: map, ENU.
string frame_id
# Не используется: управление только по скорости.
geometry_msgs/Pose target_pose
# linear — м/с в frame_id; angular.z — скорость рыскания, рад/с, против часовой стрелки.
geometry_msgs/Twist target_velocity
# Время исходных данных команды (sim time) для проверки свежести.
builtin_interfaces/Time source_stamp
bool valid
# Состояние источника: FOLLOWING, DATA_HOLD, MANUAL, MISSION, HOLD и т. п.
string safety_state
