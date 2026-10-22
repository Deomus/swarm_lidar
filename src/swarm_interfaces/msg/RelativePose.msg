# Положение ведомого относительно лидера по LiDAR. Длины в метрах.
uint8 target_id
# Центр видимого кластера в frame_id; orientation — identity, yaw цели не измеряется.
geometry_msgs/Pose pose
# Расстояние до центра кластера, м.
float32 distance
# Эвристика компактности кластера 0..1, не вероятность и не covariance.
float32 confidence
# Время облака LiDAR (sim time), а не время публикации.
builtin_interfaces/Time measurement_stamp
bool valid
# Система датчика: uav_0/lidar_link (FLU датчика, без компенсации наклона).
string frame_id
# Измеряемый аппарат: uav_<target_id>/base_link.
string child_frame_id
