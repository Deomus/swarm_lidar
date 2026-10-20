# Запуск четырёх PX4

Требования: собранный PX4 в `~/PX4-Autopilot`, Gazebo Harmonic,
`MicroXRCEAgent`, ROS 2 Jazzy. Сборка PX4 и установка системных пакетов
выполняются отдельно. Каталог PX4 можно передать через `px4_dir`.

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ROS_DOMAIN_ID=72 ros2 launch swarm_lidar swarm_demo.launch.py gui:=false
```

`gui:=true` открывает окно Gazebo. `duration:=60` ограничивает время запуска.
Запуск поднимает world `swarm`, четыре x500 с раздельными рабочими каталогами
и один DDS Agent (UDP 8889). Каждый PX4 получает `PX4_UXRCE_DDS_NS=uav_i`.
Gazebo partition — `swarm_lidar`; ROS_DOMAIN_ID наследуется всеми процессами.
Другой PX4 с тем же instance или agent-портом одновременно запускать нельзя.
При Ctrl+C завершаются только процессы текущего запуска. Логи — `log/sitl_*`.
Ни один аппарат автоматически не вооружается.

Во втором терминале:

```bash
source /opt/ros/jazzy/setup.bash
source ~/px4_ros_ws/install/setup.bash
ROS_DOMAIN_ID=72 python3 scripts/smoke_sitl.py
```

Проверяются минимум пять конечных измерений odometry на каждого БПЛА,
свежесть получения и уникальные system_id 1..4. Результат сохраняется в
`log/sitl_smoke.json`.

Чекпоинт 2026-10-20: проверка прошла для четырёх stock x500; system_id 1..4,
получено соответственно 5, 12, 6, 14 измерений. Реальный LiDAR, TF и
управление строем в этом чекпоинте ещё не подключены. Это инфраструктурный
тест, не полный smoke test приёмки и не сценарии S1/S2/S3.
