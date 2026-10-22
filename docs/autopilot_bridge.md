# Мост ROS 2 — PX4

Четыре экземпляра `bridge_node` работают в namespace `uav_0..uav_3`.
PX4-specific сообщения остаются в этом пакете. Основной конфиг:
`src/autopilot_bridge/config/bridge.yaml`; namespace, ID и известные стартовые
смещения ENU передаются launch. Системный PX4 estimator — источник своей
телеметрии; Gazebo ground truth мост не читает.

## Координаты и время

Позиция/линейная и угловая скорость `SwarmState` выражены в `frame_id=map`
(ENU). Quaternion описывает FLU-корпус относительно ENU. `yaw` — угол от +X
против часовой стрелки, `yaw_rate` — его производная с учётом roll/pitch;
она не приравнивается к body rate r. Начальные XY-смещения известны из world,
это не непрерывная ground truth локализация.

TF: `map → uav_i/base_link → uav_i/lidar_link`. Статическое смещение LiDAR
задаётся YAML (z=0.45 м); на stock x500 без LiDAR это только номинальный frame.

Все узлы управления используют Gazebo `/clock` и `use_sim_time`. Обёртка
старта PX4 после rcS перезапускает uXRCE client с `UXRCE_DDS_SYNCT=0`.
Поэтому `timestamp` SwarmState сохраняет время PX4 odometry (время симуляции),
`last_seen` — время ROS-получения/публикации. Проверяется возраст и odometry,
и статуса, и local position; требуется валидность позиции и скорости.

Откат часов отключает выдачу Offboard setpoint и очищает входные данные.
Для возобновления нужен явный `enable_stream`. Будущие команды не становятся
watermark и не блокируют последующие корректные сообщения.

## Контракты

Входы PX4 (sensor data QoS: best effort, volatile, depth 5):

- `fmu/out/vehicle_odometry`;
- `fmu/out/vehicle_local_position_v1`;
- `fmu/out/vehicle_status_v4`;
- `fmu/out/vehicle_command_ack_v1`.

Выходы PX4: `fmu/in/offboard_control_mode`, `fmu/in/trajectory_setpoint`,
`fmu/in/vehicle_command`, reliable/volatile. Суффиксы зависят от зафиксированной
версии PX4/px4_msgs и доступны как параметры.

Вход команд проекта: `/mesh/commands`, SwarmCommand с target ID и
`frame_id=map`. Используется только target_velocity.linear (м/с), поле pose
не управляет аппаратом. `target_velocity.angular.z` — ENU yaw rate (рад/с),
в PX4 yawspeed знак меняется. Возраст команды ограничен `command_timeout`.
Выходы: `/swarm/states` и `/uav_i/autopilot/state`, SwarmState, reliable,
volatile, depth 10. Частота — 20 Гц по умолчанию. При включённом stream
`/uav_i/setpoint/velocity` (`geometry_msgs/TwistStamped`, map ENU) повторяет
скорость и yaw rate, фактически отправленные PX4, включая нулевую при BRAKE.

При включённом stream без свежей валидной команды выдаётся нулевая скорость,
а не последняя устаревшая команда. Position/acceleration/jerk/yaw задаются
NaN (кроме yawspeed), OffboardControlMode.velocity=true. Скорость ограничена
по норме, yaw rate — параметром max_yaw_rate.
Этот аварийный переход задаёт цель торможения PX4 и пока не гарантирует
ограничение конечной разности ускорения/рывка самой команды.

## Команды и подтверждения

Сервисы Trigger под `/uav_i/`: `arm`, `takeoff`, `enable_stream`, `offboard`,
`hold`, `land`. `success=true` означает отправку запроса, а не завершённый
манёвр. Завершение проверяется по ack, состоянию и высоте. Arm требует
прохождения preflight; forced arm не используется. Перед Offboard нужны
enable_stream и минимум секунда heartbeat. Land/hold отключают stream.

В SITL запускается программная GCS `gcs_heartbeat`, которая посылает настоящий
MAVLink heartbeat на localhost:18570..18573. Она не отключает preflight и
не подменяет состояние PX4. Зависимость — установленный pymavlink.

Проверки в отдельных терминалах после source ROS, px4_ros_ws и install:

```bash
ROS_DOMAIN_ID=72 ros2 launch swarm_lidar swarm_demo.launch.py gui:=false
```

```bash
ROS_DOMAIN_ID=72 python3 scripts/smoke_bridge.py
ROS_DOMAIN_ID=72 python3 scripts/smoke_bridge.py --flight
```

Вариант с `--flight` реально вооружает, последовательно поднимает четыре
SITL, включает Offboard, проверяет движение по ENU-команде и торможение при
пропаже команд, затем последовательно сажает аппараты. Применять только к
этому симуляционному запуску, не к реальным аппаратам.
