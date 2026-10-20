# Интерфейс лидера

`leader_interface` управляет только `uav_0`. Положение ведомых, ground truth,
arm, takeoff, переключение PX4 в Offboard и посадка в пакет не входят.
Ведомые сопровождают лидера своим LiDAR-контуром. Источник собственной позиции
лидера — его оценка PX4 в `SwarmState`, разрешённая для навигации лидера.

## Контракты ROS

Все топики используют depth=10, reliable/volatile. Рабочие часы — Gazebo `/clock`,
в YAML включён `use_sim_time`. Метры, м/с, рад/с; координаты `map` ENU.

| Направление | Топик | Тип и назначение |
|---|---|---|
| вход | `/mesh/states` | `swarm_interfaces/SwarmState`, только uav_id=0; собственная позиция, valid, timestamp, yaw/yaw_rate |
| вход | `/leader/cmd_vel` | `geometry_msgs/TwistStamped`, линейная скорость ENU, angular.z — производная Euler yaw ENU |
| вход | `/mission/path` | `nav_msgs/Path`, XYZ waypoints в map |
| выход | `/swarm/commands` | `swarm_interfaces/SwarmCommand`, всегда uav_id=0, frame_id=map |
| выход | `/mission/execution_status` | `std_msgs/String`, JSON mode/status/waypoint/total/valid/stamp |

Сервисы `std_srvs/Trigger`: `/leader/hold`, `/leader/manual`, `/leader/mission`.
При запуске режим HOLD. MANUAL/MISSION требуют свежего собственного состояния;
MISSION также требует непустого конечного маршрута. Получение нового Path лишь
сохраняет маршрут: исполняться он начинает после явного `/leader/mission`.
Новый Path не изменяет маршрут уже исполняемой миссии.

## Ручной режим

Проверяются frame_id и исходный header.stamp. Future/out-of-order/stale пакеты
отклоняются и не изменяют watermark. Свежесть команд оценивается по исходной
метке времени, а не времени получения через mesh. После manual_timeout=0.25 с
без свежей команды публикуются valid=false, нулевая скорость и статус DEADMAN.
Следующая свежая команда разрешает движение в MANUAL; потеря собственного
состояния переводит в HOLD и требует повторного явного выбора режима.

angular.x/y должны быть нулевыми. angular.z ограничена ±max_yaw_rate=0.3 рад/с;
положительный знак означает поворот против часовой стрелки в ENU. Bridge
передаёт PX4 yawspeed с обратным знаком для NED. Этот пакет знак не меняет.
В MISSION целевая скорость yaw равна нулю; автоматического наведения yaw на
направление маршрута пока нет.

## Маршрут и ограничения

Лидер пропускает достигнутые waypoints в пределах waypoint_tolerance=0.4 м,
летит к текущему waypoint с P-регулятором waypoint_gain=0.5 и ограниченной
нормой скорости. По достижении последнего: HOLD, valid=false, нулевая скорость,
статус COMPLETE. Превышение waypoint_timeout=90 с: HOLD/WAYPOINT_TIMEOUT.
При stale собственного состояния — HOLD/STATE_STALE. Откат `/clock` очищает
входные watermark и фильтр, включает HOLD/CLOCK_RESET; движение требует нового
валидного состояния и повторного вызова сервиса режима.

Ограничитель `VelocityLimiter` из `formation_controller` даёт max_speed=1.5 м/с,
max_acceleration=1 м/с², max_jerk=2 м/с³ при фиксированном period=0.05 с
в штатной непрерывной выдаче valid-команд. При HOLD/deadman/complete выдаётся
явное запрещение движения valid=false с нулевой скоростью; это safety override,
для которого ограничения производных команд не заявляются. Физическое
торможение и подтверждение остановки выполняются ниже в bridge/PX4.
Консервативная фильтрация добавляет задержку, поэтому отсутствие столкновений
или точность полёта не следуют из этих unit-тестов.

Это исполнитель готовых waypoints, без obstacle avoidance и перепланирования.
Planner обязан предоставить безопасный маршрут и высоту с учётом всего строя.
Path не имеет срока годности: он хранится до явного запуска следующей миссии.
Пригодность маршрута к текущей карте проверяет планировщик/сценарий.

## Запуск и проверка

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run leader_interface leader_node --ros-args --params-file \
  install/leader_interface/share/leader_interface/config/leader.yaml
```

При работающей телеметрии, уже взлетевшем лидере и включённом Offboard:

```bash
ros2 service call /leader/manual std_srvs/srv/Trigger '{}'
ros2 service call /leader/hold std_srvs/srv/Trigger '{}'
```

Manual publisher обязан регулярно проставлять текущий ROS timestamp; команды
с нулевой меткой при работающем симуляторе не считаются свежими.
`source_stamp` valid-команды — минимум исходной метки manual и состояния;
в MISSION — метка состояния. Relay не освежает исходную метку.

Параметры и реальные имена топиков находятся в `config/leader.yaml`.
Проверка ядра: `python3 -m pytest src/leader_interface/test -q`.
Полётный S2/S3 и интеграция с PX4 проверяются внешними smoke/scenario-скриптами.
