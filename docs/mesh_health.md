# Канал mesh и контроль свежести

`mesh_interface` — программный интерфейс, без моделирования радиофизики.
Он передаёт сообщения без изменения полей и timestamp:

| Вход | Выход | Тип |
|---|---|---|
| `/swarm/relative_poses` | `/mesh/relative_poses` | `RelativePose` |
| `/swarm/states` | `/mesh/states` | `SwarmState` |
| `/swarm/commands` | `/mesh/commands` | `SwarmCommand` |

QoS: reliable/volatile, depth из YAML. На каждую тему своя ограниченная
очередь. Задержка и равномерный jitter задаются в секундах, loss в [0,1].
Переполнение отбрасывает новый пакет. Seed фиксирует воспроизводимость.
Jitter может менять порядок доставки. При возврате ROS-времени назад очередь
очищается. Частота доставки ограничена `period` (по умолчанию 100 Гц).

`swarm_manager` принимает `/mesh/states` и `/mesh/relative_poses`, публикует
`/swarm/health` (`diagnostic_msgs/DiagnosticArray`, reliable, volatile) на
10 Гц. Имена статусов — `uav_0..uav_3`. Проверяются timestamp измерения и
телеметрии, valid и наличие обоих потоков для ведомых. Лидеру LiDAR-поза
самого себя не требуется. Приём старого пакета не обновляет last seen.

Свежие данные дают FOLLOWING, краткий stale — DEGRADED, длительный stale,
invalid, отсутствие данных или timestamp из будущего — HOLD. Здесь FOLLOWING
означает готовность данных, а не подтверждение полёта. Выход пока не является
командой автопилоту. Переходы логируются с причиной и ROS timestamp.

Настройки находятся в `config/mesh.yaml` и `config/manager.yaml` соответствующих
пакетов. Параметры считываются при старте. Все узлы используют одни часы.

Запуск после сборки и source install/setup.bash:

```bash
ros2 run mesh_interface mesh_node
ros2 run swarm_manager manager_node
ros2 run swarm_manager health_check --ros-args -p expected_uavs:=4
```

Каждая команда запускается в отдельном терминале. Без источников состояний
и измерений проверка health должна завершиться ошибкой.

Воспроизводимый тест с синтетическими облаками и телеметрией (без PX4):

```bash
ROS_DOMAIN_ID=71 python3 scripts/smoke_pipeline.py
```

Он проверяет три LiDAR-цели, четыре health status, передачу через mesh и HOLD
после остановки облаков. Это проверка ROS-цепочки, не приёмка SITL/S1/S2/S3.
