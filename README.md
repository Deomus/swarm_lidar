# swarm_lidar

Проверка окружения и сборка (из корня репозитория):

```bash
source /opt/ros/jazzy/setup.bash
ros2 --help
gz sim --version
colcon build --symlink-install
source install/setup.bash
python3 -m pytest src -q
```

`ros2 --version` не поддерживается CLI Jazzy. Проверенные версии: Ubuntu
24.04, ROS 2 Jazzy, Gazebo Sim 8.15.0. Исходники PX4 находятся отдельно от
этого репозитория; версия на текущей машине — `878877ad02` (ветка main).

Симуляционная система ROS 2 + PX4 SITL + Gazebo для роя из четырёх БПЛА по
схеме leader-follower.

## Текущий статус

Создан первый пакет `swarm_interfaces`. Он задаёт контракты сообщений для
трекера, mesh-интерфейса, менеджера роя, контроллера формации и оценки.

Также добавлен `lidar_tracker`. В текущем MVP он принимает `PointCloud2`,
фильтрует точки по дальности и высоте, выполняет евклидову кластеризацию и
публикует валидные или невалидные `RelativePose` для `uav_1..uav_3`.

В прикладных сообщениях используется система координат ROS/Gazebo ENU.
Относительная поза измеряется от `uav_0/lidar_link` (или явно указанного frame
лидера) к frame ведомого. Ground truth разрешён только для оценки и не входит
в управляющий контур.

## Сборка

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select swarm_interfaces
source install/setup.bash
ros2 interface show swarm_interfaces/msg/RelativePose
```

Запуск трекера:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run lidar_tracker tracker_node
```

Запуск с установленной конфигурацией:

```bash
ros2 run lidar_tracker tracker_node --ros-args --params-file \
  install/lidar_tracker/share/lidar_tracker/config/tracker.yaml
```

Контракт и ограничения трекера описаны в [docs/tracker.md](docs/tracker.md).

Код проекта создан с помощью AI (Codex); используются ROS 2, PX4, Gazebo,
NumPy и SciPy. Названия внешних пакетов и команды сохраняются на английском.

По умолчанию входной topic — `/uav_0/lidar/points`, выходной —
`/swarm/relative_poses`. Параметры диапазона, кластеризации и confidence
задаются через ROS 2 parameters.
