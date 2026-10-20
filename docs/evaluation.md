# Оценка экспериментов

Пакет `evaluation` изолирует ground truth: он читает эталонные позы только
для CSV, JSON и графиков. Публикаций в управляющий контур и команд автопилоту
нет. Синтетические тесты не являются результатом полётного эксперимента.

Входы `recorder_node`:

| Тема | Тип | Назначение |
|---|---|---|
| `/evaluation/ground_truth` | `tf2_msgs/msg/TFMessage` | Позы моделей из Gazebo Pose_V bridge |
| `/swarm/relative_poses` | `swarm_interfaces/msg/RelativePose` | Измерения трекера до mesh |
| `/swarm/target_offsets` | `geometry_msgs/msg/PoseArray` | Три offsets в порядке uav_1..3 |
| `/mesh/states` | `swarm_interfaces/msg/SwarmState` | Свежесть телеметрии лидера |

Для ground truth подходят имена моделей `uav_0..uav_3` и scoped names,
заканчивающиеся на имя модели, например `swarm::uav_1`. Вложенные link frames
вроде `uav_1/base_link` игнорируются, поскольку их transform может быть локальным.
При наличии direct и scoped дублей приоритет имеет direct model name.
Transform должен содержать world-позу модели и timestamp симуляции.
Обычный `ros_gz_bridge` для `/world/swarm/pose/info` здесь не используется:
SceneBroadcaster заполняет `Pose_V.header.stamp` и `pose.name`, но не
индивидуальные TF headers. Проверенный стандартный конвертер выдавал пустые
frame names и stamp=0, поэтому такие данные не дают корректной оценки.

Вместо него `ground_truth_node` внутри пакета `evaluation` подписывается
на Gazebo Transport `/world/swarm/pose/info`. Он сохраняет верхний timestamp,
фильтрует только `pose.name=uav_0..uav_3` и публикует четыре world transforms
в `/evaluation/ground_truth` с parent `map`. Индексы массива и порядок
моделей не используются. Нужны Python bindings `gz.transport13`, `gz.msgs10`
и тот же `GZ_PARTITION`, что у симуляции. Ground truth не выходит из тем
evaluation и не публикуется в общий `/tf`.

```bash
GZ_PARTITION=swarm_lidar ros2 run evaluation ground_truth_node
```

Reader публикует позы **моделей** Gazebo. Recorder переводит их в world-позы
`base_link` как `pbase = pmodel + Rmodel * (0,0,model_base_z)`; baseline
`model_base_z=0.24` м соответствует текущему x500. Полный поворот учитывается
до вычисления локализации, формации и minpair. Для иной модели этот параметр
нужно изменить. Reader сохраняет исходные model-root позы без коррекции.

`localization_error_m` — евклидова ошибка измерения трекера относительно
центра сферической мишени в frame LiDAR. Эталон рассчитывается как
`Rleader.T * (ptarget + Rtarget * marker_offset - pleader) - sensor_offset`.
В текущем baseline sensor offset равен `(0, 0, 1.9)` м, marker offset —
`(0, 0, 0.45)` м, оба относительно `base_link`. Учитываются полные roll,
pitch и yaw моделей. Так как трекер возвращает центр видимой поверхности,
метрика включает систематическое смещение поверхности относительно центра.

`formation_error_m` — евклидова ошибка истинного взаимного расположения
`base_link` ведомого и лидера относительно заданного offset. Используется
heading FLU лидера: поворот только по yaw, Z вверх. PoseArray должен иметь
`header.frame_id=uav_0/heading`, ровно три позы и свежий timestamp.

`min_pair_distance_m` — минимальное расстояние между центрами `base_link`
всех четырёх аппаратов по шести парам. Это не расстояние между поверхностями
и не доказательство отсутствия столкновений: учитывать габариты отдельно.

Запись выполняется периодически по ROS time: три строки CSV на один срез.
Локализация согласуется с GT по `measurement_stamp`, а не по времени получения.
Выбирается ближайший снимок в пределах `match_tolerance`; интерполяции нет.
В CSV сохраняется `gt_match_delta_s`, поэтому временную ошибку можно увидеть.
Формация и minpair оцениваются по ближайшему GT к времени среза.
Частота GT и допускаемая скорость движения должны оправдывать выбранный
допуск: 0.15 с по умолчанию не является гарантией требуемой точности.

`relative_stale` объединяет отсутствие, invalid, NaN, неверный frame и устаревшее
измерение; будущие source timestamps не используются. Доли stale/GT missing/
formation missing рассчитываются по числу периодических строк, не по числу
пришедших сообщений. Валидное измерение может использоваться повторно до
timeout: статистика характеризует доступность и ошибку в срезах времени.
RMS/mean/max вычисляются только для конечных ошибок; пропуски записываются
как NaN в CSV и не превращаются в нулевую ошибку. Отдельные доли пропусков
нужны для интерпретации RMS. `localization_missing` дополнительно учитывает
отсутствие GT, согласованного с временем измерения. Пауза sim-time не увеличивает время эксперимента;
при обратном скачке времени история согласования очищается.

Запуск после сборки и подключения окружения:

```bash
ros2 run evaluation recorder_node --output log/evaluation_S1 --ros-args --params-file src/evaluation/config/evaluation.yaml
```

`samples.csv` создаётся эксклюзивно: существующая запись не перетирается.
При штатном завершении через Ctrl+C создаётся `summary.json`: общие и
поканальные RMS/mean/max, minimum minpair и доли пропусков. При принудительном
kill CSV остаётся доступным, а JSON можно восстановить вместе с графиками:

```bash
ros2 run evaluation plot_metrics log/evaluation_S1/samples.csv --output log/evaluation_S1/plots
```

Результат — `metrics.png` и восстановленный `summary.json`. Пакет использует
Matplotlib с backend Agg и не требует окна. Порог допустимой ошибки задаётся
отдельной процедурой приёмки; наличие графика само по себе не означает её прохождение.

```bash
PYTHONPATH=src/evaluation python3 -m pytest src/evaluation/test -q
```

Ограничения текущего этапа: нет автоматической нарезки S1/S2/S3, измерения
route delay и времени реакции на команды, регистрации столкновений по
контактам, автоматической записи rosbag или синхронизации видео. История
согласования ограничена, но строки итоговой записи хранятся в памяти до
завершения: для многочасовых экспериментов нужен потоковый аккумулятор.
