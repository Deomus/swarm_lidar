# Запуск в Docker

Образ содержит всё окружение проекта: ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL
на зафиксированном коммите, `px4_msgs`, Micro-XRCE-DDS Agent и сам проект.
Обычная установка без Docker по-прежнему работает; Docker нужен, чтобы
запустить проект на чистой машине без установки ROS и PX4.

## Требования

- Docker с Compose v2 (Docker Desktop или Docker Engine), около 15 ГБ места.
- Процессор x86-64. На ARM (Apple Silicon и др.) образ работает через
  эмуляцию, и симуляция очень медленная.

Видеокарта зависит от устройства, её подключает отдельное дополнение
к `compose.yaml`:

| Устройство | Дополнение | Видеокарта и окно Gazebo |
|---|---|---|
| WSL2 (Ubuntu в Windows) | `docker/wsl.yaml` | D3D12 через `/dev/dxg`, окно через WSLg |
| Windows, PowerShell, Docker Desktop на WSL2 | `docker/wsl.yaml` | то же |
| Linux с NVIDIA и `nvidia-container-toolkit` | `docker/nvidia.yaml` | драйвер NVIDIA, окно через X11 |
| Остальное (Linux без NVIDIA, macOS) | — | рендер на CPU, примерно в 5 раз медленнее реального времени |

Для WSL нужен драйвер NVIDIA в Windows; в Docker Desktop должна быть
включена WSL Integration для дистрибутива (Settings → Resources → WSL
Integration), а пользователь WSL должен состоять в группе `docker`.
`sitl.py` сам включает рендер D3D12 на NVIDIA, если видит `/dev/dxg`.

## Запуск

Скрипты сами определяют устройство и подключают нужное дополнение.
Аргументы передаются `docker compose` без изменений.

WSL, Linux, macOS:

```bash
docker/run.sh up sim-gui    # окно Gazebo и панель
docker/run.sh up sim        # без окна
```

PowerShell на Windows:

```powershell
powershell -ExecutionPolicy Bypass -File docker\run.ps1 up sim-gui
```

Без скриптов файлы перечисляются вручную, например
`docker compose -f compose.yaml -f docker/wsl.yaml up sim-gui`. Простой
`docker compose up sim` работает где угодно, но рендерит на CPU.

Панель открывается по адресу `http://localhost:8080` (порт доступен только
с этой машины). Записи полётов, rosbag и отчёты пишутся в `./log`.

Остановка — Ctrl+C или `docker compose stop`. Контейнер получает SIGINT;
если группа в полёте, панель сначала сажает её (до 200 с).

Если образа нет локально, `up` скачивает готовый
`ghcr.io/deomus/swarm-lidar:latest`.

Окно Gazebo на обычном Linux: `run.sh` разрешает X-серверу подключения
пользователя с UID 1000 (`xhost`), потому что контейнер работает от него.
На macOS нужен XQuartz с разрешёнными сетевыми клиентами.

## Проверки и сценарии в контейнере

```bash
docker/run.sh run --rm sim python3 scripts/smoke_test.py
docker/run.sh run --rm sim python3 scripts/repeat_scenarios.py \
  --output log/docker_series --runs 1 --scenarios s1 s2 s3 s3_pillars
```

Перед этим на машине не должно быть другой запущенной симуляции: общие
порт агента и партиция Gazebo.

Проверено 23 октября:

- WSL через `run.sh`: OpenGL `D3D12 (NVIDIA GeForce RTX 3080)`, smoke test
  пройден, окно Gazebo через WSLg и панель с хоста работают. С теми же
  настройками видеокарты S1 (RMS ошибки строя 0,097 м) и S2 (0,177 м)
  пройдены с посадкой.
- Без дополнений: рендер `llvmpipe`, smoke test пройден.
- PowerShell через `run.ps1`: D3D12 в контейнере, X11 через WSLg, `./log`
  подключается.
- Linux с NVIDIA (`docker/nvidia.yaml`) и macOS не проверялись.

## Сборка

```bash
docker/run.sh build sim
```

Слои: `base` (ROS, Gazebo) → `px4` (PX4 SITL) → `deps` (`px4_msgs`, агент) →
`app` (проект). Правка проекта пересобирает только `app`, около минуты.
Первая сборка — 40–60 минут; при наличии образа в реестре неизменённые
слои берутся из него (`cache_from`). Версии PX4, `px4_msgs` и агента —
аргументы в начале `docker/Dockerfile`; PX4 и `px4_msgs` меняются только
вместе. `SWARM_LIDAR_REVISION` попадает в отчёты полётов вместо номера
коммита: в образе нет `.git`. `run.sh` и `run.ps1` подставляют его из git
сами.

## Публикация образа

Один раз — токен GitHub с правом `write:packages`:

```bash
echo <ТОКЕН> | docker login ghcr.io -u deomus --password-stdin
```

После сборки и проверки:

```bash
docker tag ghcr.io/deomus/swarm-lidar:latest ghcr.io/deomus/swarm-lidar:<версия>
docker push ghcr.io/deomus/swarm-lidar:<версия>
docker push ghcr.io/deomus/swarm-lidar:latest
```

Затем в настройках пакета на GitHub сделать его публичным. Метка
`org.opencontainers.image.source` в образе привязывает пакет к репозиторию.
