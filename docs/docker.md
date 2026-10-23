# Запуск в Docker

Образ содержит всё окружение проекта: ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL
на зафиксированном коммите, `px4_msgs`, Micro-XRCE-DDS Agent и сам проект.
Обычная установка без Docker по-прежнему работает; Docker нужен, чтобы
запустить проект на чистой машине без установки ROS и PX4.

## Требования

Поддерживаются две системы:
- Ubuntu 24.04;
- WSL2 с Ubuntu 24.04.

На других системах `docker/run.sh` останавливается с ошибкой.

Кроме системы нужны Docker с Compose v2, процессор x86-64 и около 15 ГБ места.

| Система | Дополнение | Видеокарта и окно Gazebo |
|---|---|---|
| WSL2, Ubuntu 24.04 | `docker/wsl.yaml` | D3D12 через `/dev/dxg`, окно через WSLg |
| Ubuntu 24.04 с NVIDIA и `nvidia-container-toolkit` | `docker/nvidia.yaml` | драйвер NVIDIA, окно через X11 |
| Ubuntu 24.04 без NVIDIA | — | рендер на CPU, примерно в 5 раз медленнее реального времени |

**WSL.** В Windows нужен драйвер NVIDIA. В Docker Desktop включите WSL
Integration для дистрибутива (Settings → Resources → WSL Integration).
Пользователь WSL должен состоять в группе `docker`. `docker/wsl.yaml`
задаёт переменные OpenGL на весь контейнер:
- `GALLIUM_DRIVER=d3d12`;
- `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`;
- `LD_LIBRARY_PATH=/usr/lib/wsl/lib`.

Поэтому на видеокарте работает всё в контейнере, в том числе команды через
`docker compose exec`.

## Запуск

Запускайте только через `docker/run.sh`. Он проверяет систему, находит
видеокарту и подключает нужное дополнение. Аргументы передаются
`docker compose` без изменений.

```bash
docker/run.sh up sim-gui    # окно Gazebo и панель
docker/run.sh up sim        # без окна
```

`run.sh` записывает выбранный набор файлов в `.env` (`COMPOSE_FILE`),
например `COMPOSE_FILE=compose.yaml:docker/wsl.yaml` в WSL. Поэтому после
первого запуска через `run.sh` обычный `docker compose up sim-gui` тоже
подключает видеокарту. `.env` у каждой машины свой и в git не попадает.
До первого запуска `run.sh` обычный `docker compose` рендерит на CPU.

Панель открывается по адресу `http://localhost:8080` (порт доступен только
с этой машины). Записи полётов, rosbag и отчёты пишутся в `./log`.

Остановка — Ctrl+C или `docker compose stop`. Контейнер получает SIGINT;
если группа в полёте, панель сначала сажает её (до 200 с).

Если образа нет локально, `up` скачивает готовый
`ghcr.io/deomus/swarm-lidar:latest`.

На обычной Ubuntu контейнер работает от пользователя с UID 1000, поэтому
`run.sh` разрешает ему подключаться к X-серверу (`xhost`) для окна Gazebo.

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
- Ubuntu 24.04 с NVIDIA (`docker/nvidia.yaml`) не проверялась.

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
коммита: в образе нет `.git`. `run.sh` подставляет его из git сам.

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
