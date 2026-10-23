# Запуск в Docker

Образ содержит всё окружение проекта: ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL
на зафиксированном коммите, `px4_msgs`, Micro-XRCE-DDS Agent и сам проект.
Обычная установка без Docker по-прежнему работает; Docker нужен, чтобы
запустить проект на чистой машине без установки ROS и PX4.

## Требования

- Windows 10/11 с WSL2 и драйвером NVIDIA; Docker Desktop с включённой
  WSL Integration для дистрибутива (Settings → Resources → WSL Integration);
  пользователь WSL в группе `docker`.
- Около 15 ГБ свободного места.

Видеокарта пробрасывается из WSL (`/dev/dxg`, `/usr/lib/wsl`); `sitl.py`
сам включает рендер D3D12 на NVIDIA. Без видеокарты Gazebo рендерит на
процессоре, и симуляция идёт примерно в 5 раз медленнее реального времени.
Проброс в `compose.yaml` рассчитан на WSL; на обычном Linux нужен
`nvidia-container-toolkit` и свои настройки устройств.

## Запуск

Из корня репозитория:

```bash
docker compose up sim-gui    # окно Gazebo через WSLg и панель
docker compose up sim        # без окна
```

Панель — `http://localhost:8080` (порт открыт только на этой машине).
Записи полётов, rosbag и отчёты пишутся в `./log` и принадлежат
пользователю WSL. Остановка — Ctrl+C или `docker compose stop`: контейнер
получает SIGINT, и при активном полёте панель сначала сажает группу
(до 200 с).

Если образа нет локально, `docker compose up` скачивает готовый
`ghcr.io/deomus/swarm-lidar:latest`.

## Проверки и сценарии в контейнере

```bash
docker compose run --rm sim python3 scripts/smoke_test.py
docker compose run --rm sim python3 scripts/repeat_scenarios.py \
  --output log/docker_series --runs 1 --scenarios s1 s2 s3 s3_pillars
```

Перед этим на машине не должно быть другой запущенной симуляции: общие
порт агента и партиция Gazebo.

Проверено 23 октября: OpenGL в контейнере — `D3D12 (NVIDIA GeForce RTX 3080)`;
smoke test пройден; S1 (RMS ошибки строя 0,097 м) и S2 (0,177 м) пройдены
с посадкой; окно Gazebo через WSLg и панель с хоста работают.

## Сборка

```bash
SWARM_LIDAR_REVISION=$(git rev-parse HEAD) docker compose build sim
```

Слои: `base` (ROS, Gazebo) → `px4` (PX4 SITL) → `deps` (`px4_msgs`, агент) →
`app` (проект). Правка проекта пересобирает только `app`, около минуты.
Первая сборка — 40–60 минут; при наличии образа в реестре неизменённые
слои берутся из него (`cache_from`). Версии PX4, `px4_msgs` и агента —
аргументы в начале `docker/Dockerfile`; PX4 и `px4_msgs` меняются только
вместе. `SWARM_LIDAR_REVISION` попадает в отчёты полётов вместо номера
коммита: в образе нет `.git`.

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
