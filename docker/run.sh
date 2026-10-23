#!/usr/bin/env bash
# docker compose для Ubuntu 24.04 и WSL2 с Ubuntu 24.04: подключает к
# compose.yaml дополнение для видеокарты. Аргументы передаются docker compose как есть:
#   docker/run.sh up sim-gui
#   docker/run.sh run --rm sim python3 scripts/smoke_test.py
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
files=(-f "$root/compose.yaml")
note() { echo "run.sh: $*" >&2; }

# shellcheck disable=SC1091
[[ -r /etc/os-release ]] && . /etc/os-release
if [[ ${ID:-} != ubuntu || ${VERSION_ID:-} != 24.04 ]]; then
  note "проект проверен только на Ubuntu 24.04 и WSL2 с Ubuntu 24.04, а здесь ${PRETTY_NAME:-$(uname -s)}."
  note 'На этой системе запуск не тестировался: возможны ошибки. Вы запускаете на свой страх и риск,'
  note 'автор не несёт ответственности за возможные сбои.'
  # Без терминала спросить нельзя: согласие задаётся переменной SWARM_LIDAR_ACCEPT_RISK=1.
  if [[ ${SWARM_LIDAR_ACCEPT_RISK:-} != 1 ]]; then
    if [[ ! -t 0 ]]; then
      note 'нет терминала для подтверждения; чтобы всё равно запустить, задайте SWARM_LIDAR_ACCEPT_RISK=1'
      exit 1
    fi
    read -r -p 'run.sh: вы точно хотите продолжить? [y/N] ' answer
    [[ $answer == [yYдД]* ]] || { note 'запуск отменён'; exit 1; }
  fi
fi
docker info >/dev/null 2>&1 || { note 'Docker недоступен: запустите его или добавьте пользователя в группу docker'; exit 1; }

if [[ -e /dev/dxg ]]; then
  # WSL: видеокарта через D3D12; переменные OpenGL задаёт docker/wsl.yaml.
  files+=(-f "$root/docker/wsl.yaml")
  note 'WSL: видеокарта через D3D12 (GALLIUM_DRIVER=d3d12, NVIDIA), окно через WSLg'
elif command -v nvidia-smi >/dev/null && docker info --format '{{json .Runtimes}}' | grep -q '"nvidia"'; then
  files+=(-f "$root/docker/nvidia.yaml")
  note 'Ubuntu: видеокарта NVIDIA через nvidia-container-toolkit'
else
  note 'видеокарта не подключена: Gazebo рендерит на CPU, симуляция примерно в 5 раз медленнее'
  command -v nvidia-smi >/dev/null && note 'для NVIDIA установите nvidia-container-toolkit и перезапустите Docker'
fi

# Окно Gazebo на обычной Ubuntu: X-сервер пускает только своих пользователей,
# а в контейнере UID 1000. В WSL окно идёт через WSLg без разрешений.
if [[ " $* " == *" sim-gui "* && ! -e /dev/dxg ]]; then
  if command -v xhost >/dev/null && user=$(id -nu 1000 2>/dev/null); then
    xhost "+SI:localuser:$user" >/dev/null || note 'xhost не сработал: окно Gazebo может не открыться'
  else
    note 'нет xhost или пользователя с UID 1000: окно Gazebo может не открыться'
  fi
fi

# Тот же набор файлов для обычного `docker compose ...`: Compose читает
# COMPOSE_FILE из .env в корне проекта. .env у каждой машины свой, не в git.
compose_file=$(printf '%s\n' "${files[@]}" | grep -v '^-f$' | sed "s|^$root/||" | paste -sd:)
touch "$root/.env"
if ! grep -qx "COMPOSE_FILE=$compose_file" "$root/.env"; then
  sed -i '/^COMPOSE_FILE=/d' "$root/.env"
  echo "COMPOSE_FILE=$compose_file" >> "$root/.env"
  note "в .env записано COMPOSE_FILE=$compose_file: теперь и обычный docker compose подключает его"
fi

# Ревизия кода для отчётов полётов при сборке образа.
if [[ -z ${SWARM_LIDAR_REVISION:-} ]] && rev=$(git -C "$root" rev-parse HEAD 2>/dev/null); then
  export SWARM_LIDAR_REVISION=$rev
fi
exec docker compose "${files[@]}" "$@"
