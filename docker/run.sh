#!/usr/bin/env bash
# docker compose с автоопределением устройства: подключает к compose.yaml
# дополнение для видеокарты. Аргументы передаются docker compose как есть:
#   docker/run.sh up sim-gui
#   docker/run.sh run --rm sim python3 scripts/smoke_test.py
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
files=(-f "$root/compose.yaml")
note() { echo "run.sh: $*" >&2; }

docker info >/dev/null 2>&1 || { note 'Docker недоступен: запустите его или добавьте пользователя в группу docker'; exit 1; }
kernel=$(docker info --format '{{.KernelVersion}}' 2>/dev/null || true)

# Docker Desktop на Windows тоже показывает runtime nvidia, поэтому WSL
# проверяется первым: там видеокарта идёт через D3D12, а не через CUDA-драйвер.
if [[ -e /dev/dxg || $kernel == *microsoft* || $kernel == *WSL* ]]; then
  files+=(-f "$root/docker/wsl.yaml")
  note 'WSL: видеокарта через D3D12 (/dev/dxg), окно через WSLg'
elif [[ $(uname -s) == Linux ]] && command -v nvidia-smi >/dev/null \
    && docker info --format '{{json .Runtimes}}' 2>/dev/null | grep -q '"nvidia"'; then
  files+=(-f "$root/docker/nvidia.yaml")
  note 'Linux: видеокарта NVIDIA через nvidia-container-toolkit'
else
  note 'видеокарта не подключена: Gazebo рендерит на CPU, симуляция примерно в 5 раз медленнее'
  [[ $(uname -s) == Linux ]] && command -v nvidia-smi >/dev/null \
    && note 'для NVIDIA установите nvidia-container-toolkit и перезапустите Docker'
fi

case $(uname -m) in
  x86_64|amd64) ;;
  *) note 'процессор не x86-64: образ работает через эмуляцию, симуляция будет очень медленной' ;;
esac

# Окно Gazebo: на обычном Linux X-сервер пускает только своих пользователей,
# а в контейнере UID 1000; на macOS нужен XQuartz с разрешёнными сетевыми клиентами.
if [[ " $* " == *" sim-gui "* && ${files[*]} != *wsl.yaml* ]]; then
  if [[ $(uname -s) == Darwin ]]; then
    export DISPLAY=host.docker.internal:0
    command -v xhost >/dev/null && xhost +localhost >/dev/null || note 'нужен XQuartz: brew install --cask xquartz'
  elif command -v xhost >/dev/null && user=$(id -nu 1000 2>/dev/null); then
    xhost "+SI:localuser:$user" >/dev/null || note 'xhost не сработал: окно Gazebo может не открыться'
  else
    note 'нет xhost или пользователя с UID 1000: окно Gazebo может не открыться'
  fi
fi

# Ревизия кода для отчётов полётов при сборке образа.
if [[ -z ${SWARM_LIDAR_REVISION:-} ]] && rev=$(git -C "$root" rev-parse HEAD 2>/dev/null); then
  export SWARM_LIDAR_REVISION=$rev
fi
exec docker compose "${files[@]}" "$@"
