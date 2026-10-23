# docker compose с автоопределением устройства для PowerShell на Windows.
# Аргументы передаются docker compose как есть:
#   powershell -ExecutionPolicy Bypass -File docker\run.ps1 up sim-gui
#   powershell -ExecutionPolicy Bypass -File docker\run.ps1 run --rm sim python3 scripts/smoke_test.py
$root = Split-Path -Parent $PSScriptRoot
$files = @('-f', (Join-Path $root 'compose.yaml'))

$kernel = docker info --format '{{.KernelVersion}}' 2>$null
if ($LASTEXITCODE -ne 0) {
    Write-Error 'run.ps1: Docker недоступен: запустите Docker Desktop'
    exit 1
}
if ($kernel -match 'microsoft|WSL') {
    # Docker Desktop на движке WSL2: видеокарта через D3D12, окно через WSLg,
    # который виден демону по этому пути.
    $files += @('-f', (Join-Path $root 'docker\wsl.yaml'))
    $env:WSLG_DIR = '/run/desktop/mnt/host/wslg'
    Write-Host 'run.ps1: WSL2: видеокарта через D3D12, окно через WSLg'
} else {
    Write-Warning 'run.ps1: Docker Desktop не на движке WSL2: видеокарта и окно Gazebo недоступны, рендер на CPU'
}

if (-not $env:SWARM_LIDAR_REVISION -and (Get-Command git -ErrorAction SilentlyContinue)) {
    $rev = git -C $root rev-parse HEAD 2>$null
    if ($LASTEXITCODE -eq 0) { $env:SWARM_LIDAR_REVISION = $rev }
}
docker compose @files @args
exit $LASTEXITCODE
