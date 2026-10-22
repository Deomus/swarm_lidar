#!/usr/bin/env python3
"""Последовательная серия: новый SITL для каждого опыта, остановка после посадки."""

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def snapshot():
    from ament_index_python.packages import get_package_share_directory
    import swarm_lidar

    repository = Path(swarm_lidar.__file__).resolve().parents[3]
    revision = subprocess.check_output(['git', '-C', str(repository), 'rev-parse', 'HEAD'], text=True).strip()
    files = {}
    for package, name in [('formation_controller', 'controller.yaml'),
                          ('leader_interface', 'leader.yaml'), ('lidar_tracker', 'tracker.yaml'),
                          ('mesh_interface', 'mesh.yaml'), ('swarm_manager', 'manager.yaml')]:
        path = Path(get_package_share_directory(package))/'config'/name
        files[f'{package}/{name}'] = path.read_text()
    world = Path(get_package_share_directory('swarm_lidar'))/'worlds/swarm_lidar.sdf'
    geometry = Path(get_package_share_directory('swarm_lidar'))/'config/swarm.yaml'
    if geometry.exists():
        files['swarm_lidar/swarm.yaml'] = geometry.read_text()
    source_hashes = {}
    for name in ('formation_controller.core', 'formation_controller.node',
                 'lidar_tracker.tracking', 'lidar_tracker.tracker_node',
                 'mesh_interface.node', 'leader_interface.core', 'leader_interface.node'):
        path = Path(importlib.util.find_spec(name).origin)
        source_hashes[name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return dict(runtime_revision=revision, parameters=files,
                source_sha256=source_hashes,
                world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                note='Каждый сценарий начинается в новом мире, без GUI и видео.')


def landing_confirmed(path):
    try:
        return json.loads(path.read_text()).get('passed_landing') is True
    except (OSError, ValueError):
        return False


def run_one(root, kind, index, px4):
    output = root/f'{kind}_{index}'
    output.mkdir()
    metadata = snapshot()
    dump(output/'configuration.json', metadata)
    flight = output/'flight'
    simulator = scenario = None
    with (output/'sitl.log').open('x') as sim_log, (output/'scenario.log').open('x') as flight_log:
        try:
            simulator = subprocess.Popen([sys.executable, '-m', 'swarm_lidar.sitl',
                '--px4-dir', str(px4), '--gui', 'false', '--lidar', 'true', '--stack', 'true'],
                stdout=sim_log, stderr=subprocess.STDOUT, start_new_session=True)
            scenario = subprocess.Popen([sys.executable, '-m', 'swarm_lidar.scenario', kind,
                '--output', str(flight), '--record-bag'], stdout=flight_log,
                stderr=subprocess.STDOUT, start_new_session=True)
            dump(root/'current.json', dict(scenario=kind, index=index, output=str(output),
                                         simulator_pid=simulator.pid, scenario_pid=scenario.pid))
            print(f'Начат {kind}, повтор {index}: {output}', flush=True)
            scenario.wait()
        finally:
            if scenario is not None and scenario.poll() is None:
                # Исполнитель сам выполняет посадку в finally. Не убиваем его
                # жёстко и не выключаем мир под вооружёнными аппаратами.
                os.killpg(scenario.pid, signal.SIGINT)
                scenario.wait(timeout=180)
            if simulator is not None and simulator.poll() is None:
                if scenario is not None and not landing_confirmed(flight/'result.json'):
                    raise RuntimeError(f'Посадка не подтверждена. SITL PID {simulator.pid} оставлен: {output}')
                simulator.send_signal(signal.SIGTERM)
                simulator.wait(timeout=45)
        result = json.loads((flight/'result.json').read_text())
        if scenario.returncode != 0 or not result['passed']:
            raise RuntimeError(f'Опыт не пройден; серия остановлена: {output}')
        summary = json.loads((flight/'metrics/summary.json').read_text())['active_control']
        compact = dict(scenario=kind, index=index, source=str(flight),
                       runtime_revision=metadata['runtime_revision'], passed=True,
                       formation=summary['formation_error_m'],
                       localization=summary['localization_error_m'],
                       min_pair_distance_m=summary['min_pair_distance_m']['min'])
        dump(output/'summary.json', compact)
        print(f"Завершён {kind}, повтор {index}: RMS={compact['formation']['rms']:.3f}, "
              f"max={compact['formation']['max']:.3f}; посадка подтверждена", flush=True)
        return compact


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--px4-dir', type=Path, default=Path.home()/'PX4-Autopilot')
    parser.add_argument('--runs', type=int, default=3)
    parser.add_argument('--start-index', type=int, default=1)
    parser.add_argument('--scenarios', nargs='+', choices=('s1', 's2', 's3'), default=['s2'])
    args = parser.parse_args()
    if args.runs < 1 or args.start_index < 1:
        parser.error('Число повторов и начальный номер должны быть положительными')
    args.output = args.output.expanduser().resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    reports = []
    try:
        for kind in args.scenarios:
            for index in range(args.start_index, args.start_index+args.runs):
                reports.append(run_one(args.output, kind, index, args.px4_dir.expanduser().resolve()))
                dump(args.output/'summary.json', reports)
    finally:
        dump(args.output/'summary.json', reports)


if __name__ == '__main__':
    main()
