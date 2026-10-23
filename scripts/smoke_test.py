#!/usr/bin/env python3
"""Единый smoke test: сам запускает симуляцию, проверяет и останавливает.

Проверки: четыре namespace, TF map → base_link → lidar_link, облако LiDAR,
относительные позы трёх ведомых (валидность — в полётных сценариях: на земле
грунт входит в полосу поиска LiDAR), setpoint-топики и команды, состояния
автопилотов, режим группы, отсутствие критичных ошибок в журналах и
корректное завершение всех процессов запуска. Аппараты не вооружаются.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time

CRITICAL = re.compile(r'Traceback|\[FATAL\]|\[ERROR\]|завершился:|Один из обязательных ROS-узлов завершился')


def descendants(pid):
    """Все потомки процесса по /proc, включая сменившие сессию."""
    children = {}
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                parent = int((entry/'stat').read_text().rsplit(')', 1)[1].split()[1])
            except (OSError, IndexError, ValueError):
                continue
            children.setdefault(parent, []).append(int(entry.name))
    found, stack = set(), [pid]
    while stack:
        for child in children.get(stack.pop(), []):
            if child not in found:
                found.add(child)
                stack.append(child)
    return found


def simulators():
    """PID процессов PX4, агента и Gazebo по первому аргументу, не по подстроке."""
    found = []
    for entry in Path('/proc').iterdir():
        try:
            argv = (entry/'cmdline').read_bytes().split(b'\0') if entry.name.isdigit() else []
        except OSError:
            continue
        name = Path(argv[0].decode(errors='replace')).name if argv and argv[0] else ''
        if name in ('px4', 'MicroXRCEAgent') or (name == 'gz' and argv[1:2] == [b'sim']):
            found.append(int(entry.name))
    return sorted(found)


def alive(pids):
    """Живые (не зомби) процессы из списка."""
    result = []
    for pid in pids:
        try:
            state = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
        except (OSError, IndexError):
            continue
        if state != 'Z':
            result.append(pid)
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--px4-dir', default=os.environ.get('PX4_DIR', str(Path.home()/'PX4-Autopilot')))
    parser.add_argument('--timeout', type=float, default=150., help='Ожидание готовности, с')
    parser.add_argument('--output', type=Path, default=Path('log')/time.strftime('smoke_%Y%m%d_%H%M%S'))
    args = parser.parse_args()
    busy = simulators()
    if busy:
        raise SystemExit(f'Уже работает другая симуляция {busy}: остановите её, порт и GZ_PARTITION общие')
    args.output.mkdir(parents=True, exist_ok=False)
    checks, launch = {}, None
    launch_log = (args.output/'launch.log').open('w')
    started = time.monotonic()

    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from swarm_interfaces.msg import RelativePose, SwarmCommand, SwarmState
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('smoke_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
    seen = dict(states={}, poses={}, commands=set(), cloud=0, mode=None, health=None)
    for i in range(4):
        node.create_subscription(SwarmState, f'/uav_{i}/autopilot/state',
                                 lambda m, i=i: seen['states'].update({i: m}), 10)
    node.create_subscription(PointCloud2, '/uav_0/lidar/points',
                             lambda m: seen.update(cloud=seen['cloud']+1), qos_profile_sensor_data)
    node.create_subscription(RelativePose, '/swarm/relative_poses',
                             lambda m: seen['poses'].update({m.target_id: m}), 10)
    node.create_subscription(SwarmCommand, '/mesh/commands', lambda m: seen['commands'].add(m.uav_id), 10)
    node.create_subscription(String, '/swarm/mode', lambda m: seen.update(mode=json.loads(m.data)), 10)
    node.create_subscription(DiagnosticArray, '/swarm/health', lambda m: seen.update(health=m), 10)
    buffer = Buffer(node=node)
    TransformListener(buffer, node)

    def check(name, predicate, timeout, detail=lambda: ''):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if launch.poll() is not None:
                break
            if predicate():
                checks[name] = dict(passed=True, seconds=round(time.monotonic()-started, 1))
                print(f'OK: {name}', flush=True)
                return True
        checks[name] = dict(passed=False, detail=str(detail()))
        print(f'FAIL: {name} {checks[name]["detail"]}', flush=True)
        return False

    try:
        launch = subprocess.Popen(['ros2', 'launch', 'swarm_lidar', 'swarm_demo.launch.py', 'gui:=false',
                                   f'px4_dir:={args.px4_dir}'], stdout=launch_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        names = lambda: {f'{ns}/{n}'.replace('//', '/') for n, ns in node.get_node_names_and_namespaces()}
        check('четыре namespace uav_0..uav_3 с мостами PX4',
              lambda: all(f'/uav_{i}/autopilot_bridge' in names() for i in range(4)), args.timeout,
              lambda: sorted(n for n in names() if n.startswith('/uav_')))
        check('состояния четырёх автопилотов свежие и валидные',
              lambda: all(i in seen['states'] and seen['states'][i].valid for i in range(4)), args.timeout,
              lambda: {i: s.health for i, s in seen['states'].items()})
        check('TF map → uav_i/base_link → uav_i/lidar_link',
              lambda: all(buffer.can_transform('map', f'uav_{i}/lidar_link', rclpy.time.Time()) for i in range(4)),
              30.)
        check('облако LiDAR /uav_0/lidar/points', lambda: seen['cloud'] >= 5, 60., lambda: seen['cloud'])
        # На земле датчик ~2,1 м над грунтом: земля входит в полосу поиска
        # ведомых и превышает бюджет точек, поэтому валидность здесь не
        # требуется — её проверяют полётные сценарии после взлёта.
        now = lambda: node.get_clock().now().nanoseconds * 1e-9
        check('относительные позы трёх ведомых: frame, child_frame, свежая отметка',
              lambda: set(seen['poses']) >= {1, 2, 3} and all(
                  m.frame_id == 'uav_0/lidar_link' and m.child_frame_id == f'uav_{i}/base_link'
                  and abs(now() - m.measurement_stamp.sec - m.measurement_stamp.nanosec * 1e-9) < 1.
                  for i, m in seen['poses'].items()), 60.,
              lambda: {i: (m.valid, m.frame_id) for i, m in seen['poses'].items()})
        check('команды /mesh/commands для четырёх аппаратов', lambda: seen['commands'] >= {0, 1, 2, 3}, 30.,
              lambda: sorted(seen['commands']))
        check('setpoint-топики объявлены',
              lambda: all(node.count_publishers(f'/uav_{i}/setpoint/velocity') and
                          node.count_publishers(f'/uav_{i}/fmu/in/trajectory_setpoint') for i in range(4))
              and all(node.count_publishers(f'/uav_{i}/setpoint/pose') for i in range(4)), 30.)
        check('режим группы DISARMED и health по четырём аппаратам',
              lambda: (seen['mode'] or {}).get('mode') == 'DISARMED' and seen['health'] is not None
              and {f'uav_{i}' for i in range(4)} <= {s.name for s in seen['health'].status}, 30.,
              lambda: seen['mode'])
    finally:
        pids = descendants(launch.pid) | {launch.pid} if launch else set()
        if launch is not None and launch.poll() is None:
            launch.send_signal(signal.SIGINT)
        try:
            exit_code = launch.wait(timeout=90) if launch else None
        except subprocess.TimeoutExpired:
            exit_code = None
        deadline = time.monotonic() + 20
        while alive(pids) and time.monotonic() < deadline:
            time.sleep(.5)
        leftover = alive(pids)
        checks['корректное завершение всех процессов'] = dict(
            passed=launch is not None and exit_code is not None and not leftover,
            detail=f'код {exit_code}, осталось {leftover}')
        for pid in leftover:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        launch_log.close()
        text = (args.output/'launch.log').read_text(errors='replace')
        runtime = re.findall(r'Логи: (\S+)', text)
        logs = [args.output/'launch.log'] + (sorted(Path(runtime[-1]).rglob('*.log')) if runtime else [])
        errors = [f'{log.name}: {line.strip()[:200]}' for log in logs if not log.name.startswith('px4_')
                  for line in log.read_text(errors='replace').splitlines() if CRITICAL.search(line)]
        checks['нет критичных ошибок в журналах ROS и запуска'] = dict(passed=not errors, detail=errors[:20])
        node.destroy_node()
        rclpy.shutdown()
        passed = all(c['passed'] for c in checks.values())
        (args.output/'smoke.json').write_text(json.dumps(dict(passed=passed, checks=checks,
            runtime_logs=runtime[-1] if runtime else None), ensure_ascii=False, indent=2)+'\n')
        for name, result in checks.items():
            print(('OK  ' if result['passed'] else 'FAIL') + f' {name}' +
                  ('' if result['passed'] else f': {result.get("detail")}'), flush=True)
        print(f'Smoke test {"пройден" if passed else "НЕ пройден"}: {args.output/"smoke.json"}', flush=True)
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()
