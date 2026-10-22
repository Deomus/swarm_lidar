#!/usr/bin/env python3
"""Повторная обработка облака MCAP новым трекером без Gazebo и PX4."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml


CLOUD = '/uav_0/lidar/points'
TRUTH = '/evaluation/ground_truth'
# Только входы трекера и пассивной оценки. Старые оценки и команды исключены.
REPLAY_TOPICS = (CLOUD, TRUTH, '/swarm/states', '/mesh/states', '/swarm/target_offsets',
                 '/swarm/formation_status', '/mission/path', '/mission/execution_status')
REQUIRED_TYPES = {CLOUD: 'sensor_msgs/msg/PointCloud2', TRUTH: 'tf2_msgs/msg/TFMessage'}


def inspect_bag(path):
    """Метаданные нужны до создания ROS-узлов и выходного каталога."""
    with (path / 'metadata.yaml').open() as handle:
        info = yaml.safe_load(handle)['rosbag2_bagfile_information']
    if info['storage_identifier'] != 'mcap':
        raise ValueError('Поддерживается завершённая запись MCAP')
    topics = {row['topic_metadata']['name']: row for row in info['topics_with_message_count']}
    for name, expected in REQUIRED_TYPES.items():
        row = topics.get(name)
        if (row is None or row['message_count'] <= 0
                or row['topic_metadata']['type'] != expected):
            raise ValueError(f'Нет непустой темы нужного типа: {name} ({expected})')
    start = info['starting_time']['nanoseconds_since_epoch'] * 1e-9
    end = start + info['duration']['nanoseconds'] * 1e-9
    if not all(math.isfinite(value) for value in (start, end)) or end <= start:
        raise ValueError('Некорректные границы времени записи')
    return start, end, [name for name in REPLAY_TOPICS if name in topics]


def validate_window(start, end, bag_start, bag_end, rate, domain, current_domain):
    if not all(math.isfinite(value) for value in (start, end, rate)):
        raise ValueError('Время и скорость должны быть конечными числами')
    if not bag_start <= start < end <= bag_end + 1e-6:
        raise ValueError(f'Нужен участок внутри записи [{bag_start:.6f}, {bag_end:.6f}]')
    if not 0 < rate <= 1:
        raise ValueError('Допустимая скорость повтора: 0 < rate <= 1')
    if not 0 <= domain <= 101 or domain in (72, current_domain):
        raise ValueError('Выберите отдельный ROS_DOMAIN_ID от 0 до 101, отличный от текущего и 72')


def play_command(bag, start, end, bag_start, rate, topics, qos_path=None):
    if any(topic not in REPLAY_TOPICS for topic in topics):
        raise ValueError('Тема вне разрешённого списка повтора')
    command = ['ros2', 'bag', 'play', str(bag), '--storage', 'mcap',
               '--clock', '100', '--rate', str(rate), '--start-offset', str(start - bag_start),
               '--playback-duration', str(end - start), '--delay', '1',
               '--disable-keyboard-controls', '--wait-for-all-acked', '2000']
    if qos_path is not None:
        command += ['--qos-profile-overrides-path', str(qos_path)]
    return command + ['--topics', *topics]


def reliable_tracker_worker(ros_args):
    """Тот же алгоритм, с надёжной доставкой облака только при повторе."""
    import rclpy
    from lidar_tracker.tracker_node import LidarTracker
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import PointCloud2

    rclpy.init(args=ros_args)
    node = LidarTracker()
    diagnostics = {'input_clouds': 0, 'unique_input_clouds': 0, 'rejected_clouds': {},
                   'invalid_target_results': {}, 'valid_target_results': 0,
                   'note': 'Число входов считается внутри единственной подписки трекера'}
    seen = set()
    current = {'reason': None, 'active': False, 'centers': None}
    original_publish = node.publish
    original_update = node.tracker.update

    def update(centers, stamp):
        current['centers'] = len(centers)
        return original_update(centers, stamp)

    node.tracker.update = update

    def publish(stamp, results=None, confidences=None):
        valid = sum(item is not None for item in results) if results is not None else 0
        diagnostics['valid_target_results'] += valid
        missing = len(node.ids) - valid
        if missing:
            reason = current['reason']
            if not current['active']:
                reason = 'stale_timer'
            elif reason is None:
                reason = (node.rejection_reason or ('no_candidate_clusters' if current['centers'] == 0
                          else 'association_missing' if current['centers'] is not None
                          else 'pointcloud_decode_rejected'))
            counts = diagnostics['invalid_target_results']
            counts[reason] = counts.get(reason, 0) + missing
        original_publish(stamp, results, confidences)

    node.publish = publish

    def on_cloud(cloud):
        diagnostics['input_clouds'] += 1
        stamp = (cloud.header.stamp.sec, cloud.header.stamp.nanosec)
        seen.add(stamp)
        current.update(active=True, reason=None, centers=None)
        seconds = stamp[0] + stamp[1] * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - seconds
        last = node.last_stamp
        previous = None if last is None else last.sec + last.nanosec * 1e-9
        if cloud.header.frame_id != node.params['expected_frame']:
            current['reason'] = 'wrong_frame'
        elif age < -.1:
            current['reason'] = 'future_stamp'
        elif age > node.params['stale_timeout']:
            current['reason'] = 'stale_input'
        elif previous is not None and seconds <= previous:
            current['reason'] = 'nonmonotonic_stamp'
        if current['reason'] is not None:
            counts = diagnostics['rejected_clouds']
            reason = current['reason']
            counts[reason] = counts.get(reason, 0) + 1
        try:
            node.pointcloud_callback(cloud)
        finally:
            current['active'] = False

    node.destroy_subscription(node.subscription)
    node.subscription = node.create_subscription(
        PointCloud2, node.params['input_topic'], on_cloud,
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics['unique_input_clouds'] = len(seen)
        (Path.cwd() / 'tracker_input_diagnostics.json').write_text(
            json.dumps(diagnostics, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def stop_owned(process):
    """Завершает только отдельную группу процесса, созданного этим скриптом."""
    if process.poll() is not None:
        return
    for signum, timeout in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout)
            return
        except subprocess.TimeoutExpired:
            pass
    raise RuntimeError(f'Не удалось завершить собственный процесс {process.pid}')


def wait_ready(node, executor, tracker, recorder, deadline):
    """До старта времени ждём реальные подписки вместо фиксированного сна."""
    while time.monotonic() < deadline:
        for name, process in (('tracker', tracker), ('recorder', recorder)):
            if process.poll() is not None:
                raise RuntimeError(f'{name} завершился до воспроизведения: {process.returncode}')
        if (node.count_subscribers(CLOUD) >= 1 and node.count_subscribers(TRUTH) >= 1
                and node.count_subscribers('/swarm/relative_poses') >= 2
                and node.count_publishers('/swarm/relative_poses') == 1):
            return
        executor.spin_once(timeout_sec=.1)
    raise RuntimeError('Не появились подписки трекера и оценки; см. журналы')


def run(args):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from swarm_interfaces.msg import RelativePose

    bag = args.bag.expanduser().resolve()
    bag_start, bag_end, topics = inspect_bag(bag)
    end = bag_end if args.end is None else args.end
    validate_window(args.start, end, bag_start, bag_end, args.rate, args.domain,
                    int(os.environ.get('ROS_DOMAIN_ID', '0')))
    configs = {'tracker.yaml': args.params_file.expanduser().resolve(),
               'evaluation.yaml': args.evaluation_params_file.expanduser().resolve()}
    contents = {name: path.read_text() for name, path in configs.items()}
    for value in contents.values():
        if not isinstance(yaml.safe_load(value), dict):
            raise ValueError('Файл параметров должен содержать YAML-словарь ROS')
    # Начальные треки из той же стартовой расстановки, что и в штатном запуске.
    from swarm_lidar.start_geometry import load_origins, tracker_seeds
    tracker_config = yaml.safe_load(contents['tracker.yaml'])
    tracker_parameters = tracker_config['lidar_tracker']['ros__parameters']
    tracker_parameters['initial_positions'] = tracker_seeds(
        load_origins(args.start_geometry.expanduser().resolve()), tracker_parameters['sensor_offset'])
    contents['tracker.yaml'] = yaml.safe_dump(tracker_config, allow_unicode=True)
    executables = {package: Path(get_package_prefix(package)) / 'lib' / package / executable
                   for package, executable in (('lidar_tracker', 'tracker_node'),
                                                ('evaluation', 'recorder_node'))}
    if not all(path.is_file() and os.access(path, os.X_OK) for path in executables.values()):
        raise ValueError('Сначала соберите lidar_tracker и evaluation и подключите install/setup.bash')

    # Свой Context не использует текущий ROS_DOMAIN_ID и ROS CLI daemon.
    context = Context()
    rclpy.init(context=context, domain_id=args.domain, args=[])
    node = rclpy.create_node('tracker_replay_monitor', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    processes, handles = [], []
    result = {'completed': False, 'bag': str(bag), 'start': args.start, 'end': end,
              'rate': args.rate, 'domain': args.domain, 'replayed_topics': topics,
              'cloud_transport': 'reliable' if args.reliable_cloud else 'recorded_qos',
              'old_relative_poses_replayed': False,
              'bootstrap': 'Новый трекер в известных стартовых слотах; без подстановки GT',
              'source_parameters': {name: str(path) for name, path in configs.items()},
              'valid_messages': {str(i): 0 for i in (1, 2, 3)}, 'invalid_messages': 0}
    output_created = False
    try:
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.1)
        others = [name for name, namespace in node.get_node_names_and_namespaces()
                  if (name, namespace) != (node.get_name(), node.get_namespace())]
        if others or node.count_publishers('/clock'):
            raise RuntimeError(f'Домен {args.domain} уже используется: {others}; выберите другой')
        args.output.mkdir(parents=True, exist_ok=False)
        output_created = True
        for name, content in contents.items():
            (args.output / name).write_text(content)
        poses = (args.output / 'relative_poses.jsonl').open('x')
        handles.append(poses)

        def on_relative(msg):
            if msg.valid and str(msg.target_id) in result['valid_messages']:
                result['valid_messages'][str(msg.target_id)] += 1
            else:
                result['invalid_messages'] += 1
            p = msg.pose.position
            poses.write(json.dumps({'target_id': msg.target_id, 'valid': msg.valid,
                                   'measurement_stamp': [msg.measurement_stamp.sec,
                                                         msg.measurement_stamp.nanosec],
                                   'position': [p.x, p.y, p.z], 'confidence': msg.confidence}) + '\n')

        subscription = node.create_subscription(RelativePose, '/swarm/relative_poses', on_relative, 100)
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))

        def spawn(name, command, cwd=None):
            handle = (args.output / f'{name}.log').open('x')
            handles.append(handle)
            process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                                       env=env, cwd=cwd, start_new_session=True)
            processes.append((name, process))
            return process

        tracker_command = ([sys.executable, str(Path(__file__).resolve()), '--reliable-tracker-worker']
                           if args.reliable_cloud else [str(executables['lidar_tracker'])])
        tracker = spawn('tracker', [*tracker_command, '--ros-args',
                                   '--params-file', str(args.output / 'tracker.yaml'),
                                   '-p', 'use_sim_time:=true', '-p', f'input_topic:={CLOUD}',
                                   '-p', 'output_topic:=/swarm/relative_poses'], cwd=args.output)
        recorder = spawn('evaluation', [str(executables['evaluation']), '--output',
                                       str(args.output / 'metrics'), '--ros-args', '--params-file',
                                       str(args.output / 'evaluation.yaml'), '-p', 'use_sim_time:=true'])
        wait_ready(node, executor, tracker, recorder, time.monotonic() + 15)
        qos_path = None
        if args.reliable_cloud:
            qos_path = args.output / 'playback_qos.yaml'
            qos_path.write_text(yaml.safe_dump({CLOUD: {
                'history': 'keep_last', 'depth': 10, 'reliability': 'reliable',
                'durability': 'volatile'}}))
        command = play_command(bag, args.start, end, bag_start, args.rate, topics, qos_path)
        result['play_command'] = command
        player = spawn('play', command)
        deadline = time.monotonic() + 30 + 4 * (end - args.start) / args.rate
        while player.poll() is None:
            if tracker.poll() is not None or recorder.poll() is not None:
                raise RuntimeError('Трекер или оценка завершились до конца записи')
            if time.monotonic() > deadline:
                raise RuntimeError('Превышено реальное время воспроизведения')
            executor.spin_once(timeout_sec=.1)
        if player.returncode != 0:
            raise RuntimeError(f'rosbag play завершился с кодом {player.returncode}; см. play.log')
        # Доставить последние оценки без продвижения времени эксперимента.
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        stop_owned(recorder)
        summary_path = args.output / 'metrics' / 'summary.json'
        if recorder.returncode != 0:
            raise RuntimeError(f'Оценка завершилась с кодом {recorder.returncode}; см. evaluation.log')
        if not summary_path.is_file() or not all(result['valid_messages'].values()):
            raise RuntimeError('Не получены новые валидные оценки всех трёх целей или итог оценки')
        summary = json.loads(summary_path.read_text())
        if any(summary['per_target'][str(i)]['localization_error_m']['count'] == 0 for i in (1, 2, 3)):
            raise RuntimeError('Нет сопоставленной с эталоном локализации всех трёх целей')
        result['completed'] = True
        node.destroy_subscription(subscription)
    except BaseException as error:
        result['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        cleanup_errors = []
        for _, process in reversed(processes):
            try:
                stop_owned(process)
            except (OSError, RuntimeError) as error:
                cleanup_errors.append(str(error))
        result['returncodes'] = {name: process.returncode for name, process in processes}
        bad_codes = {name: code for name, code in result['returncodes'].items() if code != 0}
        cleanup_failure = None
        if result['completed'] and (cleanup_errors or bad_codes):
            result['completed'] = False
            cleanup_failure = RuntimeError(f'Ошибка завершения процессов: {bad_codes}; {cleanup_errors}')
            result['error'] = str(cleanup_failure)
        diagnostics = args.output / 'tracker_input_diagnostics.json'
        if output_created and diagnostics.is_file():
            result['tracker_input_diagnostics'] = json.loads(diagnostics.read_text())
        for handle in handles:
            handle.close()
        if output_created:
            (args.output / 'result.json').write_text(
                json.dumps(result, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)
        if cleanup_failure is not None:
            raise cleanup_failure
    print(f'Новые измерения и оценка: {args.output}', flush=True)


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == '--reliable-tracker-worker':
        return reliable_tracker_worker(argv[1:])
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='Каталог MCAP с metadata.yaml')
    parser.add_argument('--output', type=Path, required=True, help='Новый каталог результатов')
    parser.add_argument('--start', type=float, required=True,
                        help='Начало в секундах записи; аппараты должны быть в известных стартовых слотах')
    parser.add_argument('--end', type=float, help='Конец в секундах записи; по умолчанию конец bag')
    parser.add_argument('--rate', type=float, default=.5, help='Темп воспроизведения, не выше 1')
    parser.add_argument('--domain', type=int, default=83, help='Свободный ROS_DOMAIN_ID')
    parser.add_argument('--reliable-cloud', action='store_true',
                        help='Надёжная доставка облака при повторе вместо BestEffort')
    parser.add_argument('--params-file', '--tracker-params', type=Path,
                        default=root / 'src/lidar_tracker/config/tracker.yaml')
    parser.add_argument('--evaluation-params-file', '--evaluation-params', type=Path,
                        default=root / 'src/evaluation/config/evaluation.yaml')
    parser.add_argument('--start-geometry', type=Path,
                        default=root / 'src/swarm_lidar/config/swarm.yaml',
                        help='Стартовая расстановка для начальных треков')
    args = parser.parse_args(argv)
    args.output = args.output.expanduser().resolve()

    def terminate(signum, frame):
        raise KeyboardInterrupt

    previous_handler = signal.signal(signal.SIGTERM, terminate)
    try:
        run(args)
    except (ImportError, OSError, ValueError, KeyError, RuntimeError) as error:
        print(f'Ошибка повтора: {error}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('Повтор остановлен; свои процессы завершены', file=sys.stderr)
        return 130
    finally:
        signal.signal(signal.SIGTERM, previous_handler)
    return 0


if __name__ == '__main__':
    sys.exit(main())
