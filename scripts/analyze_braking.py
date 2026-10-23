#!/usr/bin/env python3
"""Проверка физического торможения после потери данных по сохранённой записи."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

from evaluation.braking_metrics import analyze_braking
from evaluation.metrics import Pose, model_id, model_to_base


def read_bag(path, start, end, base_z=.24):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from swarm_interfaces.msg import SwarmCommand
    from tf2_msgs.msg import TFMessage

    # Команды ведомых — то, что получает мост PX4 каждого ведомого.
    command_topics = {f'/uav_{i}/command' for i in (1, 2, 3)}
    types = {**{topic: SwarmCommand for topic in command_topics},
             '/swarm/formation_status': String, '/evaluation/ground_truth': TFMessage}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    positions, commands, statuses = {i: {} for i in (1, 2, 3)}, {i: [] for i in (1, 2, 3)}, {}
    while reader.has_next():
        topic, data, recorded_ns = reader.read_next()
        recorded_time = recorded_ns*1e-9
        if recorded_time < start or recorded_time > end or topic not in types:
            continue
        message = deserialize_message(data, types[topic])
        if topic == '/swarm/formation_status':
            status = json.loads(message.data)
            statuses[float(status['time'])] = status
        elif topic in command_topics:
            if message.uav_id not in commands:
                continue
            velocity = message.target_velocity.linear
            source_time = message.source_stamp.sec+message.source_stamp.nanosec*1e-9
            commands[message.uav_id].append(dict(
                time=recorded_time, source_time=source_time,
                velocity=[velocity.x, velocity.y, velocity.z], frame=message.frame_id,
                valid=message.valid, safety=message.safety_state))
        else:
            groups = {}
            for transform in message.transforms:
                match = model_id(transform.child_frame_id)
                if match is None or match[0] not in positions or transform.header.frame_id != 'map':
                    continue
                index, priority = match
                stamp = transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
                t, q = transform.transform.translation, transform.transform.rotation
                values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
                if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[3:]) < 1e-12:
                    continue
                group = groups.setdefault(stamp, {})
                if index not in group or priority < group[index][0]:
                    group[index] = priority, model_to_base(Pose(values[:3], values[3:]), base_z).position
            for stamp, group in groups.items():
                for index, (_, point) in group.items():
                    positions[index][stamp] = point
    return {i: sorted(stream.items()) for i, stream in positions.items()}, commands, [
        statuses[t] for t in sorted(statuses)]


def safety_window(commands, statuses, loss_time, end, freshness=.5, max_gap=.2,
                  clock_tolerance=0.):
    """Только свежая invalid/нулевая DATA_HOLD; восстановление обрывает окно."""
    if not math.isfinite(clock_tolerance) or not 0 <= clock_tolerance <= .05:
        raise ValueError('Допуск согласования часов должен быть от 0 до 0.05 с')
    first, last = None, None
    status_times = np.array([value['time'] for value in statuses])
    for command in commands:
        stamp = command['time']
        if stamp < loss_time-1e-9 or stamp > end:
            continue
        status_index = np.searchsorted(status_times, stamp+1e-9, side='right')-1
        status = statuses[status_index] if status_index >= 0 else None
        fresh_status = (status is not None and status['mode'] == 'DATA_HOLD'
                        and -1e-9 <= stamp-status['time'] <= freshness)
        zero = (not command['valid'] and command['safety'] == 'DATA_HOLD'
                and command['frame'] == 'map'
                and -clock_tolerance-1e-9 <= stamp-command['source_time'] <= freshness
                and all(math.isfinite(v) and abs(v) <= 1e-9 for v in command['velocity'])
                and fresh_status)
        if first is None:
            if zero:
                first = last = stamp
        elif not zero or stamp-last > max_gap+1e-9:
            # Консервативная граница — последний подтверждённый нулевой такт.
            break
        else:
            last = stamp
    return None if first is None else (first, last)


def analyze_loss(positions, commands, statuses, loss_time, end, speed_threshold=.15,
                 dwell=.5, motion_threshold=.3, max_gap=.2, clock_tolerance=0.):
    followers, rows = {}, []
    for index in (1, 2, 3):
        window = safety_window(commands[index], statuses, loss_time, end, max_gap=max_gap,
                               clock_tolerance=clock_tolerance)
        if window is None:
            followers[str(index)] = dict(physical_braking_verified=False,
                                         unverified_reason='fresh_zero_command_not_found')
            continue
        stream = positions[index]
        times = np.array([sample[0] for sample in stream])
        points = np.array([sample[1] for sample in stream]).reshape((-1, 3))
        report, velocity_times, velocities = analyze_braking(
            times, points, loss_time, window[0], window[1], speed_threshold, dwell,
            motion_threshold, max_gap=max_gap)
        followers[str(index)] = report
        for stamp, velocity in zip(velocity_times, velocities):
            rows.append(dict(timestamp=float(stamp), target_id=index,
                             vx_mps=float(velocity[0]), vy_mps=float(velocity[1]),
                             vz_mps=float(velocity[2]), speed_mps=float(np.linalg.norm(velocity))))
    return dict(loss_time=loss_time, followers=followers,
                physical_braking_verified=all(f.get('physical_braking_verified') for f in followers.values())), rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path, help='Каталог с bag и result.json')
    parser.add_argument('--output', type=Path, required=True, help='Новый JSON')
    parser.add_argument('--samples-csv', type=Path, help='Новый CSV измеренных скоростей')
    parser.add_argument('--speed-threshold', type=float, default=.15)
    parser.add_argument('--dwell', type=float, default=.5)
    parser.add_argument('--motion-threshold', type=float, default=.3)
    parser.add_argument('--max-gap', type=float, default=.2)
    parser.add_argument('--clock-tolerance', type=float, default=0.,
                        help='Явный допуск невязки часов записи и источника, с (0..0.05)')
    parser.add_argument('--model-base-z', type=float, default=.24)
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in (args.speed_threshold, args.dwell, args.motion_threshold,
                                          args.max_gap, args.model_base_z, args.clock_tolerance))
            or min(args.speed_threshold, args.dwell, args.max_gap) <= 0
            or args.motion_threshold <= args.speed_threshold
            or not 0 <= args.clock_tolerance <= .05):
        parser.error('Некорректные пороги или интервалы')
    if args.output.exists() or (args.samples_csv and args.samples_csv.exists()):
        parser.error('Выходные файлы уже существуют')
    try:
        result = json.loads((args.scenario/'result.json').read_text())
        losses = [event for event in result['events'] if event['event'].startswith('Потеря LiDAR-поз')]
        if len(losses) != 1:
            raise ValueError('Нужна ровно одна помеченная потеря LiDAR в result.json')
        loss = float(losses[0]['time'])
        if not math.isfinite(loss):
            raise ValueError('Некорректное время потери данных')
        positions, commands, statuses = read_bag(args.scenario/'bag', loss-2., loss+5., args.model_base_z)
        report, rows = analyze_loss(positions, commands, statuses, loss, loss+5.,
                                    args.speed_threshold, args.dwell, args.motion_threshold, args.max_gap,
                                    args.clock_tolerance)
        leads = [command['source_time']-command['time'] for stream in commands.values()
                 for command in stream if not command['valid'] and command['safety'] == 'DATA_HOLD'
                 and math.isfinite(command['source_time']) and math.isfinite(command['time'])]
        report.update(source=str(args.scenario), model_base_z=args.model_base_z,
                      clock_alignment_tolerance_s=args.clock_tolerance,
                      command_clock_diagnostics=dict(
                          sampled_zero_commands=len(leads),
                          source_ahead_count=sum(value > 1e-9 for value in leads),
                          max_source_ahead_s=max([0., *leads])),
                      gt_frame='map ENU', velocity_method='finite differences at interval midpoints',
                      command_event_clock='bag recording timestamp (ROS simulation clock)',
                      note='Низкая скорость уже до потери данных не подтверждает торможение '
                           'в движении. source_stamp valid-команды относится к измерению LiDAR, '
                           'а не к моменту выдачи команды. Вывод причинности не делается.')
        with args.output.open('x') as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write('\n')
        if args.samples_csv:
            with args.samples_csv.open('x', newline='') as handle:
                fields = ['timestamp', 'target_id', 'vx_mps', 'vy_mps', 'vz_mps', 'speed_mps']
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
    except (OSError, ImportError, ValueError, RuntimeError, KeyError) as error:
        print(f'Ошибка анализа: {error}', file=sys.stderr)
        return 1
    print(f'Анализ сохранён: {args.output}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
