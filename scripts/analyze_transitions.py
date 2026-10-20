#!/usr/bin/env python3
"""Перестроения: время установления и знаковый перелёт по сохранённому MCAP."""

import argparse
import csv
import json
import math
from pathlib import Path
import sys

import numpy as np

from evaluation.metrics import Pose, model_id, model_to_base, relative_heading
from evaluation.transition_metrics import analyze_transition, interpolate_offsets


def transition_windows(statuses):
    """Пауза DATA_HOLD не создаёт новое перестроение; FOLLOWING завершает цель."""
    windows = []
    active = None
    for status in statuses:
        stamp, mode = float(status['time']), status['mode']
        if mode == 'FORMATION_CHANGE' and active is None:
            if windows and 'end' not in windows[-1]:
                windows[-1]['end'] = stamp
            active = dict(start=stamp, formation=status['formation'], scale=status['scale'],
                          plan_end=None, interruptions=[])
        if active is not None:
            if mode == 'FOLLOWING':
                active['plan_end'] = stamp
                windows.append(active)
                active = None
            elif mode != 'FORMATION_CHANGE':
                if not active['interruptions'] or active['interruptions'][-1]['mode'] != mode:
                    active['interruptions'].append(dict(time=stamp, mode=mode))
        if mode in ('DISABLED', 'HOLD', 'CLOCK_RESET', 'COLLISION_STOP'):
            if active is not None:
                active['end'] = stamp
                windows.append(active)
                active = None
            if windows and 'end' not in windows[-1]:
                windows[-1]['end'] = stamp
    if active is not None:
        windows.append(active)
    if windows and 'end' not in windows[-1]:
        windows[-1]['end'] = float(statuses[-1]['time'])
    return windows


def _seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def read_bag(path, base_z=.24):
    """Чтение трёх тем без создания ROS-узла и без запуска воспроизведения."""
    import rosbag2_py
    from geometry_msgs.msg import PoseArray
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    types = {'/swarm/target_offsets': PoseArray, '/swarm/formation_status': String,
             '/evaluation/ground_truth': TFMessage}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    targets, truth, statuses = {}, {}, {}
    previous = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        message = deserialize_message(data, types[topic])
        if topic == '/swarm/target_offsets':
            stamp = _seconds(message.header.stamp)
            if message.header.frame_id != 'uav_0/heading' or len(message.poses) != 3:
                continue
            points = np.array([[p.position.x, p.position.y, p.position.z] for p in message.poses])
            if not np.isfinite(points).all():
                continue
            targets[stamp] = points
        elif topic == '/swarm/formation_status':
            value = json.loads(message.data)
            stamp = float(value['time'])
            statuses[stamp] = value
        else:
            grouped = {}
            for transform in message.transforms:
                match = model_id(transform.child_frame_id)
                if match is None or transform.header.frame_id != 'map':
                    continue
                index, priority = match
                source = _seconds(transform.header.stamp)
                pose = transform.transform
                t, q = pose.translation, pose.rotation
                values = (t.x, t.y, t.z, q.x, q.y, q.z, q.w)
                if not all(math.isfinite(v) for v in values) or sum(v*v for v in values[3:]) < 1e-12:
                    continue
                group = grouped.setdefault(source, {})
                if index not in group or priority < group[index][0]:
                    group[index] = priority, model_to_base(Pose(values[:3], values[3:]), base_z)
            for stamp, group in grouped.items():
                if all(i in group for i in range(4)):
                    truth[stamp] = np.array([relative_heading(group[0][1], group[i][1])
                                             for i in (1, 2, 3)])
            stamp = min(grouped) if grouped else None
        if stamp is not None:
            if not math.isfinite(stamp):
                raise ValueError('Неконечное время источника')
            if stamp < previous.get(topic, stamp)-1.:
                raise ValueError('Сброс времени: анализируйте одну временную эпоху отдельно')
            previous[topic] = max(stamp, previous.get(topic, stamp))
    return sorted(targets.items()), sorted(truth.items()), [statuses[t] for t in sorted(statuses)]


def analyze_bag(targets, truth, statuses, threshold=.3, dwell=1., max_gap=.2):
    target_times = np.array([entry[0] for entry in targets])
    gt_times = np.array([entry[0] for entry in truth])
    if len(targets) < 2 or not len(truth) or not statuses:
        raise ValueError('Недостаточно заданий, эталонных поз или состояний формации')
    desired = interpolate_offsets(target_times, np.array([entry[1] for entry in targets]),
                                  gt_times, max_gap)
    actual = np.array([entry[1] for entry in truth])
    reports, sample_rows = [], []
    for number, window in enumerate(transition_windows(statuses), 1):
        start, end, plan_end = window['start'], window['end'], window['plan_end']
        # Одинаковый ROS stamp может получить различное округление в JSON
        # статуса и header PoseArray. Начало берём строго из предыдущего такта.
        initial_index = np.searchsorted(target_times, start-1e-9, side='left')-1
        final_index = np.searchsorted(target_times, plan_end+1e-9, side='right')-1 if plan_end else -1
        if (plan_end is None or initial_index < 0 or final_index < 0
                or start-target_times[initial_index] > max_gap
                or plan_end-target_times[final_index] > max_gap):
            reports.append(dict(number=number, **window, unavailable='incomplete_target_history'))
            continue
        report = analyze_transition(gt_times, actual, desired, targets[initial_index][1],
                                     targets[final_index][1], start, plan_end, end,
                                     threshold, dwell, max_gap)
        report.update(number=number, formation=window['formation'], scale=window['scale'],
                      interruptions=window['interruptions'],
                      initial_target_sample_time=float(target_times[initial_index]),
                      final_target_sample_time=float(target_times[final_index]))
        reports.append(report)
        for k in np.flatnonzero((gt_times >= start) & (gt_times <= end)):
            for i in range(3):
                row = dict(transition=number, timestamp=float(gt_times[k]), target_id=i+1)
                for prefix, points in [('actual', actual[k]), ('dynamic_target', desired[k])]:
                    row.update({f'{prefix}_{axis}_m': float(points[i, j])
                                for j, axis in enumerate(('x', 'y', 'z'))})
                sample_rows.append(row)
    return dict(transitions=reports), sample_rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='Новый JSON')
    parser.add_argument('--samples-csv', type=Path, help='Новый CSV согласованных поз и заданий')
    parser.add_argument('--threshold', type=float, default=.3, help='Полоса установления, м')
    parser.add_argument('--dwell', type=float, default=1., help='Минимальная длина конечного участка, с')
    parser.add_argument('--max-gap', type=float, default=.2, help='Допустимый разрыв данных, с')
    parser.add_argument('--model-base-z', type=float, default=.24, help='Смещение base_link модели по Z, м')
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in (args.threshold, args.dwell, args.max_gap, args.model_base_z))
            or min(args.threshold, args.dwell, args.max_gap) <= 0):
        parser.error('Параметры должны быть конечными; threshold, dwell, max-gap — положительными')
    if args.output.exists() or (args.samples_csv and args.samples_csv.exists()):
        parser.error('Выходные файлы уже существуют')
    try:
        targets, truth, statuses = read_bag(args.bag, args.model_base_z)
        report, rows = analyze_bag(targets, truth, statuses, args.threshold, args.dwell, args.max_gap)
        report.update(source=str(args.bag), frame='uav_0/heading (X вперёд, Y влево, Z вверх)',
                      source_topics=['/swarm/target_offsets', '/swarm/formation_status',
                                     '/evaluation/ground_truth'],
                      model_base_z=args.model_base_z, time_source='source timestamp, simulation seconds',
                      note='Перелёт — положительное пересечение конечной цели вдоль изменения '
                           'слота. Ошибка до движущегося слота считается отдельно. '
                           'Установление подтверждено только до конца окна наблюдения.')
        with args.output.open('x') as handle:
            json.dump(report, handle, ensure_ascii=False, indent=2, allow_nan=False)
            handle.write('\n')
        if args.samples_csv:
            with args.samples_csv.open('x', newline='') as handle:
                fields = ['transition', 'timestamp', 'target_id'] + [
                    f'{prefix}_{axis}_m' for prefix in ('actual', 'dynamic_target') for axis in ('x', 'y', 'z')]
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerows(rows)
    except (ImportError, OSError, ValueError, RuntimeError, KeyError) as error:
        print(f'Ошибка анализа: {error}', file=sys.stderr)
        return 1
    print(f'Анализ сохранён: {args.output}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
