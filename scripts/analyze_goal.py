#!/usr/bin/env python3
"""Конечная ошибка ведущего по эталону в момент подтверждения цели сценария S3."""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from evaluation.metrics import Pose, model_id, model_to_base


def final_goal(samples, goals, event_time, tolerance=.15):
    """Ближайшая эталонная поза и последняя цель до события; без интерполяции."""
    if not math.isfinite(event_time) or not math.isfinite(tolerance) or tolerance <= 0:
        raise ValueError('Некорректное время события или допуск')
    eligible_goals = [(stamp, point) for stamp, point in goals if math.isfinite(stamp)
                      and stamp <= event_time and np.asarray(point).shape == (3,)
                      and np.isfinite(point).all()]
    eligible_samples = [(stamp, point) for stamp, point in samples if math.isfinite(stamp)
                        and abs(stamp-event_time) <= tolerance
                        and np.asarray(point).shape == (3,) and np.isfinite(point).all()]
    if not eligible_goals or not eligible_samples:
        raise ValueError('Нет цели до события или близкого эталонного снимка')
    goal_stamp, goal = max(eligible_goals, key=lambda item: item[0])
    stamp, point = min(eligible_samples, key=lambda item: (abs(item[0]-event_time), item[0]))
    return dict(event_time=event_time, gt_time=stamp, gt_delta_s=stamp-event_time,
                goal_time=goal_stamp, goal_map_m=list(goal), leader_base_map_m=list(point),
                final_goal_error_m=float(np.linalg.norm(np.asarray(point)-goal)),
                match_tolerance_s=tolerance)


def read_bag(path, base_z):
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/mission/goal', '/evaluation/ground_truth']))
    samples, goals = {}, []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/mission/goal':
            msg = deserialize_message(data, PoseStamped)
            if msg.header.frame_id == 'map':
                p = msg.pose.position
                goals.append((msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9, [p.x, p.y, p.z]))
        else:
            msg = deserialize_message(data, TFMessage)
            for transform in msg.transforms:
                match = model_id(transform.child_frame_id)
                if match is None or match[0] != 0 or transform.header.frame_id != 'map':
                    continue
                stamp = transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
                if stamp in samples and samples[stamp][0] <= match[1]:
                    continue
                p, q = transform.transform.translation, transform.transform.rotation
                pose = model_to_base(Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), base_z)
                samples[stamp] = match[1], pose.position
    return [(stamp, point) for stamp, (_, point) in samples.items()], goals


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--tolerance', type=float, default=.15)
    parser.add_argument('--model-base-z', type=float, default=.24)
    args = parser.parse_args()
    if (not math.isfinite(args.model_base_z) or not math.isfinite(args.tolerance)
            or args.tolerance <= 0):
        parser.error('Нужны конечное смещение основания и положительный допуск')
    if args.output.exists():
        parser.error('Выходной файл уже существует')
    result = json.loads((args.scenario/'result.json').read_text())
    events = [e for e in result['events'] if e['event'].startswith('Цель достигнута:')]
    if result['scenario'] not in ('s3', 's3_pillars') or not result['passed'] or len(events) != 1:
        parser.error('Нужен успешный S3 с единственным подтверждением цели')
    samples, goals = read_bag(args.scenario/'bag', args.model_base_z)
    report = final_goal(samples, goals, events[0]['time'], args.tolerance)
    report.update(source=str(args.scenario), model_base_z=args.model_base_z,
                  event=events[0]['event'], method='nearest source timestamp, map ENU, model to base')
    with args.output.open('x') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(f'Конечная ошибка по эталону: {report["final_goal_error_m"]:.3f} м; {args.output}')


if __name__ == '__main__':
    main()
