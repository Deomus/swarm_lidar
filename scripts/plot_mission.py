#!/usr/bin/env python3
"""Траектории лидера и ведомых в миссии S3 по эталону Gazebo из rosbag.

На графике: маршрут A*, принятый ведущим, столбы площадки и пути всех
четырёх аппаратов за время миссии (статус MISSION ведущего). Эталон
используется только здесь, для оценки, в управление он не попадает.
"""

import argparse
import json
from pathlib import Path

import yaml

from evaluation.metrics import Pose, model_id, model_to_base


def read_bag(path, base_z):
    import rosbag2_py
    from nav_msgs.msg import Path as PathMsg
    from rclpy.serialization import deserialize_message
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    types = {'/evaluation/ground_truth': TFMessage, '/mission/path': PathMsg,
             '/mission/execution_status': String}
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    tracks, route, mission = {i: {} for i in range(4)}, None, []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, types[topic])
        if topic == '/mission/path':
            # Пустой путь — отзыв маршрута после завершения; нужен последний принятый.
            if msg.poses:
                route = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        elif topic == '/mission/execution_status':
            status = json.loads(msg.data)
            if status.get('mode') == 'MISSION':
                mission.append(float(status['stamp']))
        else:
            for transform in msg.transforms:
                match = model_id(transform.child_frame_id)
                if match is None or transform.header.frame_id != 'map':
                    continue
                index, priority = match
                stamp = transform.header.stamp.sec + transform.header.stamp.nanosec*1e-9
                old = tracks[index].get(stamp)
                if old is not None and old[0] <= priority:
                    continue
                p, q = transform.transform.translation, transform.transform.rotation
                pose = model_to_base(Pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)), base_z)
                tracks[index][stamp] = priority, pose.position
    if not mission or route is None:
        raise ValueError('В записи нет миссии ведущего или маршрута')
    start, end = min(mission), max(mission)
    paths = {i: [point for stamp, (_, point) in sorted(samples.items()) if start <= stamp <= end]
             for i, samples in tracks.items()}
    return paths, route


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path, help='Каталог сценария с bag/')
    parser.add_argument('--output', type=Path, required=True, help='Новый PNG')
    parser.add_argument('--obstacles', type=Path,
                        default=Path(__file__).resolve().parents[1]/'src/swarm_lidar/config/obstacles.yaml')
    parser.add_argument('--model-base-z', type=float, default=.24)
    args = parser.parse_args()
    if args.output.exists():
        raise SystemExit(f'{args.output} уже существует')
    paths, route = read_bag(args.scenario/'bag', args.model_base_z)
    obstacles = yaml.safe_load(args.obstacles.read_text())

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, axis = plt.subplots(figsize=(8, 8))
    for x, y in obstacles['pillars']:
        axis.add_patch(plt.Circle((x, y), obstacles['radius'], color='#8d6e63', label='столб'))
    axis.plot(*zip(*route), '--', color='#424242', linewidth=1.5, label='маршрут A*')
    colors = {0: '#e53935', 1: '#1e88e5', 2: '#43a047', 3: '#fb8c00'}
    for i, points in paths.items():
        if points:
            xs, ys = [p[0] for p in points], [p[1] for p in points]
            axis.plot(xs, ys, color=colors[i], linewidth=2 if i == 0 else 1.2,
                      label='ведущий' if i == 0 else f'ведомый uav_{i}')
            axis.plot(xs[0], ys[0], 'o', color=colors[i])
            axis.plot(xs[-1], ys[-1], 's', color=colors[i])
    axis.plot(*route[-1], '*', color='black', markersize=14, label='цель')
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), loc='best')
    axis.set_aspect('equal')
    axis.set_xlabel('X (восток), м')
    axis.set_ylabel('Y (север), м')
    axis.set_title('Миссия: траектории по эталону Gazebo (кружок — старт, квадрат — конец)')
    axis.grid(True, alpha=.3)
    figure.tight_layout()
    figure.savefig(args.output, dpi=150)
    print(args.output)


if __name__ == '__main__':
    main()
