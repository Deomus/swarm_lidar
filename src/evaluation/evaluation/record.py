"""Пассивная запись CSV; ground truth не публикуется обратно в управление."""
import argparse
import csv
from collections import deque
import json
from math import isfinite
from pathlib import Path

from geometry_msgs.msg import PoseArray
import numpy as np
import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import RelativePose, SwarmState
from tf2_msgs.msg import TFMessage

from evaluation.metrics import (Pose, min_pair_distance, model_id, nearest,
                                relative_heading, relative_lidar, summarize)


FIELDS = ['timestamp', 'target_id', 'measurement_timestamp', 'gt_match_delta_s',
          'localization_error_m', 'formation_error_m', 'min_pair_distance_m',
          'relative_stale', 'leader_state_stale', 'gt_missing', 'formation_missing',
          'localization_missing']


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class Recorder(Node):
    def __init__(self, output):
        super().__init__('evaluation_recorder')
        for key, value in {'period': .1, 'match_tolerance': .15, 'stale_timeout': .5,
                           'offset_timeout': .5, 'sensor_z': .45, 'marker_z': .45}.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in
                  ['period', 'match_tolerance', 'stale_timeout', 'offset_timeout', 'sensor_z', 'marker_z']}
        if not all(isfinite(v) for v in self.p.values()) or any(
                self.p[k] <= 0 for k in ['period', 'match_tolerance', 'stale_timeout', 'offset_timeout']):
            raise ValueError('Некорректные параметры evaluation')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.file = (self.output / 'samples.csv').open('x', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
        self.writer.writeheader()
        self.rows = []
        self.gt = deque(maxlen=1000)
        self.relative = {i: deque(maxlen=100) for i in range(1, 4)}
        self.offsets = deque(maxlen=100)
        self.state = None
        self.last_time = None
        self.create_subscription(TFMessage, '/evaluation/ground_truth', self.on_gt, 10)
        self.create_subscription(RelativePose, '/swarm/relative_poses', self.on_relative, 10)
        self.create_subscription(PoseArray, '/swarm/target_offsets', self.on_offsets, 10)
        self.create_subscription(SwarmState, '/mesh/states', self.on_state, 10)
        self.create_timer(self.p['period'], self.tick)

    def on_gt(self, msg):
        grouped = {}
        for transform in msg.transforms:
            match = model_id(transform.child_frame_id)
            if match is None:
                continue
            index, priority = match
            stamp = seconds(transform.header.stamp)
            group = grouped.setdefault(stamp, {})
            if index in group and group[index][0] <= priority:
                continue
            t, q = transform.transform.translation, transform.transform.rotation
            if not all(isfinite(v) for v in [t.x, t.y, t.z, q.x, q.y, q.z, q.w]):
                continue
            if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
                continue
            group[index] = priority, Pose((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        for stamp, poses in grouped.items():
            self.gt.append((stamp, {i: item[1] for i, item in poses.items()}))

    def on_relative(self, msg):
        if msg.target_id in self.relative:
            self.relative[msg.target_id].append((seconds(msg.measurement_stamp), msg))

    def on_offsets(self, msg):
        if msg.header.frame_id == 'uav_0/heading' and len(msg.poses) == 3:
            if all(isfinite(v) for p in msg.poses for v in [p.position.x, p.position.y, p.position.z]):
                self.offsets.append((seconds(msg.header.stamp), msg))

    def on_state(self, msg):
        if msg.uav_id == 0:
            self.state = msg

    def latest(self, samples, now):
        eligible = [sample for sample in samples if sample[0] <= now]
        return max(enumerate(eligible), key=lambda entry: (entry[1][0], entry[0]))[1] if eligible else None

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_time is not None and now < self.last_time:
            self.gt.clear()
            self.offsets.clear()
            for history in self.relative.values():
                history.clear()
            self.state = None
        if self.last_time is not None and now <= self.last_time:
            self.last_time = now
            return
        self.last_time = now
        truth = nearest(self.gt, now, self.p['match_tolerance'])
        offsets = self.latest(self.offsets, now)
        offsets_fresh = offsets is not None and now - offsets[0] <= self.p['offset_timeout']
        state_fresh = (self.state is not None and self.state.valid
                       and 0 <= now - seconds(self.state.timestamp) <= self.p['stale_timeout'])
        for index in range(1, 4):
            sample = self.latest(self.relative[index], now)
            valid = (sample is not None and sample[1].valid
                     and sample[1].frame_id == 'uav_0/lidar_link'
                     and now - sample[0] <= self.p['stale_timeout']
                     and all(isfinite(v) for v in [sample[1].pose.position.x,
                                                   sample[1].pose.position.y,
                                                   sample[1].pose.position.z]))
            row = dict.fromkeys(FIELDS, float('nan'))
            row.update(timestamp=now, target_id=index, relative_stale=int(not valid),
                       leader_state_stale=int(not state_fresh), gt_missing=1,
                       formation_missing=1, localization_missing=1)
            if sample:
                row['measurement_timestamp'] = sample[0]
            if truth and all(i in truth[1] for i in range(4)):
                row['gt_missing'] = 0
                row['min_pair_distance_m'] = min_pair_distance(truth[1])
                if offsets_fresh:
                    desired = offsets[1].poses[index - 1].position
                    actual = relative_heading(truth[1][0], truth[1][index])
                    row['formation_error_m'] = float(np.linalg.norm(actual - [desired.x, desired.y, desired.z]))
                    row['formation_missing'] = 0
            matched = nearest(self.gt, sample[0], self.p['match_tolerance']) if valid else None
            if matched and 0 in matched[1] and index in matched[1]:
                point = sample[1].pose.position
                if all(isfinite(v) for v in [point.x, point.y, point.z]):
                    actual = relative_lidar(matched[1][0], matched[1][index],
                                            (0., 0., self.p['sensor_z']), (0., 0., self.p['marker_z']))
                    row['localization_error_m'] = float(np.linalg.norm(actual - [point.x, point.y, point.z]))
                    row['gt_match_delta_s'] = matched[0] - sample[0]
                    row['localization_missing'] = 0
            self.writer.writerow(row)
            self.rows.append(row)
        self.file.flush()

    def finish(self):
        self.file.close()
        result = summarize(self.rows)
        result['parameters'] = self.p
        (self.output / 'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    config, ros_args = parser.parse_known_args(args)
    rclpy.init(args=ros_args)
    node = Recorder(config.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
