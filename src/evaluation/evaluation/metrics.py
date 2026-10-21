"""Чистые функции геометрии, согласования времени и статистики."""
from dataclasses import dataclass
from math import atan2, cos, isfinite, sin
import re

import numpy as np


@dataclass(frozen=True)
class Pose:
    position: tuple
    quaternion: tuple  # XYZW, local -> world


def rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Некорректный quaternion GT')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
                     [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
                     [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)]])


def relative_lidar(leader, target, sensor_offset=(0., 0., 1.9), marker_offset=(0., 0., 0.)):
    marker_world = np.asarray(target.position) + rotation(target.quaternion) @ marker_offset
    return (rotation(leader.quaternion).T @ (marker_world - leader.position)
            - np.asarray(sensor_offset))


def model_to_base(pose, base_z=.24):
    position = np.asarray(pose.position) + rotation(pose.quaternion) @ [0., 0., base_z]
    return Pose(tuple(position), pose.quaternion)


def relative_heading(leader, target):
    matrix = rotation(leader.quaternion)
    yaw = atan2(matrix[1, 0], matrix[0, 0])
    transform = np.array([[cos(yaw), sin(yaw), 0], [-sin(yaw), cos(yaw), 0], [0, 0, 1]])
    return transform @ (np.asarray(target.position) - leader.position)


def min_pair_distance(poses):
    if not all(i in poses for i in range(4)):
        return float('nan')
    return min(float(np.linalg.norm(np.asarray(poses[i].position) - poses[j].position))
               for i in range(4) for j in range(i + 1, 4))


def nearest(samples, stamp, tolerance):
    """Ближайший снимок по source time; интерполяция отсутствует."""
    if not samples:
        return None
    candidate = min(samples, key=lambda sample: abs(sample[0] - stamp))
    return candidate if abs(candidate[0] - stamp) <= tolerance else None


def interpolate_truth(samples, stamp, tolerance):
    """Эталон на время измерения; большие разрывы остаются пропусками."""
    before = [s for s in samples if s[0] <= stamp]
    after = [s for s in samples if s[0] >= stamp]
    if not before or not after:
        return nearest(samples, stamp, min(.005, tolerance))
    a, b = max(before, key=lambda s: s[0]), min(after, key=lambda s: s[0])
    if stamp-a[0] > tolerance or b[0]-stamp > tolerance:
        return None
    if a[0] == b[0]:
        return a
    fraction = (stamp-a[0]) / (b[0]-a[0])
    poses = {}
    for key in a[1].keys() & b[1].keys():
        first, last = a[1][key], b[1][key]
        q1, q2 = np.array(first.quaternion), np.array(last.quaternion)
        q1, q2 = q1/np.linalg.norm(q1), q2/np.linalg.norm(q2)
        if q1 @ q2 < 0:
            q2 = -q2
        angle = np.arccos(np.clip(q1 @ q2, -1., 1.))
        quaternion = ((1-fraction)*q1 + fraction*q2 if angle < 1e-5 else
                      (np.sin((1-fraction)*angle)*q1 + np.sin(fraction*angle)*q2)/np.sin(angle))
        position = (1-fraction)*np.array(first.position) + fraction*np.array(last.position)
        poses[key] = Pose(tuple(position), tuple(quaternion/np.linalg.norm(quaternion)))
    return stamp, poses


class SummaryAccumulator:
    """Те же сводные показатели без сохранения всех строк в памяти."""
    metrics = ('localization_error_m', 'formation_error_m', 'min_pair_distance_m', 'route_error_m', 'goal_error_m')
    flags = ('relative_stale', 'leader_state_stale', 'gt_missing', 'formation_missing', 'localization_missing')

    def __init__(self, phases=True):
        self.count = 0
        self.values = {}
        self.shares = dict.fromkeys(self.flags, 0)
        self.phases = ({key: SummaryAccumulator(False) for key in
            ('active_control', 'line', 'column', 'wedge', 'mission')} if phases else None)

    def add_value(self, key, value):
        if not isfinite(value):
            return
        row = self.values.setdefault(key, [0, 0., 0., -float('inf'), float('inf')])
        row[0] += 1
        row[1] += value
        row[2] += value*value
        row[3], row[4] = max(row[3], value), min(row[4], value)

    def update(self, row):
        self.count += 1
        for key in self.metrics:
            self.add_value(key, row.get(key, float('nan')))
        for key in ('localization_error_m', 'formation_error_m'):
            self.add_value((row['target_id'], key), row.get(key, float('nan')))
        for key in self.flags:
            self.shares[key] += row[key]
        if self.phases is not None:
            if row.get('controller_active') == 1:
                self.phases['active_control'].update(row)
                name = {1: 'line', 2: 'column', 3: 'wedge'}.get(row.get('formation_code'))
                if name and row.get('transitioning') == 0:
                    self.phases[name].update(row)
            if row.get('mission_active') == 1:
                self.phases['mission'].update(row)

    def stats(self, key):
        n, total, squares, maximum, minimum = self.values.get(key, [0, 0., 0., None, None])
        return dict(count=n, mean=total/n if n else None, rms=(squares/n)**.5 if n else None,
                    max=maximum, min=minimum)

    def result(self):
        result = {key: self.stats(key) for key in self.metrics}
        result['rows'] = self.count
        result.update({key+'_share': value/self.count if self.count else None for key, value in self.shares.items()})
        result['per_target'] = {str(i): {key: self.stats((i, key)) for key in
            ('localization_error_m', 'formation_error_m')} for i in range(1, 4)}
        if self.phases is not None:
            result['active_control'] = self.phases['active_control'].result()
            result['following_by_formation'] = {name: self.phases[name].result() for name in ('line', 'column', 'wedge')}
            result['mission'] = self.phases['mission'].result()
        return result


def model_id(name):
    """Только model frames, не вложенные base_link/rotor frames."""
    tokens = [part for part in re.split(r'::|/', name) if part]
    if not tokens or not re.fullmatch(r'uav_[0-3]', tokens[-1]):
        return None
    return int(tokens[-1][-1]), (0 if len(tokens) == 1 else 1)


def scalar_stats(values):
    data = np.asarray([float(v) for v in values if isfinite(float(v))])
    if not len(data):
        return {'count': 0, 'mean': None, 'rms': None, 'max': None, 'min': None}
    return {'count': len(data), 'mean': float(data.mean()),
            'rms': float(np.sqrt(np.mean(data * data))),
            'max': float(data.max()), 'min': float(data.min())}


def summarize(rows, include_phases=True):
    def metric(key):
        return scalar_stats(row.get(key, float('nan')) for row in rows)
    result = {key: metric(key) for key in ['localization_error_m', 'formation_error_m',
                                         'min_pair_distance_m', 'route_error_m', 'goal_error_m']}
    result['rows'] = len(rows)
    for key in ['relative_stale', 'leader_state_stale', 'gt_missing', 'formation_missing',
                'localization_missing']:
        result[key + '_share'] = float(np.mean([row[key] for row in rows])) if rows else None
    result['per_target'] = {str(i): {
        key: scalar_stats(row[key] for row in rows if row['target_id'] == i)
        for key in ['localization_error_m', 'formation_error_m']}
        for i in range(1, 4)}
    if include_phases:
        active = [r for r in rows if r.get('controller_active') == 1]
        result['active_control'] = summarize(active, include_phases=False)
        result['following_by_formation'] = {
            name: summarize([r for r in active if r.get('formation_code') == code
                             and r.get('transitioning') == 0], include_phases=False)
            for code, name in [(1, 'line'), (2, 'column'), (3, 'wedge')]}
        result['mission'] = summarize([r for r in rows if r.get('mission_active') == 1], include_phases=False)
    return result
