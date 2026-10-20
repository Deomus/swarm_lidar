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


def relative_lidar(leader, target, sensor_offset=(0., 0., .45), marker_offset=(0., 0., .45)):
    marker_world = np.asarray(target.position) + rotation(target.quaternion) @ marker_offset
    return (rotation(leader.quaternion).T @ (marker_world - leader.position)
            - np.asarray(sensor_offset))


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


def summarize(rows):
    def metric(key):
        return scalar_stats(row[key] for row in rows)
    result = {key: metric(key) for key in ['localization_error_m', 'formation_error_m',
                                         'min_pair_distance_m']}
    result['rows'] = len(rows)
    for key in ['relative_stale', 'leader_state_stale', 'gt_missing', 'formation_missing',
                'localization_missing']:
        result[key + '_share'] = float(np.mean([row[key] for row in rows])) if rows else None
    result['per_target'] = {str(i): {
        key: scalar_stats(row[key] for row in rows if row['target_id'] == i)
        for key in ['localization_error_m', 'formation_error_m']}
        for i in range(1, 4)}
    return result
