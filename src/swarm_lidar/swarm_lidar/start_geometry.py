"""Стартовая расстановка из одного YAML: мосты PX4 и начальные треки LiDAR."""

import math
from pathlib import Path

import yaml


def load_origins(path):
    """Места старта uav_0..uav_3 в map ENU, метры."""
    origins = yaml.safe_load(Path(path).read_text())['uav_origins_enu']
    if (not isinstance(origins, list) or len(origins) != 4
            or any(not isinstance(row, list) or len(row) != 3 for row in origins)):
        raise ValueError('uav_origins_enu: нужны четыре точки XYZ')
    result = [[float(value) for value in row] for row in origins]
    if not all(math.isfinite(value) for row in result for value in row):
        raise ValueError('uav_origins_enu содержит нечисловые координаты')
    return result


def tracker_seeds(origins, sensor_offset):
    """Ожидаемые центры ведомых в uav_0/lidar_link при ровном лидере с yaw=0."""
    leader = origins[0]
    return [origin[axis] - leader[axis] - float(sensor_offset[axis])
            for origin in origins[1:] for axis in range(3)]
