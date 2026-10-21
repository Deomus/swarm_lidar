"""Консервативная проверка всего перестроения по известной карте."""

from math import floor

import numpy as np
from mission_planner.grid import PlanningError, inflated_grid


def check_transition_map(grid, position, start, end, margin=.75):
    if grid is None or grid.header.frame_id != 'map':
        raise ValueError('Нет карты площадки для проверки перестроения')
    info = grid.info
    q = info.origin.orientation
    if abs(q.x)+abs(q.y)+abs(q.z) > 1e-6 or abs(abs(q.w)-1.) > 1e-6:
        raise ValueError('Карта должна иметь нулевой поворот начала координат')
    if info.width * info.height > 40000:
        raise ValueError('Карта слишком велика для проверки перестроения')
    radius = max(float(np.linalg.norm(points, axis=1).max()) for points in (start, end)) + margin
    blocked = inflated_grid(info.width, info.height, info.resolution, grid.data, radius)
    cell = (floor((position[0]-info.origin.position.x)/info.resolution),
            floor((position[1]-info.origin.position.y)/info.resolution))
    if not 0 <= cell[0] < info.width or not 0 <= cell[1] < info.height or cell in blocked:
        raise PlanningError('Недостаточно свободного места для всего перестроения')
