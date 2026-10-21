"""Фильтрация и сопровождение целей без ROS и ground truth."""
import numpy as np
from scipy.spatial import cKDTree


def level_heights(points, rotation, sensor_offset):
    """Высота для отбора с компенсацией наклона; исходные XYZ не меняются."""
    offset = np.asarray(sensor_offset, dtype=float)
    return np.asarray(points) @ rotation.as_matrix()[2] + rotation.apply(offset)[2] - offset[2]


def remove_ground(points, distance=.04, min_points=1000, fraction=.7):
    """Удаляет только доминирующую плоскость под датчиком по RANSAC.

    Выборка ограничивает стоимость подбора. Маленькие облака корпусов не
    считаются землёй; нужны многочисленные согласованные отражения.
    Возвращает исходные точки, если уверенного плоского фона нет.
    """
    data = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(data) < min_points:
        return data
    rng = np.random.default_rng(7)
    sample = data[rng.choice(len(data), min(1024, len(data)), replace=False)]
    best, best_count = None, 0
    for _ in range(32):
        a, b, c = sample[rng.choice(len(sample), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal /= length
        if normal[2] < 0:
            normal = -normal
        offset = -float(normal @ a)
        # Земля ниже датчика; вертикальные стены и очень наклонные плоскости
        # не удаляем. Предел наклона около 35 градусов.
        if normal[2] < .82 or offset < 1.:
            continue
        count = np.count_nonzero(np.abs(sample @ normal + offset) <= distance)
        if count > best_count:
            best, best_count = (normal, offset), count
    if best is None or best_count < fraction * len(sample):
        return data
    normal, offset = best
    inliers = np.abs(np.einsum('ij,j->i', data, normal) + offset) <= distance
    if np.count_nonzero(inliers) < min_points:
        return data
    return data[~inliers]


def cluster_points(points, tolerance=0.8, min_size=3, max_size=5000):
    data = np.asarray(points, dtype=float).reshape(-1, 3)
    data = data[np.isfinite(data).all(axis=1)]
    if not len(data):
        return []
    tree = cKDTree(data)
    visited = np.zeros(len(data), dtype=bool)
    clusters = []
    for start in range(len(data)):
        if visited[start]:
            continue
        visited[start] = True
        queue, indices = [start], []
        while queue:
            current = queue.pop()
            indices.append(current)
            # Плотный фон не должен занимать весь период обработки. При
            # превышении бюджета отклоняем кадр целиком, не создавая ложные
            # маленькие кластеры из остатка большой компоненты.
            if len(indices) > max_size or tree.query_ball_point(data[current], tolerance, return_length=True) > max_size:
                return []
            for neighbour in tree.query_ball_point(data[current], tolerance):
                if not visited[neighbour]:
                    visited[neighbour] = True
                    queue.append(neighbour)
        if min_size <= len(indices) <= max_size:
            clusters.append(data[indices])
    return clusters


class TargetTracker:
    def __init__(self, seeds, gate=2.0, ambiguity=0.2, alpha=0.7, timeout=1.0):
        self.positions = np.asarray(seeds, dtype=float).reshape(-1, 3).copy()
        self.initial_positions = self.positions.copy()
        self.last_complete_positions = None
        self.last_complete_stamp = None
        self.velocity = np.zeros_like(self.positions)
        self.stamps = [None] * len(self.positions)
        self.gate, self.ambiguity = gate, ambiguity
        self.alpha, self.timeout = alpha, timeout

    def reset(self, now):
        """Повторный поиск по последнему полному LiDAR-кадру с сохранением ID.

        Сбрасываем скорость и возраст треков, но не забываем расположение
        аппаратов после перестроения. При возврате часов к новому миру
        сохранённая расстановка больше не применима.
        """
        if self.last_complete_stamp is not None and now < self.last_complete_stamp:
            self.last_complete_positions = None
            self.last_complete_stamp = None
        retained = self.last_complete_positions is not None
        self.positions = (self.last_complete_positions if retained else self.initial_positions).copy()
        self.velocity = np.zeros_like(self.positions)
        self.stamps = [None] * len(self.positions)
        return retained

    def update(self, centers, stamp):
        centers = np.asarray(centers, dtype=float).reshape(-1, 3)
        result = [None] * len(self.positions)
        if not len(centers):
            return result
        predicted = self.positions.copy()
        for i, last in enumerate(self.stamps):
            if last is not None and 0 < stamp - last <= self.timeout:
                predicted[i] += self.velocity[i] * (stamp - last)
        distances = np.linalg.norm(predicted[:, None] - centers[None, :], axis=2)
        candidates = []
        for i, row in enumerate(distances):
            last = self.stamps[i]
            if last is not None and (stamp <= last or stamp - last > self.timeout):
                continue
            order = np.argsort(row)
            if row[order[0]] > self.gate:
                continue
            if len(order) > 1 and row[order[1]] - row[order[0]] < self.ambiguity:
                continue
            candidates.append((i, int(order[0])))
        # Две цели, претендующие на один кластер, обе считаются неоднозначными.
        for i, j in candidates:
            if sum(k == j for _, k in candidates) != 1:
                continue
            last = self.stamps[i]
            position = centers[j] if last is None else (
                self.alpha * centers[j] + (1 - self.alpha) * predicted[i])
            if last is not None:
                self.velocity[i] = (position - self.positions[i]) / (stamp - last)
            self.positions[i], self.stamps[i] = position, stamp
            result[i] = (position.copy(), j)
        if all(item is not None for item in result):
            self.last_complete_positions = self.positions.copy()
            self.last_complete_stamp = stamp
        return result
