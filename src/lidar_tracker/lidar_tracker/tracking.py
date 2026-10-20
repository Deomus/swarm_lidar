"""Фильтрация и сопровождение целей без ROS и ground truth."""
import numpy as np
from scipy.spatial import cKDTree


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
            for neighbour in tree.query_ball_point(data[current], tolerance):
                if not visited[neighbour]:
                    visited[neighbour] = True
                    queue.append(neighbour)
        if min_size <= len(indices) <= max_size:
            clusters.append(data[indices])
    return clusters


class TargetTracker:
    def __init__(self, seeds, gate=2.0, ambiguity=0.2, alpha=0.7, timeout=1.0):
        self.positions = np.asarray(seeds, dtype=float).reshape(-1, 3)
        self.velocity = np.zeros_like(self.positions)
        self.stamps = [None] * len(self.positions)
        self.gate, self.ambiguity = gate, ambiguity
        self.alpha, self.timeout = alpha, timeout

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
        return result
