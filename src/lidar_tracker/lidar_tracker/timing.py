"""История ориентации по времени измерения, без эталонных координат."""

from collections import deque
from threading import RLock

from scipy.spatial.transform import Rotation, Slerp


class OrientationHistory:
    def __init__(self, tolerance=.05, capacity=200):
        self.samples = deque(maxlen=capacity)
        self.tolerance = tolerance
        self.lock = RLock()

    def clear(self):
        with self.lock:
            self.samples.clear()

    def add(self, stamp, quaternion):
        rotation = Rotation.from_quat(quaternion)
        with self.lock:
            if self.samples and stamp < self.samples[-1][0]:
                return
            if self.samples and stamp == self.samples[-1][0]:
                self.samples.pop()
            self.samples.append((stamp, rotation.as_quat()))

    def at(self, stamp):
        with self.lock:
            samples = list(self.samples)
        if not samples:
            raise ValueError('Нет истории ориентации на время LiDAR-измерения')
        before = [row for row in samples if row[0] <= stamp]
        after = [row for row in samples if row[0] >= stamp]
        if before and after:
            a, b = before[-1], after[0]
            if a[0] == b[0]:
                return Rotation.from_quat(a[1])
            return Slerp([a[0], b[0]], Rotation.from_quat([a[1], b[1]]))([stamp])[0]
        nearest = samples[0] if not before else samples[-1]
        if abs(nearest[0] - stamp) > self.tolerance:
            raise ValueError('Ориентация не согласована со временем LiDAR')
        return Rotation.from_quat(nearest[1])
