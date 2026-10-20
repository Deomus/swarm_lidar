"""Чистое ядро: ведомые нумеруются строками 0..2 (uav_1..3)."""

import math
from dataclasses import dataclass
from itertools import combinations, permutations

import numpy as np


def _positive(value, name):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be finite and positive')
    return float(value)


def _points(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3, 3) or not np.isfinite(result).all():
        raise ValueError('Expected finite (3, 3) follower offsets')
    return result.copy()


def formation_offsets(formation, scale=1.0, gap=4.0, min_separation=2.0):
    """Offsets в heading FLU; scale отклоняется при недостаточной дистанции."""
    scale = _positive(scale, 'scale')
    gap = _positive(gap, 'gap')
    _positive(min_separation, 'min_separation')
    shapes = {
        'line': [[-1, -1, 0], [-1, 0, 0], [-1, 1, 0]],
        'column': [[-1, 0, 0], [-2, 0, 0], [-3, 0, 0]],
        'wedge': [[-1, -1, 0], [-2, 0, 0], [-1, 1, 0]],
    }
    if formation not in shapes:
        raise ValueError(f'Unknown formation: {formation}')
    offsets = np.asarray(shapes[formation], dtype=float) * gap * scale
    if minimum_transition_distance(offsets, offsets) < min_separation - 1e-9:
        raise ValueError('Scale violates min_separation including leader')
    return offsets


def minimum_transition_distance(start, end):
    """Точный минимум парных расстояний на прямых путях, включая лидера."""
    start = np.vstack((np.zeros(3), _points(start)))
    end = np.vstack((np.zeros(3), _points(end)))
    result = math.inf
    for i, j in combinations(range(4), 2):
        p = start[i] - start[j]
        d = end[i] - end[j] - p
        denominator = float(d @ d)
        fraction = np.clip(-float(p @ d) / denominator, 0, 1) if denominator else 0
        result = min(result, float(np.linalg.norm(p + fraction * d)))
    return result


@dataclass(frozen=True)
class FormationSample:
    offsets: np.ndarray
    velocities: np.ndarray
    transitioning: bool


class FormationPlan:
    """Выбор безопасной перестановки slots; request не меняет план при отказе."""

    def __init__(self, min_separation=2.0, gap=4.0):
        self.min_separation = _positive(min_separation, 'min_separation')
        self.gap = _positive(gap, 'gap')
        self._start = formation_offsets('line', gap=gap, min_separation=min_separation)
        self._end = self._start.copy()
        self._time = 0.0
        self._duration = 1.0

    def pause(self, duration):
        """Сдвигает временную шкалу без изменения геометрии и назначения слотов."""
        if not math.isfinite(duration) or duration < 0:
            raise ValueError('Pause duration must be finite and nonnegative')
        shifted = self._time + duration
        if not math.isfinite(shifted):
            raise ValueError('Pause time overflow')
        self._time = shifted

    def sample(self, now):
        if not math.isfinite(now):
            raise ValueError('Nonfinite time')
        u = float(np.clip((now - self._time) / self._duration, 0, 1))
        s = u * u * (3 - 2 * u)
        ds = 6 * u * (1 - u) / self._duration
        return FormationSample(
            self._start + s * (self._end - self._start),
            ds * (self._end - self._start),
            bool(np.any(self._start != self._end) and 0 <= now - self._time < self._duration),
        )

    def request(self, formation, scale, duration, now):
        """Возвращает назначение slots; бросает ValueError при отказе."""
        _positive(duration, 'duration')
        current = self.sample(now)
        if current.transitioning and np.any(current.velocities):
            raise ValueError('Wait for active transition before requesting another')
        target = formation_offsets(formation, scale, self.gap, self.min_separation)
        candidates = []
        for order in permutations(range(3)):
            end = target[list(order)]
            if minimum_transition_distance(current.offsets, end) >= self.min_separation - 1e-9:
                cost = float(np.linalg.norm(end - current.offsets, axis=1).sum())
                candidates.append((cost, order, end))
        if not candidates:
            raise ValueError('No collision-free straight transition exists')
        _, order, end = min(candidates, key=lambda candidate: (candidate[0], candidate[1]))
        self._start, self._end = current.offsets, end.copy()
        self._time, self._duration = float(now), float(duration)
        return order


def clip_norm(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector * (limit / norm) if norm > limit else vector.copy()


class VelocityLimiter:
    """Два полюса с доказуемыми дискретными bounds при постоянном period."""

    def __init__(self, max_speed=3.0, max_acceleration=2.0, max_jerk=4.0, period=0.05):
        self.max_speed = _positive(max_speed, 'max_speed')
        self.period = _positive(period, 'period')
        a = _positive(max_acceleration, 'max_acceleration')
        j = _positive(max_jerk, 'max_jerk')
        self.alpha = min(1.0, a * period / (2 * max_speed),
                         math.sqrt(j * period * period / (4 * max_speed)))
        self._intermediate = np.zeros(3)
        self.velocity = np.zeros(3)

    def step(self, desired):
        desired = np.asarray(desired, dtype=float)
        if desired.shape != (3,) or not np.isfinite(desired).all():
            raise ValueError('Expected finite velocity vector')
        desired = clip_norm(desired, self.max_speed)
        self._intermediate += self.alpha * (desired - self._intermediate)
        self.velocity += self.alpha * (self._intermediate - self.velocity)
        return self.velocity.copy()

    def braking_envelope(self, tolerance=0.05):
        """Верхняя оценка пути команд до нуля и времени до tolerance м/с.

        Не учитывает физику PX4, задержки и исходную ошибку скорости.
        """
        _positive(tolerance, 'tolerance')
        q = 1 - self.alpha
        v = float(np.linalg.norm(self.velocity))
        w = float(np.linalg.norm(self._intermediate))
        distance = self.period * q / self.alpha * (v + w)
        steps = 0
        # Нормы ограничены аналитической огибающей двух полюсов.
        while max(v, w) > tolerance:
            w *= q
            v = q * v + self.alpha * w
            steps += 1
        return distance, steps * self.period


@dataclass(frozen=True)
class ControlResult:
    velocity_enu: np.ndarray
    error_flu: np.ndarray
    safety_state: str


class RelativeVelocityController:
    """P + feedforward. Никаких абсолютных поз ведомого в API."""

    def __init__(self, kp=0.6, stale_timeout=0.3, min_confidence=0.5,
                 max_speed=3.0, max_acceleration=2.0, max_jerk=4.0, period=0.05):
        self.kp = _positive(kp, 'kp')
        self.stale_timeout = _positive(stale_timeout, 'stale_timeout')
        if not math.isfinite(min_confidence) or not 0 <= min_confidence <= 1:
            raise ValueError('min_confidence must be in [0, 1]')
        self.min_confidence = min_confidence
        self.limiter = VelocityLimiter(max_speed, max_acceleration, max_jerk, period)
        self.last_stamp = -math.inf

    def step(self, relative_flu, offset_flu, offset_velocity_flu, leader_velocity_enu,
             leader_yaw, leader_yaw_rate, measurement_stamp, now, valid=True,
             confidence=1.0, enabled=True):
        """Недостоверные данные плавно тормозят до нулевой ENU-команды."""
        vectors = [np.asarray(v, dtype=float) for v in
                   (relative_flu, offset_flu, offset_velocity_flu, leader_velocity_enu)]
        finite = all(v.shape == (3,) and np.isfinite(v).all() for v in vectors)
        finite = finite and all(math.isfinite(v) for v in
                                (leader_yaw, leader_yaw_rate, measurement_stamp, now, confidence))
        fresh = finite and 0 <= now - measurement_stamp <= self.stale_timeout
        ordered = finite and measurement_stamp >= self.last_stamp
        good = enabled and valid and fresh and ordered and confidence >= self.min_confidence
        if finite and ordered and measurement_stamp <= now:
            self.last_stamp = measurement_stamp
        error = vectors[1] - vectors[0] if finite else np.zeros(3)
        desired = np.zeros(3)
        if good:
            r, offset, offset_velocity, leader_velocity = vectors
            c, s = math.cos(leader_yaw), math.sin(leader_yaw)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            turn_velocity = leader_yaw_rate * np.array([-offset[1], offset[0], 0])
            desired = leader_velocity + rotation @ (self.kp * (offset - r)
                                                    + offset_velocity + turn_velocity)
        velocity = self.limiter.step(desired)
        return ControlResult(velocity, error, 'FOLLOWING' if good else 'BRAKING')
