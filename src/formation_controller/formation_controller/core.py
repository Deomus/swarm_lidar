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
        # Треугольник ведомых: центральная вершина ближе к ведущему.
        'wedge': [[-2, -1, 0], [-1, 0, 0], [-2, 1, 0]],
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


class WindowedVelocityLimiter:
    """Два скользящих средних с конечной памятью и теми же границами команды.

    Для входа с нормой <= V и окна N отсчётов:
    |dv/dt| <= 2V/(NT), |d²v/dt²| <= 4V/(NT)².
    Усреднение сохраняет ограничение нормы скорости. Начальная история нулевая.
    В отличие от двух экспоненциальных звеньев задержка равна (N-1)T,
    а после 2N нулевых входов остаток команды исчезает полностью.
    """

    def __init__(self, max_speed=3., max_acceleration=2., max_jerk=4., period=.05):
        self.max_speed = _positive(max_speed, 'max_speed')
        self.period = _positive(period, 'period')
        acceleration = _positive(max_acceleration, 'max_acceleration')
        jerk = _positive(max_jerk, 'max_jerk')
        self.window = max(1, math.ceil(max(2*self.max_speed/(acceleration*self.period),
            math.sqrt(4*self.max_speed/jerk)/self.period)))
        self.delay = (self.window-1)*self.period
        self._first = np.zeros((self.window, 3))
        self._second = np.zeros((self.window, 3))
        self._first_sum = np.zeros(3)
        self._second_sum = np.zeros(3)
        self._index = 0
        self.velocity = np.zeros(3)

    def step(self, desired):
        desired = np.asarray(desired, dtype=float)
        if desired.shape != (3,) or not np.isfinite(desired).all():
            raise ValueError('Expected finite velocity vector')
        desired = clip_norm(desired, self.max_speed)
        index = self._index
        self._first_sum += desired-self._first[index]
        self._first[index] = desired
        intermediate = self._first_sum/self.window
        self._second_sum += intermediate-self._second[index]
        self._second[index] = intermediate
        self._index = (index+1) % self.window
        self.velocity = self._second_sum/self.window
        return self.velocity.copy()


@dataclass(frozen=True)
class FormationSample:
    offsets: np.ndarray
    velocities: np.ndarray
    transitioning: bool


class FormationPlan:
    """Выбор безопасной перестановки slots; request не меняет план при отказе."""

    def __init__(self, min_separation=2.0, gap=4.0, max_slot_speed=None):
        self.min_separation = _positive(min_separation, 'min_separation')
        self.gap = _positive(gap, 'gap')
        self.max_slot_speed = None if max_slot_speed is None else _positive(max_slot_speed, 'max_slot_speed')
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
        if self.max_slot_speed is not None:
            duration = max(duration, 1.5 * float(np.linalg.norm(end-current.offsets, axis=1).max()) / self.max_slot_speed)
        self._start, self._end = current.offsets, end.copy()
        self._time, self._duration = float(now), float(duration)
        return order


def limit_group_motion(velocity, yaw_rate, offsets, offset_velocities, heading, max_speed):
    """Ограничивает движение ведущего по скорости всех заданных мест.

    Геометрия перестроения не масштабируется: она задаётся отдельным планом.
    Общий коэффициент сохраняет соотношение поступательного движения и поворота.
    """
    c, s = math.cos(heading), math.sin(heading)
    rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
    offsets, offset_velocities = _points(offsets), _points(offset_velocities)
    factor = 1.
    for offset, offset_velocity in zip(offsets, offset_velocities):
        a = np.asarray(velocity) + rotation @ np.cross([0., 0., yaw_rate], offset)
        b = rotation @ offset_velocity
        if np.linalg.norm(b) > max_speed + 1e-6:
            raise ValueError('Скорость перестроения превышает возможности ведомых')
        aa, ab = float(a @ a), float(a @ b)
        if aa > 1e-12:
            bound = (-ab + math.sqrt(max(0., ab*ab-aa*(float(b @ b)-max_speed**2)))) / aa
            factor = min(factor, max(0., bound))
    return np.asarray(velocity) * factor, yaw_rate * factor, factor


def wrap_angle(angle):
    """Угол в [-π, π]: ошибка курса через ±π берётся по короткой стороне."""
    return math.remainder(angle, math.tau)


def heading_rate(leader_yaw, leader_yaw_rate, follower_yaw, gain, max_rate):
    """Скорость рыскания ведомого за курсом лидера: упреждение и P по ошибке, ENU."""
    rate = leader_yaw_rate + gain * wrap_angle(leader_yaw - follower_yaw)
    return float(np.clip(rate, -max_rate, max_rate))


def clip_norm(vector, limit):
    norm = float(np.linalg.norm(vector))
    return vector * (limit / norm) if norm > limit else vector.copy()


def slot_feedforward(offset, offset_velocity, leader_velocity, yaw, yaw_rate):
    """Скорость места в ENU: перенос ведущего, перестроение и вращение строя."""
    c, s = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    turn_velocity = yaw_rate * np.array([-offset[1], offset[0], 0])
    return leader_velocity + rotation @ (offset_velocity + turn_velocity)


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

    def stopping_displacement(self):
        """Сумма будущих команд при нулевом входе, без модели физики PX4."""
        return self.period * (1-self.alpha) / self.alpha * (self.velocity + self._intermediate)


@dataclass(frozen=True)
class ControlResult:
    velocity_enu: np.ndarray
    error_flu: np.ndarray
    safety_state: str


class RelativeVelocityController:
    """Горизонтальный P, вертикальный PD и упреждение без абсолютных поз ведомого."""

    def __init__(self, kp=0.6, stale_timeout=0.3, min_confidence=0.5,
                 max_speed=3.0, max_acceleration=2.0, max_jerk=4.0, period=0.05,
                 vertical_kp=None, vertical_kd=0., vertical_derivative_time=.2,
                 velocity_smoothing='exponential'):
        self.kp = _positive(kp, 'kp')
        self.vertical_kp = self.kp if vertical_kp is None else _positive(vertical_kp, 'vertical_kp')
        if not math.isfinite(vertical_kd) or vertical_kd < 0:
            raise ValueError('vertical_kd must be finite and nonnegative')
        self.vertical_kd = vertical_kd
        self.vertical_derivative_time = _positive(vertical_derivative_time, 'vertical_derivative_time')
        self._z_stamp, self._z_value, self._z_rate = -math.inf, None, 0.
        self.stale_timeout = _positive(stale_timeout, 'stale_timeout')
        if not math.isfinite(min_confidence) or not 0 <= min_confidence <= 1:
            raise ValueError('min_confidence must be in [0, 1]')
        self.min_confidence = min_confidence
        if velocity_smoothing not in ('exponential', 'windowed'):
            raise ValueError('Unknown velocity_smoothing')
        limiter_type = WindowedVelocityLimiter if velocity_smoothing == 'windowed' else VelocityLimiter
        self.limiter = limiter_type(max_speed, max_acceleration, max_jerk, period)
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
            desired = slot_feedforward(offset, offset_velocity, leader_velocity,
                                       leader_yaw, leader_yaw_rate)
            if measurement_stamp > self._z_stamp:
                dt = measurement_stamp-self._z_stamp
                if self._z_value is not None and dt <= self.stale_timeout:
                    measured_rate = (r[2]-self._z_value)/dt
                    alpha = dt/(self.vertical_derivative_time+dt)
                    self._z_rate += alpha*(measured_rate-self._z_rate)
                else:
                    self._z_rate = 0.
                self._z_stamp, self._z_value = measurement_stamp, r[2]
            correction = self.kp * (offset-r)
            correction[2] = self.vertical_kp * (offset[2]-r[2]) - self.vertical_kd * (self._z_rate-offset_velocity[2])
            desired += rotation @ correction
        else:
            self._z_stamp, self._z_value, self._z_rate = -math.inf, None, 0.
        velocity = self.limiter.step(desired)
        return ControlResult(velocity, error, 'FOLLOWING' if good else 'BRAKING')
