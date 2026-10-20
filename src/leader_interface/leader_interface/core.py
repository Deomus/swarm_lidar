"""Логика режимов без зависимости от ROS и абсолютных поз ведомых."""

import math
from dataclasses import dataclass

import numpy as np

from formation_controller.core import VelocityLimiter, clip_norm


def vector3(value):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError('Expected finite XYZ')
    return vector.copy()


def fresh(stamp, now, timeout):
    return math.isfinite(stamp) and math.isfinite(now) and 0 <= now - stamp <= timeout


@dataclass(frozen=True)
class LeaderOutput:
    velocity: np.ndarray
    yaw_rate: float
    valid: bool
    source_stamp: float
    status: str
    waypoint_index: int


class LeaderCore:
    def __init__(self, period=.05, max_speed=1.5, max_acceleration=1., max_jerk=2.,
                 manual_timeout=.25, state_timeout=.5, waypoint_tolerance=.4,
                 waypoint_gain=.25, waypoint_timeout=90., max_yaw_rate=.3):
        parameters = [period, max_speed, max_acceleration, max_jerk, manual_timeout,
                      state_timeout, waypoint_tolerance, waypoint_gain, waypoint_timeout,
                      max_yaw_rate]
        if not all(math.isfinite(v) and v > 0 for v in parameters):
            raise ValueError('Parameters must be positive and finite')
        self.period = period
        self.max_speed = max_speed
        self.max_yaw_rate = max_yaw_rate
        self.manual_timeout = manual_timeout
        self.state_timeout = state_timeout
        self.tolerance = waypoint_tolerance
        self.gain = waypoint_gain
        self.waypoint_timeout = waypoint_timeout
        self.limits = dict(max_speed=max_speed, max_acceleration=max_acceleration,
                           max_jerk=max_jerk, period=period)
        self.limiter = VelocityLimiter(**self.limits)
        self.mode = 'HOLD'
        self.status = 'HOLD'
        self.position = None
        self.state_stamp = -math.inf
        self.state_valid = False
        self.manual_velocity = np.zeros(3)
        self.manual_yaw_rate = 0.
        self.manual_stamp = -math.inf
        self.pending_path = None
        self.path = None
        self.index = 0
        self.waypoint_started = 0.
        self.last_tick = None

    def state(self, position, stamp, valid, now):
        if not fresh(stamp, now, self.state_timeout) or stamp < self.state_stamp:
            return False
        try:
            position = vector3(position)
        except ValueError:
            valid = False
            position = np.zeros(3)
        self.position, self.state_stamp, self.state_valid = position, stamp, bool(valid)
        return True

    def manual(self, velocity, stamp, now, yaw_rate=0.):
        if not fresh(stamp, now, self.manual_timeout) or stamp < self.manual_stamp:
            return False
        try:
            velocity = vector3(velocity)
        except ValueError:
            return False
        if not math.isfinite(yaw_rate):
            return False
        self.manual_velocity, self.manual_stamp = velocity, stamp
        self.manual_yaw_rate = float(np.clip(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate))
        return True

    def set_path(self, waypoints):
        points = np.asarray(waypoints, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0 or not np.isfinite(points).all():
            raise ValueError('Path must contain finite XYZ waypoints')
        self.pending_path = points.copy()

    def revoke_path(self):
        """Пустой путь планировщика отзывает ожидающую и текущую миссию."""
        self.pending_path = self.path = None
        self.index = 0
        if self.mode == 'MISSION':
            self.mode, self.status = 'HOLD', 'PATH_REVOKED'
            self.limiter = VelocityLimiter(**self.limits)

    def select_mode(self, mode, now):
        if mode not in ('HOLD', 'MANUAL', 'MISSION'):
            raise ValueError('Unknown mode')
        if mode != 'HOLD' and not (self.state_valid and fresh(self.state_stamp, now, self.state_timeout)):
            raise ValueError('No fresh valid leader state')
        if mode == 'MISSION':
            if self.pending_path is None:
                raise ValueError('No accepted mission path')
            self.path = self.pending_path.copy()
            self.index, self.waypoint_started = 0, now
        self.mode = self.status = mode

    def reset_time(self):
        self.mode, self.status = 'HOLD', 'CLOCK_RESET'
        self.state_stamp = self.manual_stamp = -math.inf
        self.state_valid = False
        self.limiter = VelocityLimiter(**self.limits)
        self.last_tick = None
        self.pending_path = self.path = None

    def tick(self, now):
        if not math.isfinite(now):
            raise ValueError('Nonfinite clock')
        if self.last_tick is not None and now < self.last_tick:
            self.reset_time()
        self.last_tick = now
        desired = np.zeros(3)
        yaw_rate = 0.
        stamp = now
        valid = False
        if self.mode != 'HOLD' and not (self.state_valid and fresh(self.state_stamp, now, self.state_timeout)):
            self.mode, self.status = 'HOLD', 'STATE_STALE'
        if self.mode == 'MANUAL':
            valid = fresh(self.manual_stamp, now, self.manual_timeout)
            self.status = 'MANUAL' if valid else 'DEADMAN'
            if valid:
                desired = self.manual_velocity
                yaw_rate = self.manual_yaw_rate
                stamp = min(self.manual_stamp, self.state_stamp)
        elif self.mode == 'MISSION':
            while self.index < len(self.path) and np.linalg.norm(self.path[self.index] - self.position) <= self.tolerance:
                self.index += 1
                self.waypoint_started = now
            if self.index == len(self.path):
                self.mode, self.status = 'HOLD', 'COMPLETE'
            elif now - self.waypoint_started > self.waypoint_timeout:
                self.mode, self.status = 'HOLD', 'WAYPOINT_TIMEOUT'
            else:
                desired = clip_norm(self.gain * (self.path[self.index] - self.position), self.max_speed)
                self.status, valid, stamp = 'MISSION', True, self.state_stamp
        velocity = self.limiter.step(desired)
        # HOLD/deadman — явное запрещение дальнейшего движения, не скрытое следование.
        if not valid:
            velocity = np.zeros(3)
            self.limiter = VelocityLimiter(**self.limits)
        return LeaderOutput(velocity, yaw_rate, valid, stamp, self.status, self.index)
