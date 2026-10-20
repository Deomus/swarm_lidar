"""Готовность группы по свежему статусу контроллера, без координат аппаратов."""

import math


class FormationGuard:
    READY_MODES = frozenset(('FOLLOWING', 'FORMATION_CHANGE'))

    def __init__(self, required=False, status_timeout=.5, grace=1.):
        if (not isinstance(required, bool)
                or not math.isfinite(status_timeout) or status_timeout <= 0
                or not math.isfinite(grace) or grace < 0):
            raise ValueError('Некорректные параметры контроля готовности формации')
        self.required = required
        self.status_timeout, self.grace = status_timeout, grace
        self.last_time = None
        self.reset()

    def reset(self):
        self.status_stamp = -math.inf
        self.status_ready = False
        self.unavailable_since = None

    def observe_time(self, now):
        if not math.isfinite(now):
            raise ValueError('Время должно быть конечным')
        if self.last_time is not None and now < self.last_time:
            self.reset()
        self.last_time = now

    def update(self, status, now):
        """Принимает только свежий статус с новой исходной отметкой времени."""
        self.observe_time(now)
        if not isinstance(status, dict):
            return False
        stamp, mode = status.get('time'), status.get('mode')
        if (not isinstance(stamp, (int, float)) or isinstance(stamp, bool)
                or not math.isfinite(stamp) or not isinstance(mode, str) or not mode
                or not 0 <= now - stamp <= self.status_timeout
                or stamp <= self.status_stamp):
            return False
        self.status_stamp, self.status_ready = stamp, mode in self.READY_MODES
        if self.status_ready:
            self.unavailable_since = None
        return True

    def ready(self, now):
        self.observe_time(now)
        return (not self.required or self.status_ready
                and 0 <= now - self.status_stamp <= self.status_timeout)

    def require_ready(self, now):
        if not self.ready(now):
            raise ValueError('FORMATION_UNAVAILABLE: нет свежей готовой формации')

    def should_hold(self, active, now):
        """Считает только непрерывную неготовность при активном ведущем."""
        ready = self.ready(now)
        if not self.required or not active or ready:
            self.unavailable_since = None
            return False
        if self.unavailable_since is None:
            self.unavailable_since = now
        return now - self.unavailable_since >= self.grace
