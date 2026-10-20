"""Ограниченное упреждение скорости ведущего перед общим ограничителем команды."""

import math

import numpy as np


class LeaderVelocityPredictor:
    """Оценивает ускорение по новым измерениям и добавляет ограниченное упреждение.

    Результат сам по себе не ограничивает скорость, ускорение и рывок команды:
    после сложения с регулятором нужен общий ограничитель. Предсказание помогает
    при плавном движении, но не является полным обращением сглаживающего фильтра.
    """

    def __init__(self, horizon=1.9, max_acceleration=1., max_correction=.6,
                 smoothing_time=.3, stale_timeout=.5):
        parameters = {'horizon': horizon, 'max_acceleration': max_acceleration,
                      'max_correction': max_correction,
                      'smoothing_time': smoothing_time, 'stale_timeout': stale_timeout}
        for name, value in parameters.items():
            minimum_allowed = value >= 0 if name in ('horizon', 'max_correction') else value > 0
            if not math.isfinite(value) or not minimum_allowed:
                raise ValueError(f'Некорректный параметр {name}')
            setattr(self, name, float(value))
        self.reset()

    def reset(self):
        """Забывает историю; первое новое измерение проходит без упреждения."""
        self._velocity = None
        self._stamp = None
        self._now = None
        self._acceleration = np.zeros(3)
        self._prediction = None

    @staticmethod
    def _limited(vector, maximum):
        # Масштабирование сначала предотвращает переполнение нормы больших чисел.
        largest = float(np.max(np.abs(vector)))
        if largest == 0:
            return vector.copy()
        scaled = vector / largest
        scaled_norm = float(np.linalg.norm(scaled))
        if largest > maximum / scaled_norm:
            return scaled * (maximum / scaled_norm)
        return vector.copy()

    def update(self, velocity, stamp, now):
        """Возвращает прогноз; старые и повторные измерения не меняют оценку.

        Невалидные данные и устаревшее/будущее время сбрасывают историю и вызывают
        ValueError. При откате часов или большом разрыве первый свежий отсчёт
        начинает новую историю без производной через разрыв.
        """
        try:
            velocity = np.asarray(velocity, dtype=float)
            valid = (velocity.shape == (3,) and np.isfinite(velocity).all()
                     and math.isfinite(stamp) and math.isfinite(now))
        except (TypeError, ValueError, OverflowError):
            valid = False
        if not valid:
            self.reset()
            raise ValueError('Требуются конечные XYZ скорости и время')
        age = now - stamp
        if not math.isfinite(age) or not 0 <= age <= self.stale_timeout:
            self.reset()
            raise ValueError('Измерение скорости устарело или пришло из будущего')
        if self._now is not None and now < self._now:
            self.reset()
        self._now = now
        if self._stamp is not None and stamp <= self._stamp:
            return self._prediction.copy()
        if self._stamp is None or stamp - self._stamp > self.stale_timeout:
            self._acceleration = np.zeros(3)
            prediction = velocity.copy()
        else:
            dt = stamp - self._stamp
            # Разность конечных скоростей тоже может переполниться.
            with np.errstate(over='ignore', invalid='ignore', divide='ignore'):
                acceleration = (velocity - self._velocity) / dt
            if not np.isfinite(acceleration).all():
                self.reset()
                raise ValueError('Переполнение при оценке ускорения')
            acceleration = self._limited(acceleration, self.max_acceleration)
            weight = -math.expm1(-dt / self.smoothing_time)
            self._acceleration += weight * (acceleration - self._acceleration)
            # Сначала ограничиваем ускорение, чтобы произведение не переполнилось.
            correction_acceleration = self._limited(
                self._acceleration,
                self.max_correction / self.horizon if self.horizon else 0.)
            correction = correction_acceleration * self.horizon
            with np.errstate(over='ignore', invalid='ignore'):
                prediction = velocity + correction
            if not np.isfinite(prediction).all():
                self.reset()
                raise ValueError('Переполнение при прогнозе скорости')
        self._velocity = velocity.copy()
        self._stamp = stamp
        self._prediction = prediction.copy()
        return prediction.copy()
