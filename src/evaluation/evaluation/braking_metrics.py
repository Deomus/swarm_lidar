"""Наблюдаемая остановка по эталонным координатам без ROS."""

import math

import numpy as np

from .metrics import scalar_stats


def velocity_from_positions(times, positions, max_gap=.2):
    """Секущая скорость на середине интервала; через пропуски — NaN."""
    times, positions = np.asarray(times, float), np.asarray(positions, float)
    if (times.ndim != 1 or positions.shape != (len(times), 3)
            or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
            or not math.isfinite(max_gap) or max_gap <= 0):
        raise ValueError('Некорректные координаты или время')
    dt = np.diff(times)
    velocity = np.diff(positions, axis=0) / dt[:, None]
    good = (dt <= max_gap + 1e-9) & np.isfinite(velocity).all(axis=1)
    velocity[~good] = np.nan
    return (times[:-1]+times[1:])/2., velocity


def _value_at(times, values, stamp, max_gap):
    if not len(times):
        return None
    right = np.searchsorted(times, stamp)
    if right < len(times) and abs(times[right]-stamp) <= 1e-9:
        return values[right].copy() if np.isfinite(values[right]).all() else None
    if (not 0 < right < len(times) or times[right]-times[right-1] > max_gap + 1e-9
            or not np.isfinite(values[right-1:right+1]).all()):
        return None
    u = (stamp-times[right-1])/(times[right]-times[right-1])
    return values[right-1]*(1-u)+values[right]*u


def first_low_speed(times, speeds, start, end, threshold=.15, dwell=.5, max_gap=.2):
    """Первый подтверждённый участок низкой скорости, целиком внутри окна."""
    times, speeds = np.asarray(times, float), np.asarray(speeds, float)
    if (times.ndim != 1 or times.shape != speeds.shape or not np.isfinite(times).all()
            or np.any(np.diff(times) <= 0)
            or not all(math.isfinite(v) for v in (start, end, threshold, dwell, max_gap))
            or end < start or min(threshold, dwell, max_gap) <= 0):
        raise ValueError('Некорректные параметры проверки остановки')
    beginning, previous = None, None
    for stamp, speed in zip(times, speeds):
        if stamp < start-1e-9 or stamp > end+1e-9:
            continue
        if not math.isfinite(speed) or speed > threshold + 1e-9:
            beginning = None
        else:
            if beginning is None or (previous is not None and stamp-previous > max_gap+1e-9):
                beginning = float(stamp)
            if stamp-beginning >= dwell-1e-9:
                return dict(start_time=beginning, confirmed_time=float(stamp))
        previous = stamp
    return None


def _distance(times, positions, start, end, max_gap):
    first = _value_at(times, positions, start, max_gap)
    last = _value_at(times, positions, end, max_gap)
    if first is None or last is None:
        return None
    middle = (times > start) & (times < end)
    stamps = np.r_[start, times[middle], end]
    path = np.vstack((first, positions[middle], last))
    if np.any(np.diff(stamps) > max_gap+1e-9) or not np.isfinite(path).all():
        return None
    return dict(path_length_m=float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum()),
                displacement_m=float(np.linalg.norm(last-first)),
                start_position_map_m=first.tolist(), end_position_map_m=last.tolist())


def analyze_braking(times, positions, loss_time, zero_time, safety_end,
                    speed_threshold=.15, dwell=.5, motion_threshold=.3,
                    pre_window=1., max_gap=.2):
    """Отделяет измерение остановки от проверки торможения в движении.

    Остановка может наблюдаться и у уже неподвижного аппарата. В этом
    случае braking_time_s и braking_distance_m остаются None. Сильное
    снижение скорости ещё до потери данных также запрещает приписывать
    остановку началу safety. Это диагностическое сравнение, не доказательство
    причинности даже при успешной проверке.
    """
    values = (loss_time, zero_time, safety_end, speed_threshold, dwell,
              motion_threshold, pre_window, max_gap)
    if (not all(math.isfinite(v) for v in values)
            or not loss_time <= zero_time <= safety_end
            or min(speed_threshold, dwell, motion_threshold, pre_window, max_gap) <= 0
            or motion_threshold <= speed_threshold):
        raise ValueError('Некорректные параметры торможения')
    times, positions = np.asarray(times, float), np.asarray(positions, float)
    velocity_times, velocities = velocity_from_positions(times, positions, max_gap)
    speeds = np.linalg.norm(velocities, axis=1)
    initial_velocity = _value_at(velocity_times, velocities, loss_time, max_gap)
    zero_velocity = _value_at(velocity_times, velocities, zero_time, max_gap)
    end_velocity = _value_at(velocity_times, velocities, safety_end, max_gap)
    initial_speed = None if initial_velocity is None else float(np.linalg.norm(initial_velocity))
    zero_speed = None if zero_velocity is None else float(np.linalg.norm(zero_velocity))
    end_speed = None if end_velocity is None else float(np.linalg.norm(end_velocity))
    before = (velocity_times >= loss_time-pre_window) & (velocity_times < loss_time)
    early = before & (velocity_times < loss_time-pre_window/2)
    late = before & (velocity_times >= loss_time-pre_window/2)
    pre = scalar_stats(speeds[before])
    early_values, late_values = speeds[early], speeds[late]
    early_values, late_values = early_values[np.isfinite(early_values)], late_values[np.isfinite(late_values)]
    early_speed = float(np.median(early_values)) if len(early_values) else None
    late_speed = float(np.median(late_values)) if len(late_values) else None
    complete_pre = (len(velocity_times[before]) >= 3
                    and velocity_times[before][0]-(loss_time-pre_window) <= max_gap
                    and loss_time-velocity_times[before][-1] <= max_gap
                    and np.isfinite(speeds[before]).all()
                    and np.all(np.diff(velocity_times[before]) <= max_gap+1e-9))
    reason = None
    if not complete_pre or initial_speed is None or zero_speed is None:
        reason = 'insufficient_initial_motion_data'
    elif initial_speed < motion_threshold or late_speed is None or late_speed < motion_threshold:
        reason = 'insufficient_initial_motion'
    elif early_speed-late_speed > max(.1, .2*early_speed):
        reason = 'already_decelerating_before_loss'
    elif zero_speed < motion_threshold:
        reason = 'insufficient_motion_at_zero_command'
    stop = first_low_speed(velocity_times, speeds, zero_time, safety_end,
                           speed_threshold, dwell, max_gap)
    if reason is None and stop is None:
        reason = 'stop_not_confirmed_during_safety'
    distance = None if stop is None else _distance(times, positions, zero_time,
                                                  stop['start_time'], max_gap)
    if reason is None and distance is None:
        reason = 'missing_stop_path'
    result = dict(loss_time=loss_time, first_fresh_zero_time=zero_time,
                  command_reaction_delay_s=zero_time-loss_time,
                  continuous_safety_end_time=safety_end, safety_duration_s=safety_end-zero_time,
                  initial_speed_at_loss_mps=initial_speed, speed_at_zero_command_mps=zero_speed,
                  speed_at_safety_end_mps=end_speed,
                  safety_speed_mps=scalar_stats(speeds[(velocity_times >= zero_time)
                                                      & (velocity_times <= safety_end)]),
                  pre_loss_speed_mps=pre, pre_loss_early_median_mps=early_speed,
                  pre_loss_late_median_mps=late_speed,
                  stop_speed_threshold_mps=speed_threshold, required_stop_dwell_s=dwell,
                  initial_motion_threshold_mps=motion_threshold,
                  physical_braking_verified=reason is None, unverified_reason=reason,
                  observed_low_speed=stop,
                  braking_time_s=None if reason else stop['start_time']-zero_time,
                  braking_distance_m=None if reason else distance['path_length_m'],
                  observed_path_to_low_speed=distance,
                  path_during_safety=_distance(times, positions, zero_time, safety_end, max_gap))
    return result, velocity_times, velocities
