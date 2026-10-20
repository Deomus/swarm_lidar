#!/usr/bin/env python3
"""Оценка запаздывания формы скорости ведомых по завершённой записи MCAP."""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def _samples(values):
    """Сортирует отметки измерений; последняя запись заменяет дубликат времени."""
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return np.empty((0, 2))
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError('Нужен массив пар (время измерения, скорость)')
    values = values[np.isfinite(values[:, 0])]
    values = values[np.argsort(values[:, 0], kind='stable')]
    if len(values) > 1:
        values = values[np.r_[values[1:, 0] != values[:-1, 0], True]]
    return values


def _interpolate(values, times, max_gap):
    """Интерполирует только внутри непрерывных участков конечных измерений."""
    result = np.interp(times, values[:, 0], values[:, 1])
    right = np.searchsorted(values[:, 0], times, side='left')
    right = np.clip(right, 0, len(values) - 1)
    exact = np.isclose(times, values[right, 0], rtol=0., atol=1e-9)
    left = np.maximum(right - 1, 0)
    gap = values[right, 0] - values[left, 0]
    valid = exact | ((right > 0) & (gap <= max_gap + 1e-12)
                     & np.isfinite(values[left, 1]) & np.isfinite(values[right, 1]))
    valid &= (times >= values[0, 0]) & (times <= values[-1, 0])
    result[exact] = values[right[exact], 1]
    result[~valid | ~np.isfinite(result)] = np.nan
    return result


def estimate_delay(leader, follower, dt=.05, max_lag=3., max_gap=.5, min_samples=40):
    """Ищет положительное запаздывание ведомого по нормированной корреляции.

    Входы — пары (секунды времени измерения, скорость вдоль X). Положительный
    лаг сопоставляет раннюю скорость ведущего с более поздней скоростью ведомого.
    Это сходство формы сигналов, а не измерение причинной задержки передачи.
    При недостатке данных или постоянной скорости lag_seconds равен None.
    """
    if (not all(math.isfinite(value) for value in (dt, max_lag, max_gap))
            or dt <= 0 or max_lag < 0 or max_gap <= 0
            or not isinstance(min_samples, int) or min_samples < 3):
        raise ValueError('Некорректные параметры оценки задержки')
    leader, follower = _samples(leader), _samples(follower)
    unavailable = {'lag_seconds': None, 'correlation': None, 'count': 0,
                   'reason': 'insufficient_data'}
    if len(leader) < 2 or len(follower) < 2:
        return unavailable
    start = max(leader[0, 0], follower[0, 0])
    end = min(leader[-1, 0], follower[-1, 0])
    if end <= start:
        return unavailable
    if not math.isfinite(end-start) or (end-start)/dt > 2_000_000:
        raise ValueError('Слишком длинное окно; задайте start/end одной временной эпохи')
    times = start + np.arange(math.floor((end - start) / dt) + 1) * dt
    leading = _interpolate(leader, times, max_gap)
    following = _interpolate(follower, times, max_gap)
    best = None
    saw_flat = False
    for lag in range(min(math.floor(max_lag / dt), len(times) - 1) + 1):
        a = leading[:len(times) - lag]
        b = following[lag:]
        good = np.isfinite(a) & np.isfinite(b)
        count = int(np.count_nonzero(good))
        if count < min_samples:
            continue
        a, b = a[good], b[good]
        a = a - a.mean()
        b = b - b.mean()
        if np.std(a) <= 1e-6 or np.std(b) <= 1e-6:
            saw_flat = True
            continue
        correlation = float(np.clip(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b)), -1., 1.))
        if best is None or correlation > best['correlation'] + 1e-12:
            best = {'lag_seconds': float(lag * dt), 'correlation': correlation,
                    'count': count, 'reason': None}
    if best is None and saw_flat:
        unavailable['reason'] = 'flat_signal'
    return best if best is not None else unavailable


def read_states(path, start=None, end=None):
    """Читает только /mesh/states; узлы ROS и живые подписки не создаются."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message

    from swarm_interfaces.msg import SwarmState

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/mesh/states']))
    streams = {i: [] for i in range(4)}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != '/mesh/states':
            continue
        state = deserialize_message(data, SwarmState)
        if state.uav_id not in streams:
            continue
        stamp = state.timestamp.sec + state.timestamp.nanosec * 1e-9
        if (not math.isfinite(stamp) or (start is not None and stamp < start)
                or (end is not None and stamp > end)):
            continue
        velocity = state.velocity.linear.x if state.valid and state.frame_id == 'map' else math.nan
        streams[state.uav_id].append((stamp, velocity))
    return streams


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path, help='Каталог завершённой записи MCAP или файл .mcap')
    parser.add_argument('--start', type=float, help='Начало участка, секунды времени измерений')
    parser.add_argument('--end', type=float, help='Конец участка, секунды времени измерений')
    parser.add_argument('--output', type=Path, required=True, help='Новый JSON; существующий не заменяется')
    args = parser.parse_args(argv)
    if any(value is not None and not math.isfinite(value) for value in (args.start, args.end)):
        parser.error('--start и --end должны быть конечными числами')
    if args.start is not None and args.end is not None and args.start >= args.end:
        parser.error('--start должен быть меньше --end')
    if args.output.exists():
        parser.error('Выходной файл уже существует')
    try:
        streams = read_states(args.bag, args.start, args.end)
        result = {'description': 'Оценка запаздывания формы скорости, не причинной задержки связи',
                  'topic': '/mesh/states', 'axis': 'map ENU X', 'time_source': 'SwarmState.timestamp',
                  'start': args.start, 'end': args.end, 'dt': .05, 'max_lag': 3., 'max_gap': .5,
                  'followers': {str(i): estimate_delay(streams[0], streams[i]) for i in (1, 2, 3)}}
        with args.output.open('x') as handle:
            json.dump(result, handle, ensure_ascii=False, indent=2)
            handle.write('\n')
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        print(f'Ошибка анализа: {error}', file=sys.stderr)
        return 1
    print(f'Результаты: {args.output}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
