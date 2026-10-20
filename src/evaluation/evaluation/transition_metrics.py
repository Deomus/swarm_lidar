"""Измерения перестроения по позам, без ROS и без данных трекера."""

import math

import numpy as np

from .metrics import scalar_stats


def interpolate_offsets(times, offsets, query, max_gap=.2):
    """Линейное согласование соседних заданий; разрывы не заполняются."""
    times = np.asarray(times, dtype=float)
    offsets = np.asarray(offsets, dtype=float)
    query = np.asarray(query, dtype=float)
    if (times.ndim != 1 or query.ndim != 1 or offsets.shape != (len(times), 3, 3)
            or not np.isfinite(times).all() or not np.isfinite(offsets).all()
            or not np.isfinite(query).all() or np.any(np.diff(times) <= 0)
            or not math.isfinite(max_gap) or max_gap <= 0):
        raise ValueError('Некорректные отметки времени или задания')
    result = np.full((len(query), 3, 3), np.nan)
    if not len(times):
        return result
    right = np.searchsorted(times, query)
    for i, index in enumerate(right):
        if index < len(times) and abs(times[index] - query[i]) <= 1e-9:
            result[i] = offsets[index]
        elif 0 < index < len(times) and times[index] - times[index-1] <= max_gap + 1e-12:
            fraction = (query[i] - times[index-1]) / (times[index] - times[index-1])
            result[i] = offsets[index-1] + fraction * (offsets[index] - offsets[index-1])
    return result


def settling_time(times, errors, start, end, threshold=.3, dwell=1., max_gap=.2):
    """Начало конечного непрерывного участка в полосе, длиной хотя бы dwell.

    Учитываются только измерения после окончания движения заданного слота.
    Выход из полосы или пропуск > max_gap обрывает подтверждение. После
    последнего измерения результат не экстраполируется.
    """
    times, errors = np.asarray(times, float), np.asarray(errors, float)
    if (times.ndim != 1 or errors.shape != times.shape
            or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
            or not all(math.isfinite(v) for v in (start, end, threshold, dwell, max_gap))
            or end < start or min(threshold, dwell, max_gap) <= 0):
        raise ValueError('Некорректные параметры установления')
    selected = (times >= start - 1e-9) & (times <= end + 1e-9)
    times, errors = times[selected], errors[selected]
    if not len(times) or end - times[-1] > max_gap + 1e-9:
        return None
    beginning = None
    for i, (stamp, error) in enumerate(zip(times, errors)):
        if not np.isfinite(error) or error > threshold:
            beginning = None
            continue
        if beginning is None or (i and stamp - times[i-1] > max_gap + 1e-9):
            beginning = float(stamp)
    return beginning if beginning is not None and times[-1] - beginning >= dwell - 1e-9 else None


def _peak(times, values, actual):
    index = int(np.argmax(values))
    return dict(signed_peak_m=float(values[index]),
                overshoot_m=max(0., float(values[index])),
                time=float(times[index]), actual_heading_m=actual[index].tolist())


def analyze_transition(times, actual, desired, start_slots, final_slots, start,
                       plan_end, end, threshold=.3, dwell=1., max_gap=.2,
                       reaction_threshold=.05, reaction_dwell=.2):
    """Одна принятая смена слотов; строки actual/desired — аппараты 1, 2, 3.

    Все координаты заданы в heading ведущего: X вперёд, Y влево, Z вверх.
    signed_peak = max((actual-final)·unit(final-start)), поэтому отрицательный
    пик означает, что конечную плоскость ни разу не пересекли. Нулевое
    перемещение слота не имеет определённого направления перелёта.
    """
    times = np.asarray(times, float)
    actual, desired = np.asarray(actual, float), np.asarray(desired, float)
    initial, final = np.asarray(start_slots, float), np.asarray(final_slots, float)
    if (times.ndim != 1 or actual.shape != (len(times), 3, 3) or desired.shape != actual.shape
            or initial.shape != (3, 3) or final.shape != (3, 3)
            or not np.isfinite(times).all() or np.any(np.diff(times) <= 0)
            or not np.isfinite(initial).all() or not np.isfinite(final).all()
            or not all(math.isfinite(v) for v in
                       (start, plan_end, end, threshold, dwell, max_gap,
                        reaction_threshold, reaction_dwell))
            or not start <= plan_end <= end
            or min(threshold, dwell, max_gap, reaction_threshold, reaction_dwell) <= 0):
        raise ValueError('Некорректные данные перестроения')
    selected = (times >= start - 1e-9) & (times <= end + 1e-9)
    times, actual, desired = times[selected], actual[selected], desired[selected]
    valid = np.isfinite(actual).all(axis=(1, 2)) & np.isfinite(desired).all(axis=(1, 2))
    dynamic = np.linalg.norm(actual - desired, axis=2)
    final_error = np.linalg.norm(actual - final, axis=2)
    final_error[~valid] = np.nan
    joint = settling_time(times, np.max(final_error, axis=1), plan_end, end,
                          threshold, dwell, max_gap)
    result = dict(start_time=start, target_end_time=plan_end, observation_end_time=end,
                  target_duration_s=plan_end-start,
                  initial_slots_heading_m=initial.tolist(), final_slots_heading_m=final.tolist(),
                  threshold_m=threshold, required_tail_s=dwell, max_gap_s=max_gap,
                  samples=int(len(times)), matched_samples=int(valid.sum()),
                  settled_time=joint, settling_duration_s=None if joint is None else joint-start,
                  post_target_settling_s=None if joint is None else joint-plan_end,
                  followers={})
    for index in range(3):
        good = valid
        stamps, positions = times[good], actual[good, index]
        delta = final[index] - initial[index]
        distance = float(np.linalg.norm(delta))
        steady = settling_time(times, final_error[:, index], plan_end, end,
                               threshold, dwell, max_gap)
        item = dict(travel_m=distance, dynamic_slot_error_m=scalar_stats(dynamic[good, index]),
                    final_slot_error_m=scalar_stats(final_error[good, index]),
                    settled_time=steady, settling_duration_s=None if steady is None else steady-start,
                    projection=None, axes={}, reaction_time=None, reaction_delay_s=None,
                    reaction_threshold_m=reaction_threshold, reaction_dwell_s=reaction_dwell)
        if len(stamps) and distance > 1e-9:
            direction = delta / distance
            item['projection'] = _peak(stamps, (positions-final[index]) @ direction, positions)
            item['projection']['direction_heading'] = direction.tolist()
            # Нельзя приписывать отсутствие ранних измерений задержке движения.
            if stamps[0] - start <= max_gap + 1e-9:
                moved = (positions-positions[0]) @ direction
                beginning = None
                for k, stamp in enumerate(stamps):
                    if moved[k] <= reaction_threshold + 1e-9:
                        beginning = None
                        continue
                    if beginning is None or (k and stamp-stamps[k-1] > max_gap + 1e-9):
                        beginning = float(stamp)
                    if stamp-beginning >= reaction_dwell-1e-9:
                        item['reaction_time'] = beginning
                        item['reaction_delay_s'] = beginning-start
                        break
        for axis, name in enumerate(('x', 'y', 'z')):
            item['axes'][name] = (_peak(stamps, np.sign(delta[axis]) *
                                      (positions[:, axis]-final[index, axis]), positions)
                                  if len(stamps) and abs(delta[axis]) > 1e-9 else None)
        result['followers'][str(index+1)] = item
    return result
