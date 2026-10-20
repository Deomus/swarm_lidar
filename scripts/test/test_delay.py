"""Проверки оценки сдвига скорости на искусственных измерениях."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('analyze_delay', Path(__file__).parents[1] / 'analyze_delay.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
estimate_delay = module.estimate_delay


def signal(times):
    return np.sin(.9 * times) + .5 * np.sin(2.13 * times) + .25 * np.cos(.31 * times)


@pytest.mark.parametrize('delay', [0., .35, 1.2, 2.95])
def test_known_positive_delay(delay):
    times = np.arange(0., 40., .05)
    leading = np.column_stack((times, signal(times)))
    following = np.column_stack((times, signal(times - delay)))
    result = estimate_delay(leading, following)
    assert result['lag_seconds'] == pytest.approx(delay, abs=.05)
    assert result['correlation'] > .99
    assert result['count'] >= 700


def test_demeaning_and_positive_scaling_do_not_change_delay():
    times = np.arange(0., 30., .05)
    result = estimate_delay(np.column_stack((times, signal(times) + 12.)),
                            np.column_stack((times, 3. * signal(times - .7) - 5.)))
    assert result['lag_seconds'] == pytest.approx(.7)
    assert result['correlation'] > .999


def test_flat_speed_has_no_identifiable_delay():
    times = np.arange(0., 10., .05)
    result = estimate_delay(np.column_stack((times, np.ones(len(times)))),
                            np.column_stack((times, np.ones(len(times)) * 2.)))
    assert result['lag_seconds'] is None
    assert result['reason'] == 'flat_signal'


def test_large_gaps_are_not_interpolated_into_fake_observations():
    times = np.array([0., .05, 10., 10.05])
    values = np.column_stack((times, signal(times)))
    result = estimate_delay(values, values)
    assert result['lag_seconds'] is None
    assert result['reason'] == 'insufficient_data'


def test_missing_sections_still_allow_delay_from_other_valid_sections():
    times = np.arange(0., 40., .05)
    a = np.column_stack((times, signal(times)))
    b = np.column_stack((times, signal(times - .5)))
    a = a[(times < 10.) | (times > 15.)]
    b = b[(times < 22.) | (times > 26.)]
    result = estimate_delay(a, b)
    assert result['lag_seconds'] == pytest.approx(.5)
    assert result['correlation'] > .999
    assert result['count'] < 650


def test_nan_and_short_streams_are_unavailable():
    times = np.arange(0., 10., .05)
    values = np.column_stack((times, signal(times)))
    bad = values.copy()
    bad[:, 1] = np.nan
    for left, right in [(values, bad), ([], values), (values[:3], values),
                        (values, values + [100., 0.])]:
        result = estimate_delay(left, right)
        assert result['lag_seconds'] is None


def test_negative_delay_is_never_reported_as_negative_lag():
    times = np.arange(0., 20., .05)
    result = estimate_delay(np.column_stack((times, signal(times))),
                            np.column_stack((times, signal(times + .3))))
    assert 0. <= result['lag_seconds'] <= 3.


def test_sorting_duplicates_and_nan_time_do_not_create_shift():
    times = np.arange(0., 10., .05)
    values = np.column_stack((times, signal(times)))
    disordered = np.vstack((values[::-1], [np.nan, 0.], values[10]))
    result = estimate_delay(values, disordered)
    assert result['lag_seconds'] == 0.
    assert result['correlation'] > .999


@pytest.mark.parametrize('parameters', [{'dt': 0.}, {'max_gap': -1.},
                                         {'max_lag': -1.}, {'min_samples': 2}])
def test_invalid_parameters(parameters):
    with pytest.raises(ValueError):
        estimate_delay([], [], **parameters)
