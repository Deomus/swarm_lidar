"""Проверки прогноза и итоговых ограничений без ROS и физического симулятора."""

import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from formation_controller.core import VelocityLimiter
from formation_controller.prediction import LeaderVelocityPredictor


def test_constant_velocity_and_returned_arrays_are_independent():
    predictor = LeaderVelocityPredictor()
    velocity = np.array([.7, -.2, .1])
    for stamp in np.arange(0., 2., .05):
        output = predictor.update(velocity, stamp, stamp)
        np.testing.assert_array_equal(output, velocity)
        output[:] = 100
    np.testing.assert_array_equal(predictor.update(velocity, 2., 2.), velocity)


def test_ramp_estimates_acceleration_and_caps_correction():
    predictor = LeaderVelocityPredictor()
    for stamp in np.arange(0., 2., .05):
        velocity = np.array([stamp, 0., 0.])
        prediction = predictor.update(velocity, stamp, stamp)
        assert 0 <= prediction[0] - velocity[0] <= .6 + 1e-12
    assert prediction[0] - velocity[0] == pytest.approx(.6)


def test_stopping_and_noise_stay_bounded_and_settle():
    predictor = LeaderVelocityPredictor()
    rng = np.random.default_rng(84)
    for index in range(200):
        stamp = index * .05
        velocity = rng.normal(0., .2, 3) if index < 100 else np.zeros(3)
        prediction = predictor.update(velocity, stamp, stamp)
        assert np.linalg.norm(prediction - velocity) <= .6 + 1e-12
    assert np.linalg.norm(prediction) < 1e-6


def test_duplicate_and_old_measurements_do_not_create_acceleration():
    predictor = LeaderVelocityPredictor()
    predictor.update([0., 0., 0.], 1., 1.)
    expected = predictor.update([.1, 0., 0.], 1.1, 1.1)
    np.testing.assert_array_equal(predictor.update([10., 0., 0.], 1.1, 1.15), expected)
    np.testing.assert_array_equal(predictor.update([-10., 0., 0.], 1.05, 1.2), expected)
    result = predictor.update([.2, 0., 0.], 1.2, 1.2)
    assert .2 < result[0] <= .8


def test_rewind_gap_and_explicit_reset_prime_without_acceleration():
    predictor = LeaderVelocityPredictor()
    predictor.update([0., 0., 0.], 10., 10.)
    predictor.update([1., 0., 0.], 10.1, 10.1)
    np.testing.assert_array_equal(predictor.update([2., 0., 0.], 1., 1.), [2., 0., 0.])
    np.testing.assert_array_equal(predictor.update([3., 0., 0.], 2., 2.), [3., 0., 0.])
    predictor.reset()
    np.testing.assert_array_equal(predictor.update([4., 0., 0.], 2.1, 2.1), [4., 0., 0.])


@pytest.mark.parametrize('velocity,stamp,now', [
    ([float('nan'), 0., 0.], 1., 1.), ([0., 0.], 1., 1.),
    ([0., 0., 0.], float('inf'), 1.), ([0., 0., 0.], 1., float('nan')),
    ([0., 0., 0.], 2., 1.), ([0., 0., 0.], 1., 2.),
])
def test_invalid_inputs_clear_history(velocity, stamp, now):
    predictor = LeaderVelocityPredictor()
    predictor.update([0., 0., 0.], 1., 1.)
    predictor.update([.1, 0., 0.], 1.1, 1.1)
    with pytest.raises(ValueError):
        predictor.update(velocity, stamp, now)
    np.testing.assert_array_equal(predictor.update([1., 0., 0.], 3., 3.), [1., 0., 0.])


@pytest.mark.parametrize('parameters', [
    {'horizon': -1.}, {'max_correction': -1.}, {'max_acceleration': 0.},
    {'smoothing_time': 0.}, {'stale_timeout': 0.}, {'horizon': float('nan')},
])
def test_parameters_reject_invalid_values(parameters):
    with pytest.raises(ValueError):
        LeaderVelocityPredictor(**parameters)


@pytest.mark.parametrize('parameters', [{'horizon': 0.}, {'max_correction': 0.}])
def test_zero_prediction_setting_passes_velocity_unchanged(parameters):
    predictor = LeaderVelocityPredictor(**parameters)
    predictor.update([0., 0., 0.], 1., 1.)
    np.testing.assert_array_equal(predictor.update([1., 0., 0.], 1.1, 1.1), [1., 0., 0.])


def test_overflow_in_finite_input_difference_resets_history():
    predictor = LeaderVelocityPredictor()
    predictor.update([-1e308, 0., 0.], 1., 1.)
    with pytest.raises(ValueError, match='Переполнение'):
        predictor.update([1e308, 0., 0.], 1.1, 1.1)
    np.testing.assert_array_equal(predictor.update([1., 0., 0.], 1.2, 1.2), [1., 0., 0.])


def test_prediction_reduces_tracking_rms_in_ideal_smooth_motion():
    # Идеальная модель: команда ведомого сразу становится его скоростью.
    # Этот тест сравнивает запаздывание алгоритма, а не доказывает свойства PX4.
    period = .05
    predictor = LeaderVelocityPredictor()
    previous = VelocityLimiter(2., 4., 8., period)
    predicted = VelocityLimiter(2., 4., 8., period)
    leader_position = 0.
    positions = np.zeros(2)
    errors = []
    for index in range(2400):
        now = index * period
        velocity = np.array([np.sin(.25 * now), 0., 0.])
        forward = predictor.update(velocity, now, now)
        outputs = [previous.step(velocity + [.3 * (leader_position - positions[0]), 0., 0.]),
                   predicted.step(forward + [.3 * (leader_position - positions[1]), 0., 0.])]
        leader_position += velocity[0] * period
        positions += np.array([output[0] for output in outputs]) * period
        if now > 20.:
            errors.append(leader_position - positions.copy())
    rms = np.sqrt(np.mean(np.square(errors), axis=0))
    assert rms[1] < .7 * rms[0], rms


def test_original_final_speed_acceleration_and_jerk_bounds_remain():
    period = .05
    predictor = LeaderVelocityPredictor()
    limiter = VelocityLimiter(2., 4., 8., period)
    rng = np.random.default_rng(74)
    velocities = [np.zeros(3)]
    for index in range(1200):
        now = index * period
        leader_velocity = rng.normal(0., 2., 3)
        forward = predictor.update(leader_velocity, now, now)
        # Имитация произвольных поправок регулятора и геометрии перестроения.
        velocities.append(limiter.step(forward + rng.normal(0., 10., 3)))
    velocities = np.asarray(velocities)
    accelerations = np.diff(velocities, axis=0) / period
    jerks = np.diff(np.vstack(([0., 0., 0.], accelerations)), axis=0) / period
    assert np.linalg.norm(velocities, axis=1).max() <= 2. + 1e-9
    assert np.linalg.norm(accelerations, axis=1).max() <= 4. + 1e-9
    assert np.linalg.norm(jerks, axis=1).max() <= 8. + 1e-9
