"""Границы команды и задержка конечного сглаживания, без модели физики PX4."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from formation_controller.core import (
    RelativeVelocityController, VelocityLimiter, WindowedVelocityLimiter,
)


@pytest.mark.parametrize('speed,acceleration,jerk,period', [
    (2., 4., 8., .05), (3., 2., 4., .02), (1., 1e4, 1e6, .1)])
def test_vector_bounds_with_reversals_and_arbitrary_inputs(speed, acceleration, jerk, period):
    limiter = WindowedVelocityLimiter(speed, acceleration, jerk, period)
    rng = np.random.default_rng(41)
    inputs = np.vstack((np.tile([speed, 0., 0.], (150, 1)),
                        np.tile([-speed, 0., 0.], (150, 1)),
                        rng.normal(0., 10., (1500, 3)), np.zeros((300, 3))))
    velocities = np.vstack((np.zeros(3), [limiter.step(value) for value in inputs]))
    accelerations = np.diff(velocities, axis=0)/period
    jerks = np.diff(np.vstack((np.zeros(3), accelerations)), axis=0)/period
    assert np.linalg.norm(velocities, axis=1).max() <= speed+1e-8
    assert np.linalg.norm(accelerations, axis=1).max() <= acceleration+1e-8
    assert np.linalg.norm(jerks, axis=1).max() <= jerk+1e-8


def test_shorter_delay_and_finite_settling_without_raising_limits():
    old, new = VelocityLimiter(2., 4., 8.), WindowedVelocityLimiter(2., 4., 8.)
    responses = np.array([[old.step([1., 0., 0.])[0], new.step([1., 0., 0.])[0]]
                          for _ in range(300)])
    # Интеграл отставания скорости равен задержке постоянного переноса.
    lag = .05*np.sum(1.-responses, axis=0)
    assert lag[0] == pytest.approx(1.9, abs=1e-4)
    assert lag[1] == pytest.approx(.95)
    assert new.delay == pytest.approx(.95)
    for _ in range(2*new.window):
        result = new.step([0., 0., 0.])
    np.testing.assert_allclose(result, 0., atol=1e-12)


@pytest.mark.parametrize('value', [[np.nan, 0., 0.], [1., 2.], [np.inf, 0., 0.]])
def test_invalid_input_preserves_filter_state(value):
    actual, reference = WindowedVelocityLimiter(), WindowedVelocityLimiter()
    actual.step([1., 2., 0.])
    reference.step([1., 2., 0.])
    with pytest.raises(ValueError):
        actual.step(value)
    np.testing.assert_array_equal(actual.step([0., 0., 0.]), reference.step([0., 0., 0.]))


def test_new_smoothing_preserves_stale_and_confidence_gates():
    controller = RelativeVelocityController(velocity_smoothing='windowed')
    args = dict(relative_flu=[-4., 0., 0.], offset_flu=[-4., 0., 0.],
                offset_velocity_flu=[0., 0., 0.], leader_velocity_enu=[1., 0., 0.],
                leader_yaw=0., leader_yaw_rate=0., measurement_stamp=1., now=1.)
    assert controller.step(**args).safety_state == 'FOLLOWING'
    assert controller.step(**dict(args, now=2.)).safety_state == 'BRAKING'
    assert controller.step(**dict(args, measurement_stamp=2., now=2., confidence=.1)).safety_state == 'BRAKING'
    for tick in range(200):
        output = controller.step(**dict(args, measurement_stamp=2., now=3.+tick*.05))
    np.testing.assert_allclose(output.velocity_enu, 0., atol=1e-12)


def test_vertical_loop_damps_delayed_sensor_and_vehicle_response():
    # Прежняя модель не включала задержку измерения и команды. С новым
    # фильтром усиление 1/1.2 возбуждало колебания, найденные в длинном S1.
    path = Path(__file__).parents[1]/'config/controller.yaml'
    configured = yaml.safe_load(path.read_text())['formation_controller']['ros__parameters']
    settings = {key: configured[key] for key in ('kp', 'vertical_kp', 'vertical_kd',
        'velocity_smoothing', 'stale_timeout', 'max_speed', 'max_acceleration', 'max_jerk',
        'period', 'vertical_derivative_time')}
    period = settings['period']
    lag_steps, sample_steps = max(1, round(.2/period)), max(1, round(.1/period))
    results = []
    for parameters in [dict(settings, velocity_smoothing='windowed', vertical_kp=1., vertical_kd=1.2),
                       settings]:
        controller = RelativeVelocityController(**parameters)
        history, commands = [.4]*(lag_steps+2), [0.]*(lag_steps+2)
        relative_z, velocity, measured, stamp = .4, 0., .4, 0.
        errors = []
        for tick in range(round(80/period)):
            now = tick*period
            if tick % sample_steps == 0:
                measured, stamp = history[-1-lag_steps], now-lag_steps*period
            result = controller.step([-4., 0., measured], [-4., 0., 0.],
                [0., 0., 0.], [0., 0., 0.], 0., 0., stamp, now)
            commands.append(result.velocity_enu[2])
            velocity += period/.45*(commands[-1-lag_steps]-velocity)
            unobserved_drift = .35 if 5 < now < 15 else -.35 if 15 < now < 25 else 0.
            relative_z += period*(velocity-unobserved_drift)
            history.append(relative_z)
            errors.append(relative_z)
        results.append(np.asarray(errors))
    tail = round(20/period)
    assert np.sqrt(np.mean(results[0][-tail:]**2)) > .5
    assert np.sqrt(np.mean(results[1][-tail:]**2)) < .01
    assert np.max(np.abs(results[1])) < .8
