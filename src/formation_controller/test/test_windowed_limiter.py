"""Границы команды и задержка конечного сглаживания, без модели физики PX4."""

import numpy as np
import pytest

from formation_controller.core import (
    RelativeVelocityController, VelocityLimiter, WindowedVelocityLimiter, slot_feedforward,
)
from formation_controller.prediction import LeaderVelocityPredictor


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


def test_following_translation_turns_and_stops_on_ideal_model():
    period = .05
    leader = VelocityLimiter(1.5, 1., 2., period)
    filters = [VelocityLimiter(2., 4., 8., period), WindowedVelocityLimiter(2., 4., 8., period)]
    predictors = [LeaderVelocityPredictor(horizon=horizon) for horizon in (1.9, .95)]
    offset = np.array([-4., -4., 0.])
    positions = np.tile(offset, (2, 1))
    leader_position, yaw, errors = np.zeros(3), 0., []
    for tick in range(1800):
        now = tick*period
        phase = now % 24
        speed = (.5, 1., 1.5)[min(2, int(now//24))]
        desired = ([speed, 0., 0.] if phase < 12 else [0., speed/2, .1] if phase < 16
                   else [0., 0., -.1] if phase < 20 else [0., 0., 0.])
        rate = .08 if 12 <= phase < 16 else -.08 if 16 <= phase < 20 else 0.
        velocity = leader.step(desired)
        leader_position += velocity*period
        yaw += rate*period
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        target = leader_position+rotation@offset
        forward = slot_feedforward(offset, np.zeros(3), velocity, yaw, rate)
        for index in range(2):
            predicted = predictors[index].update(forward, now, now)
            positions[index] += period*filters[index].step(predicted+.3*(target-positions[index]))
        errors.append(np.linalg.norm(target-positions, axis=1))
    errors = np.asarray(errors)
    rms = np.sqrt(np.mean(errors**2, axis=0))
    assert rms[1] < .7*rms[0]
    assert errors[:, 1].max() < .8*errors[:, 0].max()


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
