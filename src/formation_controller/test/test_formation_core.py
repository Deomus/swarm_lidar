import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from formation_controller.core import (  # noqa: E402
    FormationPlan,
    RelativeVelocityController,
    VelocityLimiter,
    formation_offsets,
    minimum_transition_distance,
)


def test_offsets_scale_and_leader_separation():
    np.testing.assert_allclose(formation_offsets('line'), [[-4, -4, 0], [-4, 0, 0], [-4, 4, 0]])
    for name in ('line', 'column', 'wedge'):
        base = formation_offsets(name)
        lower_scale = 2 / minimum_transition_distance(base, base)
        positions = formation_offsets(name, scale=lower_scale)
        assert minimum_transition_distance(positions, positions) >= 2 - 1e-9
        with pytest.raises(ValueError):
            formation_offsets(name, scale=lower_scale * .99)
    with pytest.raises(ValueError):
        formation_offsets('other')


def test_analytic_distance_finds_crossing_and_leader_collision():
    start = np.array([[-4, -4, 0], [-4, 4, 0], [-8, 0, 0]])
    assert minimum_transition_distance(start, start[[1, 0, 2]]) == 0
    end = start.copy()
    end[2] = [8, 0, 0]
    assert minimum_transition_distance(start, end) == 0


def test_analytic_distance_agrees_with_dense_sample():
    rng = np.random.default_rng(23)
    for _ in range(12):
        start, end = rng.normal(size=(2, 3, 3)) * 5
        analytic = minimum_transition_distance(start, end)
        dense = math.inf
        for fraction in np.linspace(0, 1, 2001):
            points = np.vstack(([0, 0, 0], start + fraction * (end - start)))
            distances = np.linalg.norm(points[:, None] - points[None, :], axis=2)
            dense = min(dense, float(distances[np.triu_indices(4, 1)].min()))
        assert analytic <= dense + 1e-9
        assert dense - analytic < 0.01


def test_transition_safe_assignment_endpoint_and_smoothness():
    plan = FormationPlan()
    order = plan.request('column', 1, 4, now=10)
    assert sorted(order) == [0, 1, 2]
    np.testing.assert_allclose(plan.sample(10).velocities, 0)
    np.testing.assert_allclose(plan.sample(14).velocities, 0)
    np.testing.assert_allclose(plan.sample(14).offsets, formation_offsets('column')[list(order)])
    for t in np.linspace(10, 14, 101):
        p = plan.sample(t).offsets
        assert minimum_transition_distance(p, p) >= 2 - 1e-9
    with pytest.raises(ValueError, match='active transition'):
        plan.request('wedge', 1, 4, now=12)


def test_failed_request_preserves_existing_plan():
    plan = FormationPlan()
    initial = plan.sample(5).offsets
    with pytest.raises(ValueError):
        plan.request('column', 0.1, 4, now=5)
    np.testing.assert_allclose(plan.sample(5).offsets, initial)
    with pytest.raises(ValueError):
        plan.request('column', 1, 0, now=5)


def test_safe_endpoints_but_no_safe_transition_is_rejected():
    plan = FormationPlan(min_separation=4)
    # Все конечные пары допустимы, но все 3! прямых путей слишком тесны.
    target = formation_offsets('column', min_separation=4)
    assert minimum_transition_distance(target, target) == 4
    initial = plan.sample(10).offsets
    with pytest.raises(ValueError, match='No collision-free'):
        plan.request('column', 1, 4, now=10)
    np.testing.assert_allclose(plan.sample(12).offsets, initial)


def test_transition_offset_velocity_matches_finite_difference():
    plan = FormationPlan()
    plan.request('wedge', 1.3, 5, now=10)
    h = 1e-5
    middle = plan.sample(12).velocities
    numerical = (plan.sample(12 + h).offsets - plan.sample(12 - h).offsets) / (2 * h)
    np.testing.assert_allclose(middle, numerical, atol=1e-8)


def test_limiter_speed_acceleration_and_jerk_under_reversals():
    limiter = VelocityLimiter(max_speed=3, max_acceleration=2, max_jerk=4, period=0.05)
    velocities = [np.zeros(3)]
    rng = np.random.default_rng(4)
    for n in range(1500):
        desired = rng.normal(size=3) * 100 if n % 30 else np.array([100, 0, 0])
        velocities.append(limiter.step(desired))
    velocities = np.asarray(velocities)
    acceleration = np.diff(velocities, axis=0) / 0.05
    jerk = np.diff(np.vstack(([0, 0, 0], acceleration)), axis=0) / 0.05
    assert np.linalg.norm(velocities, axis=1).max() <= 3 + 1e-9
    assert np.linalg.norm(acceleration, axis=1).max() <= 2 + 1e-9
    assert np.linalg.norm(jerk, axis=1).max() <= 4 + 1e-9


def test_braking_envelope_bounds_command_distance_and_settling_time():
    limiter = VelocityLimiter()
    for _ in range(400):
        limiter.step([3, 0, 0])
    distance_bound, time_bound = limiter.braking_envelope()
    distance = 0
    for n in range(2000):
        speed = np.linalg.norm(limiter.step([0, 0, 0]))
        distance += speed * limiter.period
        if (n + 1) * limiter.period >= time_bound:
            assert speed <= 0.05 + 1e-9
    assert distance <= distance_bound + 1e-9


def test_feedforward_flu_rotation_turn_and_error():
    controller = RelativeVelocityController()
    offset = np.array([-4.0, 0, 0])
    result = controller.step(offset + [1, 0, 0], offset, [0, 0, 0], [1, 0, 0],
                             math.pi / 2, 0.1, 10, 10)
    # ENU v = [1,0,0] + R90 * [-0.6,-0.4,0] = [1.4,-0.6,0].
    expected = np.array([1.4, -0.6, 0]) * controller.limiter.alpha ** 2
    np.testing.assert_allclose(result.velocity_enu, expected)
    assert result.safety_state == 'FOLLOWING'


@pytest.mark.parametrize('change', [
    {'valid': False}, {'measurement_stamp': 8}, {'measurement_stamp': 11},
    {'relative_flu': [np.nan, 0, 0]}, {'enabled': False}, {'confidence': 0.1},
])
def test_bad_measurement_cannot_create_follow_command(change):
    controller = RelativeVelocityController()
    args = dict(relative_flu=[-4, 0, 0], offset_flu=[-4, 0, 0],
                offset_velocity_flu=[0, 0, 0], leader_velocity_enu=[2, 0, 0],
                leader_yaw=0, leader_yaw_rate=0, measurement_stamp=10, now=10)
    args.update(change)
    result = controller.step(**args)
    assert result.safety_state == 'BRAKING'
    np.testing.assert_array_equal(result.velocity_enu, 0)


def test_out_of_order_measurement_rejected_and_stale_commands_brake():
    controller = RelativeVelocityController()
    args = dict(relative_flu=[-4, 0, 0], offset_flu=[-4, 0, 0],
                offset_velocity_flu=[0, 0, 0], leader_velocity_enu=[2, 0, 0],
                leader_yaw=0, leader_yaw_rate=0)
    for n in range(200):
        result = controller.step(**args, measurement_stamp=10 + n * .05, now=10 + n * .05)
    peak = np.linalg.norm(result.velocity_enu)
    result = controller.step(**args, measurement_stamp=19.9, now=20)
    assert result.safety_state == 'BRAKING'
    for _ in range(1000):
        result = controller.step(**args, measurement_stamp=19.9, now=21)
    assert np.linalg.norm(result.velocity_enu) < peak * .001
