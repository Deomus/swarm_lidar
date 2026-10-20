import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'formation_controller'))

from leader_interface.core import LeaderCore  # noqa: E402


def ready(**parameters):
    core = LeaderCore(**parameters)
    assert core.state([0, 0, 3], 10, True, 10)
    return core


def test_default_hold_never_controls_followers_and_rejects_missing_state():
    core = LeaderCore()
    output = core.tick(10)
    assert not output.valid and output.status == 'HOLD'
    np.testing.assert_array_equal(output.velocity, 0)
    with pytest.raises(ValueError, match='state'):
        core.select_mode('MANUAL', 10)


@pytest.mark.parametrize('yaw_rate,expected', [(2, .3), (-2, -.3), (.1, .1)])
def test_manual_yaw_sign_rate_bound_and_deadman(yaw_rate, expected):
    core = ready()
    assert core.manual([1, 0, 0], 10, 10, yaw_rate=yaw_rate)
    core.select_mode('MANUAL', 10)
    output = core.tick(10)
    assert output.valid and output.yaw_rate == pytest.approx(expected)
    assert output.velocity[0] > 0
    output = core.tick(10.3)
    assert not output.valid and output.status == 'DEADMAN'
    assert output.yaw_rate == 0
    np.testing.assert_array_equal(output.velocity, 0)


def test_future_out_of_order_and_invalid_manual_not_poison_watermark():
    core = ready()
    assert not core.manual([1, 0, 0], 1000, 10)
    assert core.manual([1, 0, 0], 10, 10)
    assert not core.manual([1, 0, 0], 9.99, 10)
    assert not core.manual([np.nan, 0, 0], 10.1, 10.1)
    assert not core.manual([1, 0, 0], 10.1, 10.1, yaw_rate=np.nan)
    assert core.manual_stamp == 10


def test_state_stale_requires_explicit_mode_recovery():
    core = ready()
    core.manual([1, 0, 0], 10, 10)
    core.select_mode('MANUAL', 10)
    core.tick(10)
    output = core.tick(10.6)
    assert not output.valid and output.status == 'STATE_STALE' and core.mode == 'HOLD'
    core.state([0, 0, 3], 10.7, True, 10.7)
    core.manual([1, 0, 0], 10.7, 10.7)
    assert not core.tick(10.7).valid
    core.select_mode('MANUAL', 10.7)
    assert core.tick(10.75).valid


def test_mission_path_is_pending_until_explicit_mode_service_and_completes():
    core = ready()
    core.set_path([[0, 0, 3], [2, 0, 3]])
    assert not core.tick(10).valid
    core.select_mode('MISSION', 10)
    output = core.tick(10)
    assert output.status == 'MISSION' and output.waypoint_index == 1
    assert output.velocity[0] > 0 and output.yaw_rate == 0
    core.set_path([[50, 0, 3]])
    # Новая публикация маршрута не заменяет уже исполняемый маршрут.
    np.testing.assert_array_equal(core.path[-1], [2, 0, 3])
    core.state([2, 0, 3], 10.1, True, 10.1)
    output = core.tick(10.1)
    assert output.status == 'COMPLETE' and core.mode == 'HOLD' and not output.valid
    np.testing.assert_array_equal(output.velocity, 0)


def test_mission_timeout_and_path_validation():
    core = ready(waypoint_timeout=.2)
    for bad in ([], [[1, 2]], [[np.nan, 0, 3]]):
        with pytest.raises(ValueError):
            core.set_path(bad)
    with pytest.raises(ValueError, match='path'):
        core.select_mode('MISSION', 10)
    core.set_path([[20, 0, 3]])
    core.select_mode('MISSION', 10)
    output = core.tick(10.3)
    assert not output.valid and output.status == 'WAYPOINT_TIMEOUT'


def test_clock_rewind_clears_inputs_and_requires_explicit_recovery():
    core = ready()
    core.manual([1, 0, 0], 10, 10)
    core.select_mode('MANUAL', 10)
    core.tick(10)
    output = core.tick(1)
    assert output.status == 'CLOCK_RESET' and not output.valid
    assert core.state([0, 0, 3], 1.1, True, 1.1)
    assert core.manual([1, 0, 0], 1.1, 1.1)
    assert not core.tick(1.1).valid


def test_manual_velocity_limits_in_continuous_valid_operation():
    core = ready()
    core.select_mode('MANUAL', 10)
    velocities = [np.zeros(3)]
    for index in range(800):
        now = 10 + index * .05
        core.state([0, 0, 3], now, True, now)
        core.manual([100 * (-1 if index % 100 < 50 else 1), 0, 0], now, now)
        output = core.tick(now)
        assert output.valid
        velocities.append(output.velocity)
    velocity = np.array(velocities)
    acceleration = np.diff(velocity, axis=0) / .05
    jerk = np.diff(np.vstack(([0, 0, 0], acceleration)), axis=0) / .05
    assert np.linalg.norm(velocity, axis=1).max() <= 1.5 + 1e-9
    assert np.linalg.norm(acceleration, axis=1).max() <= 1 + 1e-9
    assert np.linalg.norm(jerk, axis=1).max() <= 2 + 1e-9
