import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'formation_controller'))

from leader_interface.core import LeaderCore, simplify_straight_path  # noqa: E402


def test_path_simplification_keeps_turns_reversals_and_vertical_changes():
    points = np.array([[0,0,3], [0,0,3], [1,0,3], [2,0,3],
                       [2,1,3], [2,2,3], [2,1,3], [2,1,4]], float)
    np.testing.assert_array_equal(simplify_straight_path(points),
        [[0,0,3], [2,0,3], [2,2,3], [2,1,3], [2,1,4]])


def test_arrival_requires_low_measured_speed():
    core = ready()
    core.set_path([[2,0,3]])
    core.select_mode('MISSION', 10)
    core.state([2,0,3], 10.1, True, 10.1, velocity=[.8,0,0])
    assert core.tick(10.1).status == 'MISSION'
    core.state([2,0,3], 10.2, True, 10.2, velocity=[.1,0,0])
    assert core.tick(10.2).status == 'COMPLETE'


def test_long_straight_segment_can_finish_without_false_timeout():
    core = LeaderCore()
    position = np.array([-80., -80., 3.])
    velocity = np.zeros(3)
    core.state(position, 0., True, 0., velocity=velocity)
    core.set_path([[-80., -80., 3.], [0., 0., 3.], [80., 80., 3.]])
    core.select_mode('MISSION', 0.)
    for step in range(5000):
        now = step * core.period
        core.state(position, now, True, now, velocity=velocity)
        result = core.tick(now)
        assert result.status != 'WAYPOINT_TIMEOUT'
        if result.status == 'COMPLETE':
            break
        velocity = result.velocity
        position += velocity * core.period
    else:
        pytest.fail('Длинный маршрут не завершился')
    assert np.linalg.norm(position - [80., 80., 3.]) <= core.tolerance


def test_dense_straight_route_avoids_braking_at_each_metre():
    core = ready()
    core.set_path([[x,0,3] for x in range(13)])
    core.select_mode('MISSION', 10)
    position, velocity = np.array([0.,0.,3.]), np.zeros(3)
    commands = [np.zeros(3)]
    for step in range(700):
        now = 10+step*.05
        core.state(position, now, True, now, velocity=velocity)
        output = core.tick(now)
        if output.status == 'COMPLETE':
            break
        commands.append(output.velocity)
        velocity += (output.velocity-velocity)*.05/.35
        position += velocity*.05
    assert output.status == 'COMPLETE'
    assert step*.05 < 30.
    assert np.linalg.norm(position-[12,0,3]) < .4
    assert np.linalg.norm(velocity) < .15
    commands = np.asarray(commands)
    assert commands[:,0].max() > .8
    assert abs(position[1]) < 1e-9
    accelerations = np.diff(commands, axis=0)/.05
    jerks = np.diff(np.vstack(([0.,0.,0.], accelerations)), axis=0)/.05
    assert np.linalg.norm(commands, axis=1).max() <= 1.5+1e-9
    assert np.linalg.norm(accelerations, axis=1).max() <= 1.+1e-9
    assert np.linalg.norm(jerks, axis=1).max() <= 2.+1e-9


def test_invalid_velocity_cannot_enable_mission():
    core = ready()
    core.state([0,0,3], 10.1, True, 10.1, velocity=[float('nan'),0,0])
    core.set_path([[2,0,3]])
    with pytest.raises(ValueError, match='state'):
        core.select_mode('MISSION', 10.1)


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


def test_revoked_path_stops_active_mission_and_cannot_be_restarted():
    core = ready()
    core.set_path([[20, 0, 3]])
    core.select_mode('MISSION', 10)
    assert core.tick(10).valid
    core.revoke_path()
    output = core.tick(10.1)
    assert output.status == 'PATH_REVOKED' and not output.valid
    np.testing.assert_array_equal(output.velocity, 0)
    with pytest.raises(ValueError, match='path'):
        core.select_mode('MISSION', 10.1)


def test_revoking_pending_path_does_not_interrupt_manual_control():
    core = ready()
    core.set_path([[20, 0, 3]])
    core.manual([1, 0, 0], 10, 10)
    core.select_mode('MANUAL', 10)
    core.revoke_path()
    assert core.tick(10.1).valid
    assert core.pending_path is None


def test_mission_converges_through_close_waypoints_with_delayed_velocity_response():
    core = ready()
    position = np.array([0., 0., 2.5])
    velocity = np.zeros(3)
    core.set_path([[-.5, -.5, 3], [.5, -.5, 3], [1.5, -.5, 3],
                   [2.5, -.5, 3], [3.5, .5, 3], [4.5, 1.5, 3],
                   [5.5, 2.5, 3], [6., 3., 3.]])
    core.select_mode('MISSION', 10)
    for step in range(1800):
        now = 10 + step * .05
        core.state(position, now, True, now)
        output = core.tick(now)
        # Упрощённый объект с постоянной времени 0.3 с, не модель PX4.
        velocity += (output.velocity - velocity) * .05 / .35
        position += velocity * .05
        if output.status == 'COMPLETE':
            break
    assert output.status == 'COMPLETE'
    for _ in range(100):
        velocity += -velocity * .05 / .35
        position += velocity * .05
    assert np.linalg.norm(position - [6, 3, 3]) < .5
    assert np.linalg.norm(velocity) < .15


def facing(yaw, waypoint, position=(0., 0., 3.)):
    core = LeaderCore()
    core.state(list(position), 10, True, 10, yaw=yaw)
    core.set_path([waypoint])
    core.select_mode('MISSION', 10)
    return core.tick(10.)


def test_mission_turns_in_place_before_moving_to_waypoint():
    output = facing(0., [0., 5., 3.])
    assert output.yaw_rate == .3 and output.valid
    np.testing.assert_array_equal(output.velocity, 0.)


def test_mission_moves_and_corrects_small_heading_error():
    output = facing(np.pi/2 - .2, [0., 5., 3.])
    assert output.yaw_rate == pytest.approx(.2)
    assert output.velocity[1] > 0.


def test_heading_error_uses_short_side_across_pi():
    output = facing(3., [-5., -.5, 3.])
    assert output.yaw_rate == pytest.approx(np.arctan2(-.5, -5.) + 2*np.pi - 3.)
    assert output.velocity[0] < 0.


def test_close_waypoint_or_unknown_yaw_keeps_heading():
    assert facing(0., [0., .8, 3.]).yaw_rate == 0.
    output = facing(None, [0., 5., 3.])
    assert output.yaw_rate == 0. and output.velocity[1] > 0.


def test_mission_faces_each_segment_and_completes_route():
    core = ready()
    position, velocity, yaw = np.array([0., 0., 3.]), np.zeros(3), 0.
    core.set_path([[0., 0., 3.], [0., 6., 3.], [-6., 6., 3.]])
    core.select_mode('MISSION', 10)
    headings = []
    for step in range(4000):
        now = 10 + step * .05
        core.state(position, now, True, now, velocity=velocity, yaw=yaw)
        output = core.tick(now)
        velocity += (output.velocity - velocity) * .05 / .35
        position += velocity * .05
        yaw += output.yaw_rate * .05
        if np.linalg.norm(velocity[:2]) > .5:
            headings.append(np.angle(np.exp(1j*(np.arctan2(velocity[1], velocity[0]) - yaw))))
        if output.status == 'COMPLETE':
            break
    assert output.status == 'COMPLETE'
    assert np.max(np.abs(headings)) < .35
    assert yaw == pytest.approx(np.pi, abs=.1)
