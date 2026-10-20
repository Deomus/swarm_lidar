"""Нулевая команда, низкая скорость и торможение в движении — разные факты."""

import numpy as np
import pytest

from evaluation.braking_metrics import analyze_braking, first_low_speed, velocity_from_positions


def decelerating_path():
    times = np.arange(-.5, 4.001, .01)
    after = np.clip(times-1., 0., 1.)
    x = np.where(times <= 1., times, 1.+after-.5*after**2)
    return times, np.column_stack((x, times*0., times*0.+3.))


def test_velocity_uses_actual_irregular_intervals_and_midpoint_time():
    times = np.array([0., .03, .1, .19, .3])
    direction = np.array([1., -2., .5])
    stamps, velocity = velocity_from_positions(times, times[:, None]*direction)
    np.testing.assert_allclose(stamps, (times[1:]+times[:-1])/2.)
    np.testing.assert_allclose(velocity, np.broadcast_to(direction, (4, 3)))


def test_gap_does_not_become_a_fake_low_velocity_interval():
    times = np.array([0., .1, 2., 2.1])
    stamps, velocity = velocity_from_positions(times, np.zeros((4, 3)))
    assert np.isnan(velocity[1]).all()
    assert first_low_speed(stamps, np.linalg.norm(velocity, axis=1), 0., 2.1) is None


def test_known_motion_deceleration_time_and_distance():
    times, points = decelerating_path()
    report, _, _ = analyze_braking(times, points, .8, 1., 3., speed_threshold=.1)
    assert report['physical_braking_verified']
    assert report['initial_speed_at_loss_mps'] == pytest.approx(1.)
    assert report['speed_at_zero_command_mps'] == pytest.approx(.999, abs=.005)
    assert report['command_reaction_delay_s'] == pytest.approx(.2)
    assert report['braking_time_s'] == pytest.approx(.9, abs=.01)
    assert report['braking_distance_m'] == pytest.approx(.495, abs=.002)


def test_already_stationary_is_not_successful_emergency_braking():
    times = np.arange(-1., 3., .01)
    points = np.column_stack((times*.03, times*0., times*0.+3.))
    report, _, _ = analyze_braking(times, points, 0., .4, 1.5)
    assert report['observed_low_speed'] is not None
    assert report['unverified_reason'] == 'insufficient_initial_motion'
    assert not report['physical_braking_verified']
    assert report['braking_time_s'] is None
    assert report['braking_distance_m'] is None


def test_already_decelerating_before_loss_is_not_attributed_to_safety():
    times = np.arange(-.5, 3.001, .01)
    t = np.minimum(times, 2.)
    points = np.column_stack((2*t-.5*t*t, times*0., times*0.))
    report, _, _ = analyze_braking(times, points, 1., 1.2, 3.)
    assert report['observed_low_speed'] is not None
    assert report['unverified_reason'] == 'already_decelerating_before_loss'
    assert report['braking_time_s'] is None


def test_motion_that_already_stopped_before_zero_command_is_not_new_braking():
    times, points = decelerating_path()
    report, _, _ = analyze_braking(times, points, .8, 2.2, 3.5)
    assert report['unverified_reason'] == 'insufficient_motion_at_zero_command'
    assert report['braking_time_s'] is None


def test_recovery_before_stop_dwell_leaves_braking_unconfirmed():
    times, points = decelerating_path()
    report, _, _ = analyze_braking(times, points, .8, 1., 2.2, speed_threshold=.1)
    assert report['unverified_reason'] == 'stop_not_confirmed_during_safety'
    assert report['observed_low_speed'] is None
    assert report['braking_time_s'] is None


def test_missing_braking_path_prevents_distance_claim():
    times, points = decelerating_path()
    keep = (times < 1.1) | (times > 1.8)
    report, _, _ = analyze_braking(times[keep], points[keep], .8, 1., 3.)
    assert report['observed_low_speed'] is not None
    assert report['unverified_reason'] == 'missing_stop_path'
    assert report['braking_distance_m'] is None


def test_incomplete_initial_data_does_not_establish_motion_conditions():
    times, points = decelerating_path()
    keep = times >= .6
    report, _, _ = analyze_braking(times[keep], points[keep], .8, 1., 3.)
    assert report['unverified_reason'] == 'insufficient_initial_motion_data'


def test_below_threshold_flash_is_not_a_confirmed_stop():
    times = np.arange(0., 2., .05)
    speeds = np.ones(len(times))
    speeds[(times >= .5) & (times < .9)] = 0.
    assert first_low_speed(times, speeds, 0., 2.) is None


@pytest.mark.parametrize('parameters', [dict(dwell=0.), dict(motion_threshold=.1),
                                         dict(max_gap=np.nan), dict(speed_threshold=-1.)])
def test_invalid_parameters_are_rejected(parameters):
    times, points = decelerating_path()
    with pytest.raises(ValueError):
        analyze_braking(times, points, .8, 1., 3., **parameters)
