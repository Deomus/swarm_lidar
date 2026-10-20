"""Знак перелёта, полоса установления и неполные измерения."""

import numpy as np
import pytest

from evaluation.transition_metrics import analyze_transition, interpolate_offsets, settling_time


def trajectory(sign=1.):
    times = np.arange(0., 8.01, .1)
    final = np.array([[sign, 0., 0.]] * 3)
    desired = np.minimum(times / 2., 1.)[:, None, None] * final
    positions = np.minimum(np.maximum(times-.5, 0.)/2., 1.)
    positions += .2*np.maximum(1.-abs(times-3.)/.5, 0.)
    actual = positions[:, None, None]*final
    return times, actual, desired, np.zeros((3, 3)), final


@pytest.mark.parametrize('sign', [1., -1.])
def test_signed_projection_counts_crossing_in_both_directions(sign):
    times, actual, desired, initial, final = trajectory(sign)
    report = analyze_transition(times, actual, desired, initial, final, 0., 2., 8.)
    follower = report['followers']['1']
    assert follower['projection']['signed_peak_m'] == pytest.approx(.2)
    assert follower['projection']['overshoot_m'] == pytest.approx(.2)
    assert follower['projection']['time'] == pytest.approx(3.)
    assert follower['axes']['x']['overshoot_m'] == pytest.approx(.2)
    assert follower['axes']['y'] is None
    assert follower['axes']['z'] is None
    assert follower['reaction_delay_s'] == pytest.approx(.7)


def test_no_crossing_keeps_negative_peak_and_zero_overshoot():
    times, actual, desired, initial, final = trajectory()
    actual = np.minimum(actual, .8)
    report = analyze_transition(times, actual, desired, initial, final, 0., 2., 8.)
    peak = report['followers']['1']['projection']
    assert peak['signed_peak_m'] == pytest.approx(-.2)
    assert peak['overshoot_m'] == 0.


def test_perpendicular_error_is_not_signed_overshoot():
    times, actual, desired, initial, final = trajectory()
    actual[:, :, 0] = np.minimum(actual[:, :, 0], 1.)
    actual[:, :, 1] = 5.
    report = analyze_transition(times, actual, desired, initial, final, 0., 2., 8.)
    follower = report['followers']['1']
    assert follower['projection']['overshoot_m'] == 0.
    assert follower['dynamic_slot_error_m']['max'] > 5.
    assert follower['axes']['y'] is None
    assert report['settled_time'] is None


def test_diagonal_projection_is_normalized_not_sum_of_axis_errors():
    times, actual, desired, initial, final = trajectory()
    actual[:, :, 1] = actual[:, :, 0]
    desired[:, :, 1] = desired[:, :, 0]
    final[:, 1] = 1.
    report = analyze_transition(times, actual, desired, initial, final, 0., 2., 8.)
    assert report['followers']['1']['projection']['overshoot_m'] == pytest.approx(.2*np.sqrt(2))


def test_unmoved_follower_has_no_direction_or_reaction_measurement():
    times, actual, desired, initial, final = trajectory()
    actual[:, 1] = desired[:, 1] = final[1] = 0.
    report = analyze_transition(times, actual, desired, initial, final, 0., 2., 8.)
    follower = report['followers']['2']
    assert follower['projection'] is None
    assert follower['axes'] == dict(x=None, y=None, z=None)
    assert follower['reaction_delay_s'] is None
    assert follower['settled_time'] == pytest.approx(2.)


def test_settling_uses_final_target_after_target_stops_and_last_exit():
    times = np.arange(0., 8.01, .1)
    errors = np.zeros(len(times))
    errors[(times >= 4.) & (times <= 4.5)] = .31
    assert settling_time(times, errors, 2., 8.) == pytest.approx(4.6)
    assert settling_time(times, errors, 2., 5.) is None


def test_gap_and_unknown_tail_do_not_confirm_settling():
    times = np.array([2., 2.1, 2.2, 6., 6.1, 6.2])
    assert settling_time(times, np.zeros(len(times)), 2., 6.2) is None
    times = np.arange(2., 6.01, .1)
    assert settling_time(times, np.zeros(len(times)), 2., 8.) is None
    errors = np.zeros(len(times))
    errors[-1] = np.nan
    assert settling_time(times, errors, 2., 6.) is None


def test_final_metrics_and_dynamic_metrics_are_separate():
    times, _, desired, initial, final = trajectory()
    report = analyze_transition(times, desired, desired, initial, final, 0., 2., 8.)
    follower = report['followers']['1']
    assert follower['dynamic_slot_error_m']['max'] == 0.
    assert follower['final_slot_error_m']['max'] == 1.
    assert report['settling_duration_s'] == pytest.approx(2.)


def test_missing_early_gt_does_not_become_measured_reaction_delay():
    times, actual, desired, initial, final = trajectory()
    keep = times >= 1.
    report = analyze_transition(times[keep], actual[keep], desired[keep], initial, final, 0., 2., 8.)
    assert report['followers']['1']['reaction_delay_s'] is None


def test_interpolation_never_fills_gaps_or_extrapolates():
    times = np.array([0., .1, 2., 2.1])
    offsets = np.broadcast_to(times[:, None, None], (4, 3, 3)).copy()
    result = interpolate_offsets(times, offsets, np.array([-.1, 0., .05, 1., 2., 2.2]))
    assert np.isnan(result[[0, 3, 5]]).all()
    assert result[2, 0, 0] == pytest.approx(.05)
    assert result[4, 0, 0] == 2.


@pytest.mark.parametrize('kwargs', [dict(threshold=0.), dict(dwell=-1.), dict(max_gap=np.nan)])
def test_rejects_invalid_parameters(kwargs):
    times, actual, desired, initial, final = trajectory()
    with pytest.raises(ValueError):
        analyze_transition(times, actual, desired, initial, final, 0., 2., 8., **kwargs)


def test_rejects_unsorted_time():
    times, actual, desired, initial, final = trajectory()
    with pytest.raises(ValueError):
        analyze_transition(times[::-1], actual, desired, initial, final, 0., 2., 8.)
