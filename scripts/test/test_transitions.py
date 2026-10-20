"""Выделение смен слотов из записанных состояний, включая паузы и обрыв."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

spec = importlib.util.spec_from_file_location('analyze_transitions',
                                            Path(__file__).parents[1] / 'analyze_transitions.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def status(time, mode, formation='column'):
    return dict(time=time, mode=mode, formation=formation, scale=1.)


def test_data_hold_during_motion_is_one_interrupted_transition():
    windows = module.transition_windows([
        status(0., 'FOLLOWING'), status(1., 'FORMATION_CHANGE'), status(2., 'DATA_HOLD'),
        status(3., 'FORMATION_CHANGE'), status(5., 'FOLLOWING'), status(8., 'DISABLED')])
    assert len(windows) == 1
    assert windows[0]['start'] == 1.
    assert windows[0]['plan_end'] == 5.
    assert windows[0]['end'] == 8.
    assert windows[0]['interruptions'] == [dict(time=2., mode='DATA_HOLD')]


def test_next_transition_ends_observation_and_shutdown_does_not_extend_old_window():
    windows = module.transition_windows([
        status(1., 'FORMATION_CHANGE'), status(3., 'FOLLOWING'), status(5., 'DISABLED'),
        status(10., 'FORMATION_CHANGE'), status(12., 'FOLLOWING'), status(15., 'FOLLOWING')])
    assert [w['end'] for w in windows] == [5., 15.]


def test_incomplete_transition_is_preserved_without_fake_final_target():
    times = np.arange(0., 2.01, .1)
    samples = [(t, np.full((3, 3), t)) for t in times]
    report, rows = module.analyze_bag(samples, samples,
                                     [status(1., 'FORMATION_CHANGE'), status(2., 'DISABLED')])
    assert report['transitions'][0]['unavailable'] == 'incomplete_target_history'
    assert rows == []


def test_actual_slot_order_is_preserved_instead_of_canonical_shape_assignment():
    times = np.arange(0., 6.01, .1)
    final = np.array([[-12., 0., 0.], [-4., 0., 0.], [-8., 0., 0.]])
    targets = [(t, min(max(t-1., 0.)/2., 1.)*final) for t in times]
    statuses = [status(0., 'FOLLOWING'), status(1., 'FORMATION_CHANGE'),
                status(3., 'FOLLOWING'), status(6., 'DISABLED')]
    report, rows = module.analyze_bag(targets, targets, statuses)
    transition = report['transitions'][0]
    assert transition['final_slots_heading_m'] == final.tolist()
    assert transition['followers']['1']['travel_m'] == 12.
    assert transition['followers']['2']['travel_m'] == 4.
    assert transition['settling_duration_s'] == pytest.approx(2.)
    assert len(rows) > 100


def test_rounding_difference_does_not_use_first_moving_slot_as_initial_target():
    times = np.arange(0., 3.01, .05)
    targets = [(t, np.full((3, 3), min(max(t-.25, 0.), 1.))) for t in times]
    statuses = [status(.2, 'FOLLOWING'), status(.3000000000000001, 'FORMATION_CHANGE'),
                status(1.3, 'FOLLOWING'), status(3., 'DISABLED')]
    report, _ = module.analyze_bag(targets, targets, statuses)
    transition = report['transitions'][0]
    assert transition['initial_target_sample_time'] == pytest.approx(.25)
    assert transition['initial_slots_heading_m'] == np.zeros((3, 3)).tolist()
