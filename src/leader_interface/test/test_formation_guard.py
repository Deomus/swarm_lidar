"""Групповая готовность не использует эталонные или абсолютные позы ведомых."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from leader_interface.formation_guard import FormationGuard  # noqa: E402


def status(stamp, mode='FOLLOWING'):
    return {'time': stamp, 'mode': mode}


def test_standalone_does_not_require_group_or_trigger_hold():
    guard = FormationGuard()
    guard.require_ready(10.)
    assert guard.ready(10.)
    assert not guard.should_hold(True, 100.)


@pytest.mark.parametrize('mode', ['FOLLOWING', 'FORMATION_CHANGE'])
def test_selection_requires_fresh_ready_status(mode):
    guard = FormationGuard(required=True)
    with pytest.raises(ValueError, match='FORMATION_UNAVAILABLE'):
        guard.require_ready(10.)
    assert guard.update(status(10., mode), 10.)
    guard.require_ready(10.5)
    with pytest.raises(ValueError, match='FORMATION_UNAVAILABLE'):
        guard.require_ready(10.501)


def test_persistent_data_hold_stops_after_grace():
    guard = FormationGuard(required=True)
    assert guard.update(status(10.), 10.)
    assert not guard.should_hold(True, 10.)
    assert guard.update(status(10.1, 'DATA_HOLD'), 10.1)
    assert not guard.should_hold(True, 10.1)
    assert guard.update(status(10.9, 'DATA_HOLD'), 10.9)
    assert not guard.should_hold(True, 11.)
    assert guard.should_hold(True, 11.101)


def test_short_point_seven_loss_recovers_without_stopping_mission():
    guard = FormationGuard(required=True)
    assert guard.update(status(10.), 10.)
    assert not guard.should_hold(True, 10.49)
    assert not guard.should_hold(True, 10.51)
    assert not guard.should_hold(True, 10.69)
    assert guard.update(status(10.7), 10.7)
    assert not guard.should_hold(True, 10.7)
    assert guard.unavailable_since is None


def test_short_explicit_data_hold_recovers_and_next_loss_has_new_grace():
    guard = FormationGuard(required=True)
    guard.update(status(10., 'DATA_HOLD'), 10.)
    assert not guard.should_hold(True, 10.)
    guard.update(status(10.7), 10.7)
    assert not guard.should_hold(True, 10.7)
    guard.update(status(11., 'DATA_HOLD'), 11.)
    assert not guard.should_hold(True, 11.)
    assert not guard.should_hold(True, 11.9)
    assert guard.should_hold(True, 12.01)


def test_fresh_ready_status_resets_grace_even_between_timer_ticks():
    guard = FormationGuard(required=True)
    guard.update(status(10., 'DATA_HOLD'), 10.)
    guard.should_hold(True, 10.)
    guard.update(status(10.8), 10.8)
    guard.update(status(10.9, 'DATA_HOLD'), 10.9)
    assert not guard.should_hold(True, 10.9)
    assert not guard.should_hold(True, 11.1)


def test_missing_status_stays_unready_after_ttl_and_cannot_select():
    guard = FormationGuard(required=True)
    guard.update(status(10.), 10.)
    assert not guard.should_hold(True, 10.51)
    assert guard.should_hold(True, 11.52)
    with pytest.raises(ValueError, match='FORMATION_UNAVAILABLE'):
        guard.require_ready(11.52)


@pytest.mark.parametrize('value', [status(1000.), status(9.), status(10., ''),
                                  status(float('nan')), status(True),
                                  {'mode': 'FOLLOWING'}, {'time': '10', 'mode': 'FOLLOWING'}, []])
def test_stale_future_or_malformed_status_does_not_poison_watermark(value):
    guard = FormationGuard(required=True)
    assert not guard.update(value, 10.)
    assert guard.status_stamp == -float('inf')
    assert guard.update(status(10.), 10.)


def test_out_of_order_and_duplicate_cannot_restore_ready_mode():
    guard = FormationGuard(required=True)
    guard.update(status(10., 'DATA_HOLD'), 10.)
    assert not guard.update(status(9.9), 10.1)
    assert not guard.update(status(10.), 10.1)
    assert not guard.ready(10.1)
    assert guard.update(status(10.1), 10.1)
    assert guard.ready(10.1)


def test_time_rewind_clears_ready_status_and_grace_then_accepts_new_epoch():
    guard = FormationGuard(required=True)
    guard.update(status(10.), 10.)
    assert guard.ready(10.)
    assert not guard.ready(1.)
    assert guard.status_stamp == -float('inf')
    assert guard.unavailable_since is None
    assert guard.update(status(1.), 1.)
    guard.require_ready(1.)


def test_inactive_leader_does_not_accumulate_failure_time():
    guard = FormationGuard(required=True)
    assert not guard.should_hold(False, 10.)
    assert not guard.should_hold(False, 100.)
    assert not guard.should_hold(True, 100.)
    assert guard.should_hold(True, 101.01)
    assert not guard.should_hold(False, 101.02)
    assert guard.unavailable_since is None


@pytest.mark.parametrize('parameters', [{'status_timeout': 0}, {'status_timeout': float('nan')},
                                       {'grace': -1}, {'grace': float('inf')}, {'required': 1}])
def test_invalid_configuration(parameters):
    with pytest.raises(ValueError):
        FormationGuard(**parameters)
