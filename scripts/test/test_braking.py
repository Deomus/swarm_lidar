"""Порядок команд по времени записи; source_stamp может относиться к LiDAR."""

import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location('analyze_braking',
                                            Path(__file__).parents[1] / 'analyze_braking.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def command(time, source=None, valid=False, safety='DATA_HOLD'):
    return dict(time=time, source_time=time if source is None else source,
                velocity=[0., 0., 0.], frame='map', valid=valid, safety=safety)


def test_recovery_with_old_measurement_stamp_ends_zero_command_window():
    commands = [command(1.), command(1.1), command(1.2),
                command(1.3, source=.9, valid=True, safety='FOLLOWING')]
    statuses = [dict(time=1., mode='DATA_HOLD')]
    assert module.safety_window(commands, statuses, .8, 2.) == (1., 1.2)


def test_stale_and_future_source_stamp_do_not_establish_fresh_zero():
    commands = [command(1., source=.1), command(1.1, source=1.5), command(1.2)]
    statuses = [dict(time=1., mode='DATA_HOLD')]
    assert module.safety_window(commands, statuses, .8, 2.) == (1.2, 1.2)


def test_record_clock_skew_requires_explicit_offline_tolerance():
    commands = [command(1.), command(1.1, source=1.104), command(1.2)]
    statuses = [dict(time=1., mode='DATA_HOLD')]
    assert module.safety_window(commands, statuses, .8, 2.) == (1., 1.)
    assert module.safety_window(commands, statuses, .8, 2., clock_tolerance=.005) == (1., 1.2)
    commands[1]['source_time'] = 1.11
    assert module.safety_window(commands, statuses, .8, 2., clock_tolerance=.005) == (1., 1.)


@pytest.mark.parametrize('tolerance', [-.001, .051, float('nan')])
def test_invalid_clock_tolerance_is_rejected(tolerance):
    with pytest.raises(ValueError, match='Допуск'):
        module.safety_window([], [], 0., 1., clock_tolerance=tolerance)


def test_gap_ends_confirmation_at_last_contiguous_zero():
    commands = [command(1.), command(1.1), command(1.5)]
    statuses = [dict(time=1., mode='DATA_HOLD'), dict(time=1.5, mode='DATA_HOLD')]
    assert module.safety_window(commands, statuses, .8, 2.) == (1., 1.1)


def test_missing_data_hold_status_does_not_confirm_control_mode():
    commands = [command(1.), command(1.1)]
    assert module.safety_window(commands, [], .8, 2.) is None


def test_disabled_zero_command_is_not_a_data_loss_braking_event():
    commands = [command(1., safety='DISABLED'), command(1.1, safety='HOLD')]
    statuses = [dict(time=1., mode='DATA_HOLD')]
    assert module.safety_window(commands, statuses, .8, 2.) is None


def test_no_success_claim_when_fresh_zero_commands_are_missing():
    report, rows = module.analyze_loss({i: [] for i in (1, 2, 3)},
                                       {i: [] for i in (1, 2, 3)}, [], 1., 3.)
    assert not report['physical_braking_verified']
    assert rows == []
    assert all(value['unverified_reason'] == 'fresh_zero_command_not_found'
               for value in report['followers'].values())
