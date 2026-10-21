"""Повторы нельзя смешивать по параметрам или скрывать неудачные результаты."""

import importlib.util
from pathlib import Path

import pytest


@pytest.fixture
def compare(monkeypatch):
    scripts = Path(__file__).parents[1]
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location('compare_under_test', scripts/'compare_repeats.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run(passed=True, fingerprint='same', settled=10.):
    return dict(configuration_sha256=fingerprint, scenario='s1', passed=passed,
                formation=dict(rms=.2, max=.5), min_pair_distance_m=3.,
                localization=dict(rms=.04), confirmed_zero_delay_s=None,
                transitions=[dict(number=1, formation='line', scale=1., settling_duration_s=settled)])


def test_statistics_use_sample_standard_deviation(compare):
    assert compare.distribution([1., 2., 3.]) == dict(count=3, mean=2., sample_std=1., min=1., max=3.)
    assert compare.distribution([1.])['sample_std'] is None


def test_failed_and_unsettled_runs_remain_visible(compare):
    report = compare.summarize_runs([run(), run(passed=False, settled=None)])
    assert not report['all_passed']
    assert len(report['runs']) == 2
    transition = report['transition_summary'][0]
    assert transition['unsettled_runs'] == 1
    assert transition['settling_duration_s']['count'] == 1


def test_different_parameters_cannot_be_reported_as_same_series(compare):
    with pytest.raises(ValueError, match='параметры'):
        compare.summarize_runs([run(), run(fingerprint='other')])


def test_compact_transition_preserves_unavailable_and_largest_overshoot(compare):
    result = compare.compact_transition(dict(number=1, formation='line', scale=1.,
        followers={'1': dict(projection=dict(overshoot_m=.2)),
                   '2': dict(projection=dict(overshoot_m=.4)), '3': dict(projection=None)}))
    assert result['max_projected_overshoot_m'] == .4
    missing = compare.compact_transition(dict(number=1, unavailable='incomplete_target_history'))
    assert missing['unavailable'] == 'incomplete_target_history'
    assert missing['max_projected_overshoot_m'] is None


def test_equal_command_windows_exclude_settling_but_keep_inactive_errors(compare):
    rows = []
    for stamp, error, active in [(9., 100., 1), (10., .2, 1), (11., .4, 0), (12., 99., 1)]:
        rows.append(dict(timestamp=stamp, target_id=1, formation_error_m=error,
                         localization_error_m=.04, min_pair_distance_m=3., relative_stale=0,
                         leader_state_stale=0, gt_missing=0, formation_missing=0,
                         localization_missing=0, controller_active=active))
    result = compare.fixed_profile_summary([dict(kind='manual_input', start=10.)], rows, 2.)
    assert result['rows'] == 2
    assert result['formation_error_m']['rms'] == pytest.approx(.1**.5)
    assert result['controller_active_share'] == .5
