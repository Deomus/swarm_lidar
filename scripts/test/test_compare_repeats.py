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
