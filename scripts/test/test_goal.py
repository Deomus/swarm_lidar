import importlib.util
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location('goal_analysis', Path(__file__).parents[1]/'analyze_goal.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_final_error_uses_nearest_truth_and_latest_goal_before_event():
    report = module.final_goal([(10.1, [3., 4., 0.]), (9., [100., 0., 0.])],
                               [(1., [8., 0., 0.]), (8., [0., 0., 0.]),
                                (11., [3., 4., 0.])], 10.)
    assert report['final_goal_error_m'] == 5.
    assert report['gt_delta_s'] == pytest.approx(.1)
    assert report['goal_time'] == 8.


@pytest.mark.parametrize('samples,goals', [
    ([(9., [0., 0., 0.])], [(1., [0., 0., 0.])]),
    ([(10., [0., 0., 0.])], [(11., [0., 0., 0.])]),
    ([(10., [float('nan'), 0., 0.])], [(1., [0., 0., 0.])]),
    ([(10., [0., 0., 0.])], [(1., [0., 0.])]),
])
def test_missing_stale_or_invalid_samples_do_not_produce_final_error(samples, goals):
    with pytest.raises(ValueError):
        module.final_goal(samples, goals, 10.)


@pytest.mark.parametrize('stamp,tolerance', [(float('nan'), .15), (10., 0.), (10., float('inf'))])
def test_invalid_matching_parameters_are_rejected(stamp, tolerance):
    with pytest.raises(ValueError):
        module.final_goal([], [], stamp, tolerance)
