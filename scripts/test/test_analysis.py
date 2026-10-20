import importlib.util
from pathlib import Path

import pytest


def test_manual_windows_do_not_mix_takeoff_and_loss_samples():
    spec = importlib.util.spec_from_file_location('analysis_under_test', Path(__file__).parents[1] / 'analyze_scenario.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    events = [dict(time=10., event='Проверка ручного входа: 0.5 м/с'),
              dict(time=20., event='Проверка ручного входа: 1.0 м/с'),
              dict(time=30., event='Потеря LiDAR-поз в mesh: 0.7 с')]
    rows = []
    for stamp, error in [(9., 100.), (10., .1), (19., .2), (20., 1.), (29., 2.), (30., 99.)]:
        rows.append(dict(timestamp=stamp, target_id=1, localization_error_m=.15,
                         formation_error_m=error, min_pair_distance_m=3., relative_stale=0,
                         leader_state_stale=0, gt_missing=0, formation_missing=0, localization_missing=0))
    result = module.analyze(events, rows)['windows']
    assert len(result) == 2
    assert result[0]['requested_speed_mps'] == .5
    assert result[0]['metrics']['formation_error_m']['mean'] == pytest.approx(.15)
    assert result[1]['metrics']['formation_error_m']['mean'] == 1.5
    assert all(window['duration_s'] == 10. for window in result)
