"""Проверки посадки без запуска ROS-узлов и полётного контроллера."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def scenario_module():
    spec = importlib.util.spec_from_file_location('scenario_under_test', Path(__file__).parents[1] / 'run_scenario.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_scenario(module, monkeypatch, fresh=True):
    scenario = module.Scenario.__new__(module.Scenario)
    scenario.node = object()
    scenario.states = {i: SimpleNamespace(armed=True) for i in range(4)}
    scenario.fresh = lambda: fresh
    scenario.event = lambda message: None
    clock = [0.]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=lambda: clock[0]))
    def spin(*args, **kwargs):
        clock[0] += .5
    monkeypatch.setattr(module.rclpy, 'spin_once', spin)
    return scenario


def test_landing_retries_temporary_service_failure(scenario_module, monkeypatch):
    scenario = make_scenario(scenario_module, monkeypatch)
    attempts = {i: 0 for i in range(4)}
    def call(path):
        if not path.endswith('/land'):
            return
        index = int(path.split('/')[1][-1])
        attempts[index] += 1
        if index == 2 and attempts[index] == 1:
            raise RuntimeError('Временный отказ сервиса')
        scenario.states[index].armed = False
    scenario.call = call
    scenario.land()
    assert attempts == {0: 1, 1: 1, 2: 2, 3: 1}


def test_cached_disarmed_state_is_not_landing_confirmation(scenario_module, monkeypatch):
    scenario = make_scenario(scenario_module, monkeypatch, fresh=False)
    for state in scenario.states.values():
        state.armed = False
    requests = []
    scenario.call = requests.append
    with pytest.raises(RuntimeError, match='свежей телеметрией'):
        scenario.land()
    assert all(f'/uav_{i}/land' in requests for i in range(4))


def test_maneuver_timeout_uses_simulation_clock(scenario_module):
    scenario = scenario_module.Scenario.__new__(scenario_module.Scenario)
    clock = [0.]
    scenario.now = lambda: clock[0]
    def wait(predicate, timeout, reason):
        # Симуляция вчетверо медленнее реального времени.
        for _ in range(int(timeout)):
            if predicate():
                return
            clock[0] += .25
        raise RuntimeError(reason)
    scenario.wait = wait
    scenario.wait_sim(lambda: clock[0] >= 1.5, 2., 'Манёвр')
    assert clock[0] == 1.5


@pytest.mark.parametrize('new_time,reason', [(2.1, 'лимит'), (-1., 'назад')])
def test_maneuver_rejects_simulation_timeout_and_rewind(scenario_module, new_time, reason):
    scenario = scenario_module.Scenario.__new__(scenario_module.Scenario)
    clock = [0.]
    scenario.now = lambda: clock[0]
    def wait(predicate, timeout, message):
        clock[0] = new_time
        predicate()
    scenario.wait = wait
    with pytest.raises(RuntimeError, match=reason):
        scenario.wait_sim(lambda: False, 2., 'Манёвр')
