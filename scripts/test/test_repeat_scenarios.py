"""Мир завершается только после подтверждённой посадки, включая отказ сценария."""

import importlib.util
import json
from pathlib import Path
import signal

import pytest


@pytest.mark.parametrize('confirmed', [False, True])
def test_failed_flight_keeps_world_unless_landing_confirmed(tmp_path, monkeypatch, confirmed):
    spec = importlib.util.spec_from_file_location('repeat_under_test',
                                                Path(__file__).parents[1]/'repeat_scenarios.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'snapshot', lambda: {'runtime_revision': 'test'})
    signals = []

    class Simulator:
        pid = 123
        returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, signum):
            signals.append(signum)

        def wait(self, timeout):
            assert confirmed
            self.returncode = 0

    class Scenario:
        pid = 456
        returncode = 1

        def wait(self):
            directory = tmp_path/'s2_1/flight'
            directory.mkdir()
            module.dump(directory/'result.json', dict(passed=False, passed_landing=confirmed))

        def poll(self):
            return self.returncode

    simulator = Simulator()
    processes = iter((simulator, Scenario()))
    monkeypatch.setattr(module.subprocess, 'Popen', lambda *args, **kwargs: next(processes))
    with pytest.raises(RuntimeError, match='Опыт не пройден' if confirmed else 'Посадка не подтверждена'):
        module.run_one(tmp_path, 's2', 1, Path('/px4'))
    assert signals == ([signal.SIGTERM] if confirmed else [])


def test_missing_or_broken_result_does_not_confirm_landing(tmp_path):
    spec = importlib.util.spec_from_file_location('repeat_under_test',
                                                Path(__file__).parents[1]/'repeat_scenarios.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    path = tmp_path/'result.json'
    assert not module.landing_confirmed(path)
    path.write_text('{')
    assert not module.landing_confirmed(path)
    path.write_text(json.dumps(dict(passed=True, passed_landing=False)))
    assert not module.landing_confirmed(path)


def test_failed_series_does_not_leave_running_status(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location('repeat_under_test',
                                                Path(__file__).parents[1]/'repeat_scenarios.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    root = tmp_path/'series'
    monkeypatch.setattr(module.sys, 'argv', ['repeat', '--output', str(root), '--runs', '1'])

    def fail(output, kind, index, px4):
        module.dump(output/'current.json', dict(series_state='running', simulator_pid=123))
        raise RuntimeError('Сценарий отказал')

    monkeypatch.setattr(module, 'run_one', fail)
    with pytest.raises(RuntimeError, match='отказал'):
        module.main()
    assert json.loads((root/'current.json').read_text())['series_state'] == 'failed'
