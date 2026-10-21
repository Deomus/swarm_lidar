"""Проверки посадки без запуска ROS-узлов и полётного контроллера."""
import importlib
from types import SimpleNamespace

import pytest


@pytest.fixture
def scenario_module():
    return importlib.import_module('swarm_lidar.scenario')


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


def test_success_received_after_simulation_deadline_does_not_pass(scenario_module):
    scenario = scenario_module.Scenario.__new__(scenario_module.Scenario)
    clock = [0.]
    scenario.now = lambda: clock[0]
    def wait(predicate, timeout, message):
        clock[0] = 3.
        predicate()
    scenario.wait = wait
    with pytest.raises(RuntimeError, match='лимит'):
        scenario.wait_sim(lambda: True, 2., 'Манёвр')


def test_ground_wait_restarts_after_estimator_height_excursion(scenario_module):
    scenario = scenario_module.Scenario.__new__(scenario_module.Scenario)
    scenario.states = {i: SimpleNamespace(armed=False,
        pose=SimpleNamespace(position=SimpleNamespace(z=0.)),
        velocity=SimpleNamespace(linear=SimpleNamespace(z=0.))) for i in range(4)}
    scenario.fresh = lambda: True
    scenario.event = lambda _: None
    clock = [0.]
    scenario.now = lambda: clock[0]
    def wait(predicate, timeout, reason):
        for t in range(9):
            clock[0] = float(t)
            scenario.states[0].pose.position.z = 2.1 if t == 2 else 0.
            if predicate():
                assert t == 6
                return
        pytest.fail('Оценка не установилась')
    scenario.wait_sim = wait
    scenario.wait_ground_settled()


def braking_scenario(module):
    scenario = module.Scenario.__new__(module.Scenario)
    clock = [10.]
    scenario.now = lambda: clock[0]
    scenario.event = lambda _: None
    scenario.states, scenario.commands = {}, {}
    def update(speed=.1):
        sec = int(clock[0])
        stamp = SimpleNamespace(sec=sec, nanosec=round((clock[0]-sec)*1e9))
        scenario.status = dict(mode='DATA_HOLD', time=clock[0])
        for index in range(4):
            vector = SimpleNamespace(x=speed, y=0., z=0.)
            scenario.states[index] = SimpleNamespace(timestamp=stamp, valid=True,
                armed=True, frame_id='map', velocity=SimpleNamespace(linear=vector))
            if index:
                scenario.commands[index] = SimpleNamespace(source_stamp=stamp, valid=False,
                    frame_id='map', safety_state='DATA_HOLD',
                    target_velocity=SimpleNamespace(linear=SimpleNamespace(x=0., y=0., z=0.)))
    update()
    return scenario, clock, update


def test_braking_start_requires_three_fresh_measured_speeds(scenario_module):
    scenario, clock, update = braking_scenario(scenario_module)
    assert not scenario.followers_moving()
    update(.7)
    assert scenario.followers_moving()
    scenario.states[3].velocity.linear.x = .49
    assert not scenario.followers_moving()
    update(.7)
    clock[0] += .6
    assert not scenario.followers_moving()


@pytest.mark.parametrize('defect', ['stale_command', 'future_command', 'stale_status',
                                   'valid_command', 'nonzero_command', 'wrong_frame'])
def test_braking_confirmation_rejects_bad_control_evidence(scenario_module, defect):
    scenario, clock, update = braking_scenario(scenario_module)
    command = scenario.commands[2]
    if defect == 'stale_command':
        command.source_stamp = SimpleNamespace(sec=9, nanosec=0)
    elif defect == 'future_command':
        command.source_stamp = SimpleNamespace(sec=11, nanosec=0)
    elif defect == 'stale_status':
        scenario.status['time'] = 9.
    elif defect == 'valid_command':
        command.valid = True
    elif defect == 'nonzero_command':
        command.target_velocity.linear.x = .01
    else:
        command.frame_id = 'body'
    assert not scenario.fresh_data_hold(9.5)
    assert not scenario.braking_stopped(9.5, [None], [None])


def test_braking_stop_requires_continuous_fresh_low_speed_after_zero(scenario_module):
    scenario, clock, update = braking_scenario(scenario_module)
    since, hold = [None], [None]
    assert not scenario.braking_stopped(9.5, since, hold)
    clock[0] = 10.3
    update(.2)
    assert not scenario.braking_stopped(9.5, since, hold)
    assert since == [None]
    clock[0] = 10.4
    update(.1)
    assert not scenario.braking_stopped(9.5, since, hold)
    clock[0] = 10.9
    update(.1)
    assert scenario.braking_stopped(9.5, since, hold)


def test_braking_cannot_finish_after_regular_control_recovers(scenario_module):
    scenario, clock, update = braking_scenario(scenario_module)
    since, hold = [None], [None]
    scenario.braking_stopped(9.5, since, hold)
    clock[0] = 10.3
    update()
    scenario.status['mode'] = 'FOLLOWING'
    with pytest.raises(RuntimeError, match='восстановилось до'):
        scenario.braking_stopped(9.5, since, hold)


def test_braking_rejects_old_low_speed_state_from_before_zero_command(scenario_module):
    scenario, clock, update = braking_scenario(scenario_module)
    for state in scenario.states.values():
        state.timestamp = SimpleNamespace(sec=9, nanosec=900000000)
    since, hold = [None], [None]
    assert not scenario.braking_stopped(9.5, since, hold)
    assert since == [None]


def test_drop_duration_is_restored_when_stationary_start_is_rejected(scenario_module, monkeypatch):
    scenario = scenario_module.Scenario.__new__(scenario_module.Scenario)
    value = SimpleNamespace(type=scenario_module.Parameter.Type.DOUBLE.value, double_value=.7)
    future = SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(values=[value]))
    parameter_client = SimpleNamespace(wait_for_services=lambda **_: True,
                                       get_parameters=lambda _: future)
    drop_client = SimpleNamespace(wait_for_service=lambda **_: True)
    scenario.node = SimpleNamespace(create_client=lambda *args: drop_client)
    scenario.clients = []
    scenario.call = lambda *args: None
    scenario.settle = lambda *args: None
    scenario.wait = lambda predicate, *args: predicate()
    scenario.manual = lambda *args: None
    scenario.followers_moving = lambda *args: False
    scenario.event = lambda *args: None
    changes = []
    scenario.set_drop_duration = lambda client, duration: changes.append(duration)
    monkeypatch.setattr(scenario_module, 'AsyncParameterClient', lambda *args: parameter_client)
    with pytest.raises(RuntimeError, match='нет трёх свежих скоростей'):
        scenario.check_braking()
    assert changes == [3., .7]


def test_operator_finish_service_sets_explicit_completion_flag(scenario_module):
    scenario = scenario_module.Scenario.__new__(scenario_module.Scenario)
    scenario.operator_finished = False
    response = scenario.finish_operator(None, SimpleNamespace())
    assert response.success
    assert scenario.operator_finished


def test_operator_waits_for_explicit_finish_and_uses_wall_limit(scenario_module, monkeypatch):
    scenario, clock, update = braking_scenario(scenario_module)
    services = []
    scenario.node = SimpleNamespace(create_service=lambda kind, path, callback: services.append(path))
    wall = [0.]
    monkeypatch.setattr(scenario_module, 'time', SimpleNamespace(monotonic=lambda: wall[0]))
    def wait(predicate, timeout, reason):
        assert timeout == 1800.
        for tick in range(4):
            wall[0] = float(tick)
            clock[0] = 10.+tick
            update()
            if tick == 3:
                scenario.finish_operator(None, SimpleNamespace())
            if predicate():
                assert tick == 3
                return
        pytest.fail('Не принято явное завершение')
    scenario.wait = wait
    scenario.operator_session()
    assert services == ['/scenario/finish']


@pytest.mark.parametrize('fresh', [False, True])
def test_operator_rejects_missing_or_frozen_telemetry(scenario_module, monkeypatch, fresh):
    scenario, clock, update = braking_scenario(scenario_module)
    scenario.node = SimpleNamespace(create_service=lambda *args: None)
    scenario.fresh = lambda: fresh
    wall = [0.]
    monkeypatch.setattr(scenario_module, 'time', SimpleNamespace(monotonic=lambda: wall[0]))
    def wait(predicate, timeout, reason):
        assert not predicate()
        wall[0] = 5.
        predicate()
    scenario.wait = wait
    with pytest.raises(RuntimeError, match='Нет новых свежих состояний'):
        scenario.operator_session()


def test_operator_rejects_clock_rewind(scenario_module, monkeypatch):
    scenario, clock, update = braking_scenario(scenario_module)
    scenario.node = SimpleNamespace(create_service=lambda *args: None)
    monkeypatch.setattr(scenario_module, 'time', SimpleNamespace(monotonic=lambda: 0.))
    def wait(predicate, *args):
        clock[0] = 9.
        predicate()
    scenario.wait = wait
    with pytest.raises(RuntimeError, match='назад'):
        scenario.operator_session()
