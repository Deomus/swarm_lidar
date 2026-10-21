import http.client
import json
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from std_msgs.msg import String
from swarm_interfaces.msg import SwarmState

from swarm_lidar.web_panel import make_server
from swarm_lidar.web_ros import PanelBackend, formation_request, goal_request
from swarm_lidar.web_session import FlightSession


@pytest.fixture
def http_panel():
    commands = []
    backend = SimpleNamespace(snapshot=lambda: dict(connected=False),
                              submit=lambda action, body: commands.append((action, body)))
    assets = Path(__file__).parents[1]/'web'
    server = make_server(backend, assets, 0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2.)
    host = f'localhost:{server.server_port}'
    client.request('GET', '/', headers={'Host': host})
    page = client.getresponse().read().decode()
    token = re.search(r'name="panel-token" content="([^"]+)"', page)[1]
    yield client, host, token, commands
    client.close()
    server.shutdown()
    server.server_close()
    thread.join()


def request(panel, path='/api/command', body=None, **headers):
    client, host, token, _ = panel
    client.request('POST', path, body=json.dumps(body or {'action': 'hold'}),
                   headers={'Host': host, 'Content-Type': 'application/json',
                            'X-Panel-Token': token, **headers})
    response = client.getresponse()
    return response.status, json.loads(response.read())


def test_local_command_and_static_page(http_panel):
    assert request(http_panel)[0] == 202
    assert http_panel[3][0][0] == 'hold'
    client, host, _, _ = http_panel
    client.request('GET', '/api/state', headers={'Host': host})
    response = client.getresponse()
    assert json.loads(response.read()) == {'connected': False}


@pytest.mark.parametrize('headers', [
    {'Origin': 'https://external.example'}, {'Host': 'external.example'},
    {'X-Panel-Token': ''},
])
def test_cross_site_or_unauthorized_command_is_not_executed(http_panel, headers):
    assert request(http_panel, **headers)[0] == 403
    assert not http_panel[3]


def test_bad_request_and_unlisted_asset(http_panel):
    assert request(http_panel, body=['hold'])[0] == 400
    client, host, _, _ = http_panel
    client.request('GET', '/../package.xml', headers={'Host': host})
    response = client.getresponse()
    assert response.status == 404
    response.read()


@pytest.mark.parametrize('scale,duration', [(float('nan'), 25.), (1., float('inf')),
                                          (.5, 25.), (1., 1.), (True, 25.)])
def test_invalid_formation_never_reaches_ros(scale, duration):
    with pytest.raises(ValueError):
        formation_request(dict(formation='wedge', scale=scale, duration=duration))


@pytest.mark.parametrize('body', [
    {'x': float('nan'), 'y': 0., 'z': 5.},
    {'x': 0., 'y': 101., 'z': 5.},
    {'x': 0., 'y': 0., 'z': 0.},
    {'x': True, 'y': 0., 'z': 5.},
])
def test_invalid_goal_is_rejected(body):
    with pytest.raises(ValueError):
        goal_request(body)


def test_goal_request_accepts_map_coordinates():
    assert goal_request({'x': 12, 'y': -4.5, 'z': 5}) == (12., -4.5, 5.)


@pytest.fixture
def backend(tmp_path):
    rclpy.init()
    node = PanelBackend(tmp_path)
    node.now = lambda: 10.
    node.clock_progress = time.monotonic()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def feed(node, armed=False):
    for i in range(4):
        state = SwarmState(uav_id=i, valid=True, armed=armed, frame_id='map')
        state.timestamp.sec = int(node.now())
        state.timestamp.nanosec = int((node.now()-state.timestamp.sec)*1e9)
        node.on_state(state)


def test_no_takeoff_without_four_fresh_ground_states(backend):
    with pytest.raises(ValueError, match='свежие'):
        backend.submit('prepare', {})
    feed(backend, armed=True)
    with pytest.raises(ValueError, match='на земле'):
        backend.submit('prepare', {})


def test_future_state_cannot_block_later_valid_telemetry(backend):
    state = SwarmState(uav_id=0, valid=True, frame_id='map')
    state.timestamp.sec = 50
    backend.on_state(state)
    assert not backend.states
    feed(backend)
    assert backend.snapshot()['connected']
    backend.clock_progress -= 2.
    assert not backend.snapshot()['connected']


def test_commands_require_prepared_operator_and_established_formation(backend):
    feed(backend)
    with pytest.raises(ValueError, match='взлететь'):
        backend.submit('formation', dict(formation='line', scale=1., duration=25.))
    backend.flight.phase, backend.flight.kind = 'ready', 'operator'
    backend.on_formation(String(data=json.dumps(dict(time=10., mode='FORMATION_CHANGE', formation='line', scale=1.))))
    with pytest.raises(ValueError, match='установления'):
        backend.submit('formation', dict(formation='column', scale=1., duration=25.))
    backend.on_formation(String(data=json.dumps(dict(time=10., mode='FOLLOWING', formation='line', scale=1.))))
    backend.settled_since = 8.
    backend.submit('formation', dict(formation='column', scale=1., duration=25.))
    assert backend.commands.get_nowait() == ('formation', ('column', 1., 25.))


def test_second_landing_request_does_not_interrupt_landing(backend):
    backend.flight.phase = 'landing'
    backend.flight.kind = 'operator'
    backend.submit('land', {})
    assert backend.commands.empty()
    assert not backend.flight.cancelled


def test_repeated_landing_preserves_queued_finish(backend):
    feed(backend, armed=True)
    backend.flight.phase, backend.flight.kind = 'ready', 'operator'
    backend.submit('land', {})
    backend.submit('land', {})
    assert backend.commands.get_nowait() == ('finish', None)


def test_landing_has_priority_over_pending_formation(backend):
    feed(backend, armed=True)
    backend.flight.phase, backend.flight.kind = 'ready', 'operator'
    backend.commands.put(('formation', ('column', 1., 25.)))
    backend.submit('land', {})
    assert backend.commands.get_nowait() == ('finish', None)
    assert backend.commands.empty()
    assert backend.flight.phase == 'landing'


def test_established_formation_requires_sustained_small_errors(backend):
    clock = [10.]
    backend.now = lambda: clock[0]
    def update(error):
        feed(backend)
        backend.on_formation(String(data=json.dumps(dict(time=clock[0], mode='FOLLOWING',
            formation='line', scale=1., errors=[.1, error, .1]))))
        backend.tick()
    update(.7)
    assert not backend.snapshot()['formation_settled']
    clock[0] = 10.2
    update(.1)
    assert not backend.snapshot()['formation_settled']
    clock[0] = 11.3
    update(.1)
    assert backend.snapshot()['formation_settled']
    clock[0] = 11.4
    update(.65)
    assert not backend.snapshot()['formation_settled']


def test_external_landing_needs_fresh_disarmed_confirmation(backend):
    feed(backend, armed=True)
    backend.submit('land', {})
    backend.tick()
    backend.clock_progress -= 2.
    feed(backend, armed=False)
    backend.tick()
    # tick обновляет часы только при продвижении /clock; здесь они стоят.
    assert backend.action['state'] == 'pending'
    backend.clock_progress = time.monotonic()
    backend.tick()
    assert backend.action['state'] == 'ok'


def test_session_rejects_parallel_process_and_arbitrary_scenario(tmp_path):
    session = FlightSession(tmp_path)
    with pytest.raises(ValueError):
        session.start('shell')
    session.phase = 'ready'
    with pytest.raises(ValueError, match='текущий'):
        session.start('s1')


def test_finished_process_without_landing_ack_is_not_successful_shutdown(tmp_path):
    session = FlightSession(tmp_path)
    session.phase = 'landing'
    session.process = SimpleNamespace(poll=lambda: 1)
    session.thread = SimpleNamespace(join=lambda **kwargs: None, is_alive=lambda: False)
    session.landing_confirmed = False
    assert not session.close()
    session.landing_confirmed = True
    assert session.close()


def test_landing_cancels_goal_even_when_planner_answers_late(backend):
    feed(backend, armed=True)
    backend.flight.phase, backend.flight.kind = 'ready', 'operator'
    backend.goal_waiting, backend.goal_id = True, [10, 0]
    backend.submit('land', {})
    backend.on_plan_result(String(data=json.dumps(dict(status='PLANNED', goal_id=[10, 0], path_stamp=[10, 0]))))
    assert not backend.goal_waiting
    assert backend.commands.get_nowait() == ('finish', None)
    assert backend.commands.empty()


def test_goal_waits_for_matching_result_and_leader_acceptance(backend):
    feed(backend, armed=True)
    backend.flight.phase, backend.flight.kind = 'ready', 'operator'
    backend.goal_waiting, backend.goal_id = True, [10, 0]
    backend.goal_deadline = time.monotonic() + 10.
    backend.on_plan_result(String(data=json.dumps(dict(status='PLANNED', goal_id=[9, 0], path_stamp=[9, 0]))))
    backend.tick()
    assert backend.goal_waiting and backend.commands.empty()
    backend.on_plan_result(String(data=json.dumps(dict(status='PLANNED', goal_id=[10, 0], path_stamp=[10, 1]))))
    backend.tick()
    assert backend.goal_waiting
    backend.execution = dict(accepted_path_stamp=[10, 1])
    calls = []
    backend.call = lambda path: calls.append(path) or SimpleNamespace(done=lambda: False)
    backend.tick()
    assert calls == ['/leader/mission']


def test_missing_plan_expires_and_hold_can_cancel_pending_goal(backend):
    feed(backend, armed=True)
    backend.flight.phase, backend.flight.kind = 'ready', 'operator'
    backend.goal_waiting = True
    backend.goal_deadline = time.monotonic() - 1.
    backend.tick()
    assert not backend.goal_waiting and backend.action['state'] == 'error'
    backend.action['state'] = 'pending'
    backend.goal_waiting = True
    backend.submit('hold', {})
    assert not backend.goal_waiting
    assert backend.commands.get_nowait() == ('hold', None)


def test_error_of_finished_flight_still_requires_landing_confirmation(tmp_path):
    session = FlightSession(tmp_path)
    session.phase = 'error'
    session.landing_confirmed = False
    assert not session.close()
