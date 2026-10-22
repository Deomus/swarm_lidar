"""Единый режим группы §8 по состояниям узлов."""

import pytest

from swarm_manager.modes import system_mode


def drones(armed=True, nav='14', health='OK', **override):
    rows = {i: dict(armed=armed, nav_state=nav, health=health) for i in range(4)}
    for i, values in override.items():
        rows[int(i[1:])].update({('nav_state' if k == 'nav' else k): v for k, v in values.items()})
    return rows


FOLLOWING = dict(mode='FOLLOWING', errors=[.1, .2, .1], formation='line', scale=1.)


@pytest.mark.parametrize('state,formation,leader,expected', [
    (drones(armed=False, nav='4'), None, None, 'DISARMED'),
    (drones(nav='17', u2=dict(armed=False)), None, None, 'TAKEOFF'),
    (drones(), dict(mode='DISABLED'), None, 'FORMING'),
    (drones(), dict(FOLLOWING, errors=[.1, 1.2, .1]), None, 'FORMING'),
    (drones(), FOLLOWING, dict(mode='HOLD', status='HOLD'), 'HOLD'),
    (drones(), FOLLOWING, dict(mode='MANUAL', status='MANUAL'), 'FOLLOWING'),
    (drones(), FOLLOWING, dict(mode='MISSION', waypoint=0, total=2), 'MISSION'),
    (drones(), dict(FOLLOWING, mode='FORMATION_CHANGE'), None, 'FORMATION_CHANGE'),
    (drones(), dict(mode='DEGRADED', lost=[2]), None, 'DEGRADED'),
    (drones(u3=dict(nav='18')), FOLLOWING, None, 'LAND'),
    (drones(u2=dict(nav='4', health='FAILSAFE_HOLD')), dict(mode='DEGRADED', lost=[2]), None, 'FAILSAFE'),
    (drones(), dict(mode='COLLISION_STOP'), None, 'FAILSAFE'),
])
def test_modes_cover_every_tz_state(state, formation, leader, expected):
    assert system_mode(state, formation, leader)[0] == expected


def test_reason_names_the_cause():
    mode, reason = system_mode(drones(u2=dict(nav='4', health='FAILSAFE_HOLD')), None, None)
    assert mode == 'FAILSAFE' and 'uav_2: FAILSAFE_HOLD' in reason
    mode, reason = system_mode({0: drones()[0]}, None, None)
    assert mode == 'DEGRADED' and 'uav_1, uav_2, uav_3' in reason
    assert 'uav_2' in system_mode(drones(), dict(mode='DEGRADED', lost=[2]), None)[1]


def test_manager_publishes_mode_and_logs_transition_with_reason():
    import json
    from types import SimpleNamespace

    import rclpy
    from rclpy.time import Time
    from std_msgs.msg import String
    from swarm_interfaces.msg import SwarmState

    from swarm_manager.node import SwarmManager
    rclpy.init()
    try:
        node = SwarmManager()
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=100 * 10**9))
        published, logged = [], []
        node.mode_pub = SimpleNamespace(publish=lambda m: published.append(json.loads(m.data)))
        node.pub = SimpleNamespace(publish=lambda m: None)
        node.get_logger = lambda: SimpleNamespace(info=logged.append, warning=logged.append)
        for i in range(4):
            state = SwarmState(uav_id=i, valid=True, armed=True, mode='14', health='OK')
            state.timestamp.sec = 100
            node.state(state)
        node.status(String(data=json.dumps(dict(mode='DEGRADED', lost=[1], time=100.))), 'formation', 'time')
        node.tick()
        assert published[-1]['mode'] == 'DEGRADED' and 'uav_1' in published[-1]['reason']
        assert any('Режим группы: None → DEGRADED' in line and 'причина' in line and 't=100' in line
                   for line in logged)
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_real_logger_accepts_alternating_severities():
    """В полёте режимы чередуют warning и info; rclpy не должен падать."""
    import rclpy

    from swarm_manager.node import SwarmManager
    rclpy.init()
    try:
        node = SwarmManager()
        for now in (1., 2., 3.):
            node.states.clear()
            node.supervise(now)  # DEGRADED: нет телеметрии — warning
            node.mode = 'DEGRADED' if now < 3. else None
            node.states.update({i: _state(i, now) for i in range(4)})
            node.supervise(now)  # DISARMED — info
        node.destroy_node()
    finally:
        rclpy.shutdown()


def _state(i, now):
    from swarm_interfaces.msg import SwarmState
    state = SwarmState(uav_id=i, valid=True, armed=False, health='OK')
    state.timestamp.sec = int(now)
    return state
