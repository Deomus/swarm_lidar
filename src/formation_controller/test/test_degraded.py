"""Потеря одного ведомого: остальные держат места, выбывший — препятствие."""

import json
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.time import Time
from swarm_interfaces.msg import RelativePose, SwarmState

from formation_controller.node import FormationController

LINE = {1: (-4., -4.), 2: (-4., 0.), 3: (-4., 4.)}


@pytest.fixture
def controller():
    rclpy.init()
    node = FormationController()
    clock = [100.]
    node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(clock[0]*1e9)))
    sink = SimpleNamespace(commands=[], status=[])
    node.command_pub = SimpleNamespace(publish=sink.commands.append)
    node.status_pub = SimpleNamespace(publish=lambda m: sink.status.append(json.loads(m.data)))
    node.targets_pub = SimpleNamespace(publish=lambda _: None)
    node.setpoint_pose_pubs = {i: SimpleNamespace(publish=lambda _: None) for i in (1, 2, 3)}
    node.enabled = True
    yield node, clock, sink
    node.destroy_node()
    rclpy.shutdown()


def feed(node, clock, positions=LINE, valid=(1, 2, 3), modes=None, leader_x=0.):
    stamp = Time(nanoseconds=round(clock[0]*1e9)).to_msg()
    node.orientation_history.add(clock[0], [0., 0., 0., 1.])
    for i in range(4):
        state = SwarmState(uav_id=i, frame_id='map', valid=True, armed=True,
                           mode=(modes or {}).get(i, '14'))
        state.timestamp = stamp
        state.pose.orientation.w = 1.
        state.pose.position.x = leader_x if i == 0 else 0.
        node.states[i] = state
    for i, (x, y) in positions.items():
        pose = RelativePose(target_id=i, valid=i in valid, confidence=1., frame_id='uav_0/lidar_link')
        pose.measurement_stamp = stamp
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = x, y, -1.9
        node.poses[i] = pose


def tick(node, clock, sink, **kwargs):
    feed(node, clock, **kwargs)
    sink.commands.clear()
    node.tick()
    clock[0] += .05
    return {c.uav_id: c for c in sink.commands}, sink.status[-1]


def test_one_lost_follower_degrades_without_stopping_the_others(controller):
    node, clock, sink = controller
    commands, status = tick(node, clock, sink)
    assert status['mode'] == 'FOLLOWING' and all(commands[i].valid for i in (1, 2, 3))
    commands, status = tick(node, clock, sink, valid=(1, 3))
    assert status['mode'] == 'DEGRADED' and status['lost'] == [2]
    assert commands[1].valid and commands[3].valid
    assert not commands[2].valid and commands[2].safety_state == 'DATA_HOLD'
    assert commands[1].safety_state == 'DEGRADED'
    assert status['errors'][1] is None and status['errors'][0] is not None
    commands, status = tick(node, clock, sink)
    assert status['mode'] == 'FOLLOWING' and status['lost'] == []


def test_follower_out_of_offboard_is_lost(controller):
    node, clock, sink = controller
    commands, status = tick(node, clock, sink, modes={3: '4'})
    assert status['lost'] == [3] and not commands[3].valid


def test_lost_follower_stays_an_obstacle_where_it_was_measured(controller):
    node, clock, sink = controller
    tick(node, clock, sink)
    # Ведущий сместился на 3 м к ведомому 2: его старое место теперь в 1 м.
    commands, status = tick(node, clock, sink, valid=(1, 3), leader_x=-3.,
                            positions={1: (-1., -4.), 2: (-1., 0.), 3: (-1., 4.)})
    assert status['mode'] == 'COLLISION_STOP' and status['min_distance'] == pytest.approx(1.)
    assert not any(commands[i].valid for i in (1, 2, 3))


def test_all_lost_is_data_hold(controller):
    node, clock, sink = controller
    commands, status = tick(node, clock, sink, valid=())
    assert status['mode'] == 'DATA_HOLD' and status['lost'] == [1, 2, 3]
    assert all(commands[i].safety_state == 'DATA_HOLD' for i in (1, 2, 3))
