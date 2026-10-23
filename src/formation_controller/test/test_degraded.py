"""Потеря одного ведомого: остальные держат места, выбывший — препятствие.

Каждый ведомый считает свою команду сам; координатор на ведущем сводит
их статусы в режим группы.
"""

import json
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import Point
from rclpy.time import Time
from std_msgs.msg import String
from swarm_interfaces.msg import FormationPlan, RelativePose, SwarmState

from formation_controller.coordinator import FormationCoordinator
from formation_controller.core import formation_offsets
from formation_controller.node import FormationController

LINE = {1: (-4., -4.), 2: (-4., 0.), 3: (-4., 4.)}


def plan_message(clock, enabled=True, epoch=1):
    offsets = formation_offsets('line')
    points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in offsets]
    plan = FormationPlan(formation='line', scale=1., enabled=enabled, epoch=epoch, start_time=0.,
                         duration=1., start_offsets=points, end_offsets=points)
    plan.stamp = Time(nanoseconds=round(clock[0]*1e9)).to_msg()
    return plan


@pytest.fixture
def group():
    rclpy.init()
    clock = [100.]
    nodes, sinks = {}, {}
    for i in (1, 2, 3):
        node = FormationController(parameter_overrides=[rclpy.parameter.Parameter('uav_id', value=i)])
        sinks[i] = SimpleNamespace(commands=[], status=[])
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(clock[0]*1e9)))
        node.command_pub = SimpleNamespace(publish=sinks[i].commands.append)
        node.status_pub = SimpleNamespace(publish=lambda m, s=sinks[i]: s.status.append(json.loads(m.data)))
        node.setpoint_pose_pub = SimpleNamespace(publish=lambda _: None)
        nodes[i] = node
    yield nodes, clock, sinks
    for node in nodes.values():
        node.destroy_node()
    rclpy.shutdown()


def feed(node, clock, positions=LINE, valid=(1, 2, 3), modes=None, leader_x=0., plan=True, epoch=1):
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
    node.plan = plan_message(clock, epoch=epoch) if plan else None


def tick(group, **kwargs):
    nodes, clock, sinks = group
    commands, statuses = {}, {}
    for i, node in nodes.items():
        feed(node, clock, **kwargs)
        sinks[i].commands.clear()
        node.tick()
        commands[i], statuses[i] = sinks[i].commands[-1], sinks[i].status[-1]
    clock[0] += .05
    return commands, statuses


def test_each_follower_commands_only_itself(group):
    commands, statuses = tick(group)
    assert all(commands[i].uav_id == i and commands[i].valid for i in (1, 2, 3))
    assert all(statuses[i]['mode'] == 'FOLLOWING' and statuses[i]['uav_id'] == i for i in (1, 2, 3))


def test_one_lost_follower_does_not_stop_the_others(group):
    commands, statuses = tick(group, valid=(1, 3))
    assert commands[1].valid and commands[3].valid
    assert not commands[2].valid and commands[2].safety_state == 'DATA_HOLD'
    assert statuses[2]['mode'] == 'DATA_HOLD' and statuses[2]['error'] is None
    assert statuses[1]['error'] is not None


def test_follower_out_of_offboard_is_parked_not_failing(group):
    commands, statuses = tick(group, modes={3: '4'})
    assert not commands[3].valid
    # Данные годны: не отказ, мост не доводит эскалацию до посадки.
    assert commands[3].safety_state == 'PARKED'
    commands, _ = tick(group, modes={3: '4'}, valid=(1, 2))
    assert commands[3].safety_state == 'DATA_HOLD'


def test_lost_follower_stays_an_obstacle_where_it_was_measured(group):
    tick(group)
    # Ведущий сместился на 3 м к ведомому 2: его старое место теперь в 1 м.
    commands, statuses = tick(group, valid=(1, 3), leader_x=-3.,
                              positions={1: (-1., -4.), 2: (-1., 0.), 3: (-1., 4.)})
    for i in (1, 3):
        assert statuses[i]['mode'] == 'COLLISION_STOP'
        assert statuses[i]['min_distance'] == pytest.approx(1.)
        assert not commands[i].valid


def test_collision_stop_latches_until_next_enable(group):
    _, statuses = tick(group, positions={1: (-4., -4.), 2: (-4., -3.), 3: (-4., 4.)})
    assert statuses[1]['mode'] == 'COLLISION_STOP'
    # Аппараты разошлись, но координатор ещё не выключил строй: остановка держится.
    commands, statuses = tick(group)
    assert all(s['mode'] == 'COLLISION_STOP' for s in statuses.values())
    assert not any(c.valid for c in commands.values())
    # Новое включение (следующий epoch) снимает остановку.
    commands, statuses = tick(group, epoch=2)
    assert all(c.valid for c in commands.values())


def test_without_plan_followers_hold(group):
    commands, statuses = tick(group, plan=False)
    assert all(not c.valid and c.safety_state == 'DATA_HOLD' for c in commands.values())


@pytest.fixture
def coordinator():
    rclpy.init()
    node = FormationCoordinator()
    clock = [100.]
    node.now = lambda: clock[0]
    sink = SimpleNamespace(status=[], plans=[])
    node.status_pub = SimpleNamespace(publish=lambda m: sink.status.append(json.loads(m.data)))
    node.plan_pub = SimpleNamespace(publish=sink.plans.append)
    node.targets_pub = SimpleNamespace(publish=lambda _: None)
    leader = SwarmState(uav_id=0, valid=True, armed=True, frame_id='map')
    leader.timestamp.sec = 100
    node.leader = leader
    yield node, clock, sink
    node.destroy_node()
    rclpy.shutdown()


def report(node, clock, modes, ready=True, error=.1, distance=4.):
    for i, mode in modes.items():
        node.on_follower(String(data=json.dumps(dict(
            uav_id=i, time=clock[0], epoch=node.epoch, mode=mode, ready=ready, parked=False,
            error=error, yaw_error=0., min_distance=distance))))


def test_coordinator_enables_only_when_all_followers_ready(coordinator):
    node, clock, sink = coordinator
    from std_srvs.srv import Trigger
    report(node, clock, {1: 'DISABLED', 2: 'DISABLED'})
    assert not node.enable(Trigger.Request(), Trigger.Response()).success
    report(node, clock, {1: 'DISABLED', 2: 'DISABLED', 3: 'DISABLED'})
    assert node.enable(Trigger.Request(), Trigger.Response()).success and node.epoch == 1
    node.tick()
    assert sink.plans[-1].enabled and sink.plans[-1].epoch == 1 and len(sink.plans[-1].end_offsets) == 3


def test_coordinator_group_modes(coordinator):
    node, clock, sink = coordinator
    node.enabled, node.epoch = True, 1
    report(node, clock, {1: 'FOLLOWING', 2: 'FOLLOWING', 3: 'FOLLOWING'})
    node.tick()
    assert sink.status[-1]['mode'] == 'FOLLOWING' and sink.status[-1]['errors'] == [.1, .1, .1]
    report(node, clock, {1: 'FOLLOWING', 2: 'DATA_HOLD', 3: 'FOLLOWING'})
    node.tick()
    status = sink.status[-1]
    assert status['mode'] == 'DEGRADED' and status['lost'] == [2] and status['errors'][1] is None
    report(node, clock, {1: 'DATA_HOLD', 2: 'DATA_HOLD', 3: 'DATA_HOLD'})
    node.tick()
    assert sink.status[-1]['mode'] == 'DATA_HOLD' and sink.status[-1]['lost'] == [1, 2, 3]
    report(node, clock, {1: 'COLLISION_STOP', 2: 'FOLLOWING', 3: 'FOLLOWING'}, distance=1.)
    node.tick()
    assert sink.status[-1]['mode'] == 'COLLISION_STOP' and sink.status[-1]['min_distance'] == 1.
    assert not node.enabled and not sink.plans[-1].enabled


def test_coordinator_pauses_transition_while_degraded(coordinator):
    node, clock, sink = coordinator
    node.enabled, node.epoch = True, 1
    report(node, clock, {1: 'FOLLOWING', 2: 'FOLLOWING', 3: 'FOLLOWING'})
    node.tick()
    node.plan.request('column', 1., 20., clock[0])
    start = node.plan.snapshot()[2]
    clock[0] += 1.
    report(node, clock, {1: 'FOLLOWING', 2: 'DATA_HOLD', 3: 'FOLLOWING'})
    node.tick()
    assert node.plan.snapshot()[2] == pytest.approx(start + 1.)
