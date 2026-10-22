"""Ведомые держат курс лидера; позиция по-прежнему только из LiDAR."""

import math
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from swarm_interfaces.msg import RelativePose, SwarmState

from formation_controller.core import heading_rate, wrap_angle
from formation_controller.node import FormationController


def test_wrap_takes_short_side_across_pi():
    assert wrap_angle(3. - (-3.)) == pytest.approx(6. - 2*math.pi)
    assert wrap_angle(-.2) == pytest.approx(-.2)


def test_heading_rate_feedforward_correction_and_limit():
    assert heading_rate(.5, .1, .5, 1., .3) == pytest.approx(.1)
    assert heading_rate(.6, 0., .5, 1., .3) == pytest.approx(.1)
    assert heading_rate(3., .1, -3., 1., .3) == pytest.approx(.1 + 6. - 2*math.pi)
    assert heading_rate(1.5, .2, 0., 1., .3) == .3


def run_tick(match, follower_yaws, leader_yaw=3., leader_yaw_rate=.1, valid=True, targets=None):
    rclpy.init(args=['--ros-args', '-p', f'match_leader_yaw:={str(match).lower()}'])
    try:
        node = FormationController()
        now = 100.
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(now*1e9)))
        commands = []
        node.command_pub = SimpleNamespace(publish=commands.append)
        node.status_pub = node.targets_pub = SimpleNamespace(publish=lambda _: None)
        if targets is not None:
            node.setpoint_pose_pubs = {i: SimpleNamespace(publish=lambda m, i=i: targets.update({i: m}))
                                       for i in (1, 2, 3)}
        stamp = Time(nanoseconds=round(now*1e9)).to_msg()
        quaternion = Rotation.from_euler('z', leader_yaw).as_quat()
        node.orientation_history.add(now, quaternion)
        for i, yaw in enumerate([leader_yaw, *follower_yaws]):
            state = SwarmState(uav_id=i, frame_id='map', valid=True, armed=True, yaw=yaw,
                               yaw_rate=leader_yaw_rate if i == 0 else 0.)
            state.timestamp = stamp
            state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z, \
                state.pose.orientation.w = quaternion
            node.states[i] = state
        for i, y in zip([1, 2, 3], [-4., 0., 4.]):
            pose = RelativePose(target_id=i, valid=valid, confidence=1., frame_id='uav_0/lidar_link')
            pose.measurement_stamp = stamp
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = -4., y, -1.9
            node.poses[i] = pose
        node.enabled = True
        node.tick()
        node.destroy_node()
        return {c.uav_id: c for c in commands}
    finally:
        rclpy.shutdown()


def test_followers_turn_towards_leader_heading_by_short_side():
    commands = run_tick(True, [3., 2.9, -3.])
    assert all(commands[i].valid for i in (1, 2, 3))
    assert commands[1].target_velocity.angular.z == pytest.approx(.1)
    assert commands[2].target_velocity.angular.z == pytest.approx(.2)
    assert commands[3].target_velocity.angular.z == pytest.approx(.1 + 6. - 2*math.pi)


def test_heading_hold_disabled_or_data_hold_gives_zero_yaw_rate():
    for commands in (run_tick(False, [2., 2., 2.]), run_tick(True, [2., 2., 2.], valid=False)):
        assert all(commands[i].target_velocity.angular.z == 0. for i in (1, 2, 3))


def test_slot_setpoints_are_leader_position_plus_rotated_offsets():
    targets = {}
    run_tick(True, [3., 3., 3.], leader_yaw=math.pi/2, targets=targets)
    # Линия за ведущим, смотрящим на север: места на юге, с курсом ведущего.
    expected = {1: (4., -4.), 2: (0., -4.), 3: (-4., -4.)}
    for i, (x, y) in expected.items():
        p = targets[i].pose.position
        assert (p.x, p.y) == pytest.approx((x, y)) and targets[i].header.frame_id == 'map'
        q = targets[i].pose.orientation
        assert Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2] == pytest.approx(math.pi/2)
    assert run_tick(True, [3., 3., 3.], valid=False, targets=(empty := {})) and not empty
