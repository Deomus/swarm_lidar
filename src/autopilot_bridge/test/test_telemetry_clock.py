import pytest
import rclpy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleStatus

from autopilot_bridge.node import AutopilotBridge


@pytest.fixture
def bridge():
    rclpy.init()
    node = AutopilotBridge()
    clock = [10.]
    node.now = lambda: clock[0]
    yield node, clock
    node.destroy_node()
    rclpy.shutdown()


def test_future_px4_sample_waits_for_clock_without_invalidating_current_state(bridge):
    node, clock = bridge
    node.on_odom(VehicleOdometry(timestamp=10000000))
    node.on_local(VehicleLocalPosition(timestamp=10000000))
    node.on_status(VehicleStatus(timestamp=10000000))
    assert node.telemetry_fresh()
    node.on_odom(VehicleOdometry(timestamp=10004000))
    node.on_local(VehicleLocalPosition(timestamp=10004000))
    assert node.odom.timestamp == 10000000 and node.telemetry_fresh()
    clock[0] = 10.005
    node.advance_telemetry()
    assert node.odom.timestamp == 10004000 and node.telemetry_fresh()


def test_future_and_reordered_samples_cannot_replace_current_epoch(bridge):
    node, clock = bridge
    node.on_odom(VehicleOdometry(timestamp=10000000))
    node.on_odom(VehicleOdometry(timestamp=50000000))
    node.on_odom(VehicleOdometry(timestamp=9000000))
    node.advance_telemetry()
    assert node.odom.timestamp == 10000000
    assert not node.future_telemetry['odom']


def test_offboard_stream_mirrors_enu_setpoint_velocity(bridge):
    from swarm_interfaces.msg import SwarmCommand
    node, clock = bridge
    node.on_odom(VehicleOdometry(timestamp=10000000, q=[1., 0., 0., 0.],
                                 pose_frame=VehicleOdometry.POSE_FRAME_NED,
                                 velocity_frame=VehicleOdometry.VELOCITY_FRAME_NED))
    node.on_local(VehicleLocalPosition(timestamp=10000000, xy_valid=True, z_valid=True,
                                       v_xy_valid=True, v_z_valid=True))
    node.on_status(VehicleStatus(timestamp=10000000))
    command = SwarmCommand(uav_id=0, frame_id='map', valid=True)
    command.source_stamp.sec = 10
    command.target_velocity.linear.x, command.target_velocity.linear.y = 1., -.5
    command.target_velocity.angular.z = .2
    node.on_command(command)
    sent, observed = [], []
    node.setpoint_pub = type('Sink', (), dict(publish=lambda self, m: sent.append(m)))()
    node.setpoint_velocity_pub = type('Sink', (), dict(publish=lambda self, m: observed.append(m)))()
    node.stream_enabled = True
    node.tick()
    twist = observed[-1].twist
    assert (twist.linear.x, twist.linear.y, twist.angular.z) == pytest.approx((1., -.5, .2))
    assert list(sent[-1].velocity[:2]) == pytest.approx([-.5, 1.]) and sent[-1].yawspeed == pytest.approx(-.2)
