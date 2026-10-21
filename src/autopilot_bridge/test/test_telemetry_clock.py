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
