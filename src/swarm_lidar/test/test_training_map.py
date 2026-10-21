"""Планирование на штатной площадке без предварительного запуска S3."""

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from swarm_interfaces.msg import SwarmState

from mission_planner.grid import PlanningError
from mission_planner.node import MissionPlanner
from swarm_lidar.training_map import TrainingMap, training_map


def test_training_map_supports_default_planner_and_boundary_guard():
    rclpy.init()
    planner = MissionPlanner()
    try:
        planner.on_map(training_map())
        state = SwarmState(uav_id=0, frame_id='map', valid=True)
        state.timestamp = planner.get_clock().now().to_msg()
        planner.on_state(state)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x, goal.pose.position.y = 12., -5.
        path = planner.plan(goal)
        assert len(path.poses) > 1
        assert path.poses[-1].pose.position.x == 12.
        goal.pose.position.x = 99.
        with pytest.raises(PlanningError, match='inflation'):
            planner.plan(goal)
    finally:
        planner.destroy_node()
        rclpy.shutdown()


def test_map_is_delivered_to_planner_started_later():
    rclpy.init()
    source = TrainingMap()
    planner = MissionPlanner()
    try:
        # Издатель уже отправил карту до появления подписки планировщика.
        for _ in range(100):
            rclpy.spin_once(planner, timeout_sec=.02)
            if planner.map is not None:
                break
        assert planner.map is not None
        assert planner.map.info.width == 200
        assert len(planner.map.data) == 40000
    finally:
        planner.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
