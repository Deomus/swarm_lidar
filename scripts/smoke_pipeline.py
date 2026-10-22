#!/usr/bin/env python3
"""Проверка ROS-цепочки на синтетическом облаке, не испытание SITL."""
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from swarm_interfaces.msg import RelativePose, SwarmState
from lidar_tracker.tracker_node import LidarTracker
from mesh_interface.node import MeshNode
from swarm_manager.node import SwarmManager


def main():
    rclpy.init()
    nodes = [LidarTracker(), MeshNode(), SwarmManager(), Node('pipeline_smoke')]
    tracker, mesh, manager, probe = nodes
    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    pub = probe.create_publisher(PointCloud2, '/uav_0/lidar/points', qos_profile_sensor_data)
    states = probe.create_publisher(SwarmState, '/swarm/states', 10)
    poses, health = {}, []
    probe.create_subscription(RelativePose, '/mesh/relative_poses',
                              lambda m: poses.update({m.target_id: m}), 10)
    probe.create_subscription(DiagnosticArray, '/swarm/health', health.append, 10)
    rng = np.random.default_rng(8)
    points = np.vstack([rng.normal(center, .02, (20, 3)) for center in
                        [(-4., -4., 0.), (-4., 0., 0.), (-4., 4., 0.)]])

    def step(seconds, cloud=True):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            stamp = probe.get_clock().now().to_msg()
            for i in range(4):
                msg = SwarmState(uav_id=i, timestamp=stamp, last_seen=stamp, valid=True)
                states.publish(msg)
            if cloud:
                pub.publish(create_cloud_xyz32(Header(stamp=stamp, frame_id='uav_0/lidar_link'), points))
            cycle = time.monotonic() + .08
            while time.monotonic() < cycle:
                executor.spin_once(timeout_sec=.001)

    try:
        step(2.)
        assert len(poses) == 3 and all(m.valid for m in poses.values()), 'Нет трёх целей'
        assert health and all(s.level == DiagnosticStatus.OK for s in health[-1].status), 'Группа не готова'
        print('OK: PointCloud2 → tracker → mesh → manager, три цели, четыре health status')
        step(1.5, cloud=False)
        assert all(not m.valid for m in poses.values()), 'Пропажа LiDAR не помечена invalid'
        assert all(s.message == 'HOLD' for s in health[-1].status[1:]), 'Нет HOLD при пропаже LiDAR'
        print('OK: при остановке облака три цели invalid, менеджер переходит в HOLD')
    finally:
        executor.shutdown()
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
