#!/usr/bin/env python3
"""Проверка настоящего PointCloud2 Gazebo и трёх треков до взлёта."""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from swarm_interfaces.msg import RelativePose


def main():
    rclpy.init()
    node = Node('lidar_smoke', parameter_overrides=[Parameter('use_sim_time', value=True)])
    clouds, poses, seen = [], {}, {1: 0, 2: 0, 3: 0}
    def pose(msg):
        poses[msg.target_id] = msg
        if msg.valid:
            seen[msg.target_id] += 1
    subs = [node.create_subscription(PointCloud2, '/uav_0/lidar/points', clouds.append, qos_profile_sensor_data),
            node.create_subscription(RelativePose, '/mesh/relative_poses', pose, 10)]
    deadline = time.monotonic() + 60.
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            if clouds and all(v >= 10 for v in seen.values()):
                break
            # Не держим десятки мегабайт облаков в памяти.
            if len(clouds) > 1:
                del clouds[:-1]
        assert clouds, 'Нет настоящего облака Gazebo'
        assert clouds[-1].header.frame_id == 'uav_0/lidar_link', clouds[-1].header.frame_id
        assert all(v >= 10 for v in seen.values()), f'Недостаточно валидных измерений: {seen}'
        expected = {1: [-4., -4., -1.45], 2: [-4., 0., -1.45], 3: [-4., 4., -1.45]}
        for i, msg in poses.items():
            p = msg.pose.position
            error = np.linalg.norm(np.array([p.x, p.y, p.z]) - expected[i])
            assert msg.valid and error < .5, f'Цель {i}, ошибка относительно nominal slot: {error}'
            print(f'OK: uav_{i} xyz=({p.x:.3f},{p.y:.3f},{p.z:.3f}), confidence={msg.confidence:.3f}, n={seen[i]}')
        print(f'OK: реальное облако Gazebo {clouds[-1].width}×{clouds[-1].height}, три устойчивых ID')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
