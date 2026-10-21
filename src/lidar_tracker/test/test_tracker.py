import unittest
from types import SimpleNamespace

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from lidar_tracker.tracker_node import LidarTracker, cluster_points


def test_cloud_pipeline_reacquires_column_after_reset(monkeypatch):
    """Облако → кластеризация → треки → reset → те же ID после посадки."""
    rclpy.init()
    node = LidarTracker()
    messages = []
    node.publisher = SimpleNamespace(publish=messages.append)
    clock = [10.]
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0]*1e9))))
    line = np.array([[-4., -4., 0.], [-4., 0., 0.], [-4., 4., 0.]])
    column = np.array([[-8., 0., 0.], [-4., 0., 0.], [-12., 0., 0.]])
    noise = np.random.default_rng(7).normal(0., .01, (20, 3))

    def cloud(centers):
        nanos = round(clock[0]*1e9)
        stamp = Time(sec=nanos//10**9, nanosec=nanos % 10**9)
        points = np.vstack([noise+center for center in centers])
        node.pointcloud_callback(create_cloud_xyz32(
            Header(stamp=stamp, frame_id='uav_0/lidar_link'), points))

    try:
        for step in range(61):
            clock[0] = 10.+step*.1
            cloud((line+(column-line)*step/60)[::-1])
            assert all(m.valid for m in messages[-3:])
        clock[0] += 10.
        cloud(column)
        assert all(not m.valid for m in messages[-3:])
        response = node.reset(Trigger.Request(), Trigger.Response())
        assert response.success and 'LiDAR' in response.message
        clock[0] += .1
        cloud(column[::-1])
        for index, message in enumerate(messages[-3:]):
            assert message.valid and message.target_id == index+1
            p = message.pose.position
            np.testing.assert_allclose([p.x, p.y, p.z], column[index], atol=.03)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_clock_rewind_accepts_new_cloud_epoch(monkeypatch):
    rclpy.init()
    node = LidarTracker()
    clock = [100.]
    monkeypatch.setattr(node, 'get_clock', lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=round(clock[0]*1e9))))
    messages = []
    node.publisher = SimpleNamespace(publish=messages.append)
    points = np.vstack([np.array([x, y, 0.]) + np.random.default_rng(7).normal(0., .01, (12, 3))
                        for x, y in [(-4., -4.), (-4., 0.), (-4., 4.)]])
    try:
        for stamp in [100, 1]:
            clock[0] = float(stamp)
            node.pointcloud_callback(create_cloud_xyz32(Header(stamp=Time(sec=stamp),
                frame_id='uav_0/lidar_link'), points))
            assert all(m.valid for m in messages[-3:])
            assert node.last_stamp.sec == stamp
    finally:
        node.destroy_node()
        rclpy.shutdown()


class TestClusterPoints(unittest.TestCase):
    def test_separates_targets(self):
        points = np.array([
            [1.0, 0.0, 0.0], [1.1, 0.0, 0.0], [1.0, 0.1, 0.0],
            [5.0, 0.0, 0.0], [5.1, 0.0, 0.0], [5.0, 0.1, 0.0],
        ])
        clusters = cluster_points(points, tolerance=0.3, min_size=3)
        self.assertEqual(len(clusters), 2)

    def test_discards_noise(self):
        points = np.array([[1.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
        self.assertEqual(cluster_points(points, tolerance=0.3, min_size=3), [])
