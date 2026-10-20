import math
from collections import deque

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from swarm_interfaces.msg import RelativePose


def cluster_points(points, tolerance=0.8, min_size=3, max_size=5000):
    """Return connected point clusters using a deterministic radius search."""
    if len(points) == 0:
        return []
    data = np.asarray(points, dtype=float)
    visited = np.zeros(len(data), dtype=bool)
    clusters = []
    radius_sq = tolerance * tolerance
    for start in range(len(data)):
        if visited[start]:
            continue
        visited[start] = True
        queue = deque([start])
        indices = []
        while queue:
            current = queue.popleft()
            indices.append(current)
            distances = np.sum((data - data[current]) ** 2, axis=1)
            neighbours = np.flatnonzero((distances <= radius_sq) & ~visited)
            visited[neighbours] = True
            queue.extend(neighbours.tolist())
        if min_size <= len(indices) <= max_size:
            clusters.append(data[indices])
    return clusters


class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_tracker')
        self.declare_parameter('input_topic', '/uav_0/lidar/points')
        self.declare_parameter('output_topic', '/swarm/relative_poses')
        self.declare_parameter('target_ids', [1, 2, 3])
        self.declare_parameter('min_range', 0.5)
        self.declare_parameter('max_range', 80.0)
        self.declare_parameter('min_z', -10.0)
        self.declare_parameter('max_z', 10.0)
        self.declare_parameter('cluster_tolerance', 0.8)
        self.declare_parameter('min_cluster_size', 3)
        self.declare_parameter('max_cluster_size', 5000)
        self.declare_parameter('confidence_threshold', 0.25)
        self.target_ids = [int(x) for x in self.get_parameter('target_ids').value]
        self.last_positions = {}
        self.publisher = self.create_publisher(
            RelativePose, self.get_parameter('output_topic').value, 10)
        self.subscription = self.create_subscription(
            PointCloud2, self.get_parameter('input_topic').value,
            self.pointcloud_callback, 10)

    def pointcloud_callback(self, cloud):
        points = []
        min_range = float(self.get_parameter('min_range').value)
        max_range = float(self.get_parameter('max_range').value)
        min_z = float(self.get_parameter('min_z').value)
        max_z = float(self.get_parameter('max_z').value)
        for p in point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = (float(p[0]), float(p[1]), float(p[2]))
            distance = math.sqrt(x * x + y * y + z * z)
            if min_range <= distance <= max_range and min_z <= z <= max_z:
                points.append((x, y, z))
        clusters = cluster_points(
            points,
            float(self.get_parameter('cluster_tolerance').value),
            int(self.get_parameter('min_cluster_size').value),
            int(self.get_parameter('max_cluster_size').value))
        measurements = []
        for cluster in clusters:
            centroid = np.mean(cluster, axis=0)
            spread = float(np.mean(np.linalg.norm(cluster - centroid, axis=1)))
            confidence = max(0.0, min(1.0, 1.0 - spread / max(1.0, float(
                self.get_parameter('cluster_tolerance').value))))
            if confidence >= float(self.get_parameter('confidence_threshold').value):
                measurements.append((centroid, confidence, len(cluster)))
        measurements.sort(key=lambda item: (float(item[0][0] ** 2 + item[0][1] ** 2), -item[2]))
        for target_id, previous in zip(self.target_ids, measurements):
            centroid, confidence, _ = previous
            pose = RelativePose()
            pose.target_id = target_id
            pose.pose.position.x = float(centroid[0])
            pose.pose.position.y = float(centroid[1])
            pose.pose.position.z = float(centroid[2])
            pose.pose.orientation.w = 1.0
            pose.distance = float(np.linalg.norm(centroid))
            pose.confidence = float(confidence)
            pose.measurement_stamp = cloud.header.stamp
            pose.valid = True
            pose.frame_id = cloud.header.frame_id
            pose.child_frame_id = f'uav_{target_id}/base_link'
            self.publisher.publish(pose)
            self.last_positions[target_id] = centroid
        for target_id in self.target_ids[len(measurements):]:
            pose = RelativePose()
            pose.target_id = target_id
            pose.measurement_stamp = cloud.header.stamp
            pose.valid = False
            pose.confidence = 0.0
            pose.frame_id = cloud.header.frame_id
            pose.child_frame_id = f'uav_{target_id}/base_link'
            self.publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
