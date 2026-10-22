import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_srvs.srv import Trigger
from swarm_interfaces.msg import RelativePose, SwarmState

from .tracking import TargetTracker, cluster_points, remove_ground, level_heights
from .timing import OrientationHistory


class LidarTracker(Node):
    def __init__(self):
        super().__init__('lidar_tracker')
        defaults = dict(
            input_topic='/uav_0/lidar/points', output_topic='/swarm/relative_poses',
            target_ids=[1, 2, 3], initial_positions=[-4., -4., 0., -4., 0., 0., -4., 4., 0.],
            min_range=0.5, max_range=80., min_z=-10., max_z=10.,
            ground_filter=False, ground_distance=.04, ground_min_points=1000,
            ground_fraction=.7,
            level_filter=False, sensor_offset=[0., 0., 1.9], orientation_topic='/swarm/states',
            max_processing_points=10000,
            cluster_tolerance=0.5, min_cluster_size=3, max_cluster_size=500,
            max_cluster_extent=1.5, confidence_threshold=0.25,
            association_gate=2., ambiguity_margin=0.2, filter_alpha=0.7,
            stale_timeout=0.5, reacquire_timeout=2., check_period=0.1,
            expected_frame='uav_0/lidar_link', qos_depth=10, cloud_qos_depth=1)
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.params = {name: self.get_parameter(name).value for name in defaults}
        p = self.params
        self.ids = list(p['target_ids'])
        if len(p['initial_positions']) != len(self.ids) * 3 or len(set(self.ids)) != len(self.ids):
            raise ValueError('Нужны уникальные ID и три начальные координаты на каждую цель')
        if not 0 < p['filter_alpha'] <= 1 or p['cluster_tolerance'] <= 0:
            raise ValueError('Некорректные параметры фильтра или кластеризации')
        if (not np.isfinite(p['ground_distance']) or p['ground_distance'] <= 0
                or p['ground_min_points'] < 3 or not 0 < p['ground_fraction'] <= 1):
            raise ValueError('Некорректные параметры удаления земли')
        if p['cloud_qos_depth'] < 1:
            raise ValueError('Очередь облаков должна содержать хотя бы одно сообщение')
        self.tracker = TargetTracker(p['initial_positions'], p['association_gate'],
                                     p['ambiguity_margin'], p['filter_alpha'],
                                     p['reacquire_timeout'])
        self.last_stamp = None
        self.previous_clock = None
        self.rejection_reason = None
        self.orientation_history = OrientationHistory()
        self.create_subscription(SwarmState, p['orientation_topic'], self.on_orientation, 10)
        self.publisher = self.create_publisher(RelativePose, p['output_topic'], p['qos_depth'])
        self.subscription = self.create_subscription(
            PointCloud2, p['input_topic'], self.pointcloud_callback,
            QoSProfile(depth=p['cloud_qos_depth'], reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(p['check_period'], self.check_stale)
        self.reset_service = self.create_service(Trigger, '/lidar_tracker/reset', self.reset)

    def reset(self, request, response):
        retained = self.tracker.reset(self.get_clock().now().nanoseconds * 1e-9)
        self.last_stamp = None
        response.success = True
        response.message = ('Повторный поиск по последнему полному LiDAR-кадру'
                            if retained else 'Первичный поиск по initial_positions из YAML')
        self.get_logger().info(response.message)
        return response

    def publish(self, stamp, results=None, confidences=None):
        results = results or [None] * len(self.ids)
        for target_id, result in zip(self.ids, results):
            msg = RelativePose()
            msg.target_id = target_id
            msg.measurement_stamp = stamp
            msg.frame_id = self.params['expected_frame']
            msg.child_frame_id = f'uav_{target_id}/base_link'
            msg.pose.orientation.w = 1.
            if result is not None:
                position, index = result
                msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
                msg.distance = float(np.linalg.norm(position))
                msg.confidence = float(confidences[index])
                msg.valid = True
            self.publisher.publish(msg)

    def check_stale(self):
        self.observe_clock()
        if self.last_stamp is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = self.last_stamp.sec + self.last_stamp.nanosec * 1e-9
        if now - stamp > self.params['stale_timeout']:
            self.publish(self.last_stamp)

    def pointcloud_callback(self, cloud):
        self.rejection_reason = None
        self.observe_clock()
        p = self.params
        stamp = cloud.header.stamp.sec + cloud.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        previous = None if self.last_stamp is None else (
            self.last_stamp.sec + self.last_stamp.nanosec * 1e-9)
        if cloud.header.frame_id != p['expected_frame'] or age < -0.1 or age > p['stale_timeout']:
            self.publish(cloud.header.stamp)
            return
        if previous is not None and stamp <= previous:
            self.publish(cloud.header.stamp)
            return
        self.last_stamp = cloud.header.stamp
        try:
            points = point_cloud2.read_points_numpy(
                cloud, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        except (ValueError, AssertionError, KeyError):
            self.rejection_reason = 'pointcloud_decode_rejected'
            self.publish(cloud.header.stamp)
            return
        points = points[np.isfinite(points).all(axis=1)]
        distances = np.linalg.norm(points, axis=1)
        points = points[(distances >= p['min_range']) & (distances <= p['max_range'])]
        if p['ground_filter']:
            points = remove_ground(points, distance=p['ground_distance'],
                                   min_points=p['ground_min_points'], fraction=p['ground_fraction'])
        try:
            heights = (level_heights(points, self.orientation_history.at(stamp), p['sensor_offset'])
                       if p['level_filter'] else points[:, 2])
        except ValueError:
            self.rejection_reason = 'orientation_unavailable'
            self.publish(cloud.header.stamp)
            return
        points = points[(heights >= p['min_z']) & (heights <= p['max_z'])]
        if len(points) > p['max_processing_points']:
            self.rejection_reason = 'processing_budget_exceeded'
            self.publish(cloud.header.stamp)
            return
        centers, confidences = [], []
        for cluster in cluster_points(points, p['cluster_tolerance'],
                                      p['min_cluster_size'], p['max_cluster_size']):
            if np.max(np.ptp(cluster, axis=0)) > p['max_cluster_extent']:
                continue
            center = np.mean(cluster, axis=0)
            spread = float(np.mean(np.linalg.norm(cluster - center, axis=1)))
            confidence = max(0., 1. - spread / p['cluster_tolerance'])
            if confidence >= p['confidence_threshold']:
                centers.append(center)
                confidences.append(confidence)
        self.publish(cloud.header.stamp, self.tracker.update(centers, stamp), confidences)

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.previous_clock is not None and now < self.previous_clock:
            self.last_stamp = None
            self.tracker.reset(now)
            self.orientation_history.clear()
        self.previous_clock = now

    def on_orientation(self, msg):
        self.observe_clock()
        stamp = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if msg.uav_id == 0 and msg.valid and 0 <= now-stamp <= self.params['stale_timeout']:
            q = msg.pose.orientation
            try:
                self.orientation_history.add(stamp, [q.x, q.y, q.z, q.w])
            except ValueError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = LidarTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
