"""Чтение SceneBroadcaster Pose_V без потери name и общего timestamp."""
from math import isfinite
import os
import re
import sys
import traceback
from threading import Lock

from geometry_msgs.msg import TransformStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


def convert_poses(message):
    """Только world-позы четырёх top-level моделей; порядок не используется."""
    output = TFMessage()
    for pose in message.pose:
        if not re.fullmatch(r'uav_[0-3]', pose.name):
            continue
        p, q = pose.position, pose.orientation
        if not all(isfinite(value) for value in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]):
            continue
        if q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w < 1e-12:
            continue
        transform = TransformStamped()
        transform.header.frame_id = 'map'
        transform.child_frame_id = pose.name
        transform.header.stamp.sec = message.header.stamp.sec
        transform.header.stamp.nanosec = message.header.stamp.nsec
        t, r = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = p.x, p.y, p.z
        r.x, r.y, r.z, r.w = q.x, q.y, q.z, q.w
        output.transforms.append(transform)
    return output


class GroundTruthReader(Node):
    def __init__(self):
        super().__init__('evaluation_ground_truth')
        self.declare_parameter('gz_topic', '/world/swarm/pose/info')
        self.topic = self.get_parameter('gz_topic').value
        self.publisher = self.create_publisher(TFMessage, '/evaluation/ground_truth', 10)
        self.transport = GzNode()
        self._lock = Lock()
        self._closing = False
        self.transport.subscribe(Pose_V, self.topic, self.on_poses)
        self.get_logger().info(f'GT только для evaluation: {self.topic}')

    def on_poses(self, message):
        converted = convert_poses(message)
        if converted.transforms:
            with self._lock:
                if not self._closing:
                    self.publisher.publish(converted)

    def destroy_node(self):
        with self._lock:
            self._closing = True
        self.transport.unsubscribe(self.topic)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    code = 0
    node = GroundTruthReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException:
        traceback.print_exc()
        code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Поток gz-transport может вызвать Python-обработчик уже во время
        # финализации интерпретатора (Fatal Python error, SIGABRT при Ctrl+C).
        # Ресурсы ROS закрыты выше: завершаемся без финализации.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
