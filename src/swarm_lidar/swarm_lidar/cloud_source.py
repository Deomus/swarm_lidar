"""Пространственный предфильтр настоящего облака Gazebo перед передачей DDS."""

import math
import os
from pathlib import Path
import sys
import traceback
from threading import Lock

from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
from gz.transport13 import Node as GzNode
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from swarm_interfaces.msg import SwarmState
from lidar_tracker.timing import OrientationHistory
from lidar_tracker.tracking import level_heights
import yaml


def load_filter_config(path):
    """Единственный источник границ — действующий YAML трекера."""
    config = yaml.safe_load(Path(path).read_text())['lidar_tracker']['ros__parameters']
    keys = ('min_range', 'max_range', 'min_z', 'max_z')
    limits = {}
    for key in keys:
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'Некорректная граница {key}')
        limits[key] = float(value)
    if (limits['min_range'] < 0 or limits['max_range'] <= limits['min_range']
            or limits['max_z'] < limits['min_z']):
        raise ValueError('Перепутаны границы пространственного фильтра')
    for key in ('expected_frame', 'input_topic'):
        if not isinstance(config[key], str) or not config[key]:
            raise ValueError(f'Не задан {key}')
        limits[key] = config[key]
    limits['level_filter'] = config.get('level_filter', False)
    limits['sensor_offset'] = config.get('sensor_offset', [0., 0., 1.9])
    if not isinstance(limits['level_filter'], bool) or len(limits['sensor_offset']) != 3 or not all(math.isfinite(v) for v in limits['sensor_offset']):
        raise ValueError('Некорректные параметры компенсации наклона')
    return limits


def compact_cloud(message, config, rotation=None):
    """Сохраняет измеренные конечные XYZ внутри тех же четырёх границ."""
    frame_values = [value for item in message.header.data if item.key == 'frame_id' for value in item.value]
    if frame_values != [config['expected_frame']]:
        raise ValueError('Неожиданная или неоднозначная система координат облака')
    sec, nsec = message.header.stamp.sec, message.header.stamp.nsec
    if not 0 <= sec <= 2147483647 or not 0 <= nsec < 1_000_000_000:
        raise ValueError('Некорректное время измерения облака')
    width, height = message.width, message.height
    step, row = message.point_step, message.row_step
    if step < 12 or row < width*step or len(message.data) != row*height:
        raise ValueError('Некорректный размер или шаг строк облака')
    fields = {}
    for field in message.field:
        if field.name in ('x', 'y', 'z'):
            if (field.name in fields or field.count != 1
                    or field.datatype != PointCloudPacked.Field.FLOAT32
                    or field.offset+4 > step):
                raise ValueError('Некорректные или повторные поля XYZ FLOAT32')
            fields[field.name] = field.offset
    if set(fields) != {'x', 'y', 'z'}:
        raise ValueError('В облаке отсутствуют XYZ')
    offsets = [fields[name] for name in ('x', 'y', 'z')]
    if any(abs(a-b) < 4 for i, a in enumerate(offsets) for b in offsets[i+1:]):
        raise ValueError('Поля XYZ пересекаются')
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[endian+'f4']*3,
                         offsets=offsets, itemsize=step))
    structured = np.ndarray((height, width), dtype=dtype, buffer=message.data,
                            strides=(row, step))
    points = np.column_stack([structured[name].reshape(-1) for name in ('x', 'y', 'z')])
    points = points[np.isfinite(points).all(axis=1)]
    distances = np.linalg.norm(points, axis=1)
    points = points[(distances >= config['min_range']) & (distances <= config['max_range'])]
    if config.get('level_filter', False) and rotation is None:
        raise ValueError('Нет ориентации на время LiDAR-кадра')
    heights = (level_heights(points, rotation, config['sensor_offset'])
               if config.get('level_filter', False) else points[:, 2])
    points = points[(heights >= config['min_z']) & (heights <= config['max_z'])]
    points = np.asarray(points, dtype='<f4')
    output = PointCloud2()
    output.header.frame_id = frame_values[0]
    output.header.stamp.sec, output.header.stamp.nanosec = sec, nsec
    output.height, output.width = 1, len(points)
    output.fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
                     for i, name in enumerate(('x', 'y', 'z'))]
    output.is_bigendian, output.is_dense = False, True
    output.point_step, output.row_step = 12, len(points)*12
    output.data = points.tobytes()
    return output


class CloudSource(Node):
    def __init__(self):
        super().__init__('lidar_cloud_source')
        self.declare_parameter('filter_config', '')
        self.declare_parameter('gz_topic', '/uav_0/lidar/scan/points')
        path = self.get_parameter('filter_config').value
        if not path:
            raise ValueError('Нужен filter_config с действующим tracker.yaml')
        self.config = load_filter_config(path)
        self.topic = self.get_parameter('gz_topic').value
        self.orientation_history = OrientationHistory()
        self.previous_clock = None
        self.create_subscription(SwarmState, '/swarm/states', self.on_orientation, 10)
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(PointCloud2, self.config['input_topic'], qos)
        self._lock, self._closing, self._last_error = Lock(), False, None
        self.transport = GzNode()
        if not self.transport.subscribe(PointCloudPacked, self.topic, self.on_cloud):
            raise RuntimeError('Не удалось подписаться на облако Gazebo: '+self.topic)
        self.get_logger().info('Измеренные XYZ после пространственного предфильтра: '+self.topic)

    def on_cloud(self, message):
        with self._lock:
            if self._closing:
                return
        try:
            stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
            rotation = self.orientation_history.at(stamp) if self.config['level_filter'] else None
            converted = compact_cloud(message, self.config, rotation)
        except (ValueError, TypeError, OverflowError) as error:
            with self._lock:
                if not self._closing and str(error) != self._last_error:
                    self._last_error = str(error)
                    self.get_logger().warning('Облако отклонено: '+str(error))
            return
        with self._lock:
            if not self._closing:
                self.publisher.publish(converted)

    def on_orientation(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.previous_clock is not None and now < self.previous_clock:
            self.orientation_history.clear()
        self.previous_clock = now
        stamp = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        if msg.uav_id == 0 and msg.valid and 0 <= now-stamp <= .5:
            q = msg.pose.orientation
            try:
                self.orientation_history.add(stamp, [q.x, q.y, q.z, q.w])
            except ValueError:
                pass

    def destroy_node(self):
        with self._lock:
            self._closing = True
        self.transport.unsubscribe(self.topic)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    code = 0
    node = None
    try:
        node = CloudSource()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except BaseException:
        traceback.print_exc()
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # Поток gz-transport может вызвать Python-обработчик уже во время
        # финализации интерпретатора (Fatal Python error, SIGABRT при Ctrl+C).
        # Ресурсы ROS закрыты выше: завершаемся без финализации.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
