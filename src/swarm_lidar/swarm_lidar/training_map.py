"""Известная карта штатной площадки со столбами, без координат аппаратов."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def load_obstacles(path=None):
    """Столбы из config/obstacles.yaml: [(x, y)], радиус, м."""
    import yaml
    path = path or Path(get_package_share_directory('swarm_lidar')) / 'config' / 'obstacles.yaml'
    config = yaml.safe_load(Path(path).read_text())
    return [tuple(map(float, xy)) for xy in config.get('pillars') or []], float(config['radius'])


def training_map(pillars=(), radius=0.):
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 200
    grid.info.resolution = 1.
    grid.info.origin.position.x = grid.info.origin.position.y = -100.
    grid.info.origin.orientation.w = 1.
    data = [0] * 40000
    # Занята клетка, которой касается круг столба (запас — половина диагонали).
    reach = radius + math.sqrt(.5)
    for x, y in pillars:
        for cy in range(200):
            for cx in range(200):
                if math.hypot(cx - 100 + .5 - x, cy - 100 + .5 - y) <= reach:
                    data[cy * 200 + cx] = 100
    grid.data = data
    return grid


class TrainingMap(Node):
    def __init__(self):
        super().__init__('training_map')
        self.publisher = self.create_publisher(
            OccupancyGrid, '/map', QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        pillars, radius = load_obstacles()
        self.publisher.publish(training_map(pillars, radius))
        self.get_logger().info(f'Карта площадки опубликована: 200 × 200 м, столбов {len(pillars)}')


def main():
    rclpy.init()
    node = TrainingMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
