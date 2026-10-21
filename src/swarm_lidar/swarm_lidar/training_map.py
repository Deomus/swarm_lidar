"""Известная свободная карта штатной площадки, без координат аппаратов."""

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def training_map():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 200
    grid.info.resolution = 1.
    grid.info.origin.position.x = grid.info.origin.position.y = -100.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 40000
    return grid


class TrainingMap(Node):
    def __init__(self):
        super().__init__('training_map')
        self.publisher = self.create_publisher(
            OccupancyGrid, '/map', QoSProfile(depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.publisher.publish(training_map())
        self.get_logger().info('Карта свободной площадки опубликована: 200 × 200 м')


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
