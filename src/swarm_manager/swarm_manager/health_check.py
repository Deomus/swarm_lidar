import time
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

def main(args=None):
    rclpy.init(args=args)
    node = Node('health_check')
    node.declare_parameter('expected_uavs', 4)
    node.declare_parameter('timeout', 5.)
    node.declare_parameter('health_topic', '/swarm/health')
    received = []
    node.create_subscription(DiagnosticArray, node.get_parameter('health_topic').value,
                             received.append, 10)
    deadline = time.monotonic() + node.get_parameter('timeout').value
    ok = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
        if received:
            statuses = {s.name: s for s in received[-1].status}
            expected = [f'uav_{i}' for i in range(node.get_parameter('expected_uavs').value)]
            ok = all(name in statuses and statuses[name].level == DiagnosticStatus.OK for name in expected)
            if ok:
                break
    node.get_logger().info('Проверка группы: ' + ('OK' if ok else 'FAIL'))
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)

