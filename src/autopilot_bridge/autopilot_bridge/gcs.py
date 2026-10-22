"""Heartbeat программной станции управления SITL, без отключения preflight."""
import rclpy
from rclpy.node import Node
from pymavlink import mavutil

def main(args=None):
    rclpy.init(args=args)
    node = Node('gcs_heartbeat')
    node.declare_parameter('count', 4)
    node.declare_parameter('base_port', 18570)
    links = [mavutil.mavlink_connection(f'udpout:127.0.0.1:{node.get_parameter("base_port").value+i}',
                                      source_system=255, source_component=190)
             for i in range(node.get_parameter('count').value)]
    def beat():
        for link in links:
            link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                    mavutil.mavlink.MAV_STATE_ACTIVE)
    node.create_timer(1., beat)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for link in links:
            link.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

