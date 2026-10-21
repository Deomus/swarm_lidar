import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from swarm_interfaces.msg import RelativePose, SwarmState
from .health import data_status

class SwarmManager(Node):
    def __init__(self):
        super().__init__('swarm_manager')
        defaults = dict(expected_uavs=4, poses_topic='/mesh/relative_poses',
                        states_topic='/mesh/states', health_topic='/swarm/health',
                        short_timeout=.3, long_timeout=1., period=.1, qos_depth=10)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.poses, self.states, self.previous = {}, {}, {}
        self.previous_clock = None
        self.pose_sub = self.create_subscription(RelativePose, self.p['poses_topic'], self.pose, 10)
        self.state_sub = self.create_subscription(SwarmState, self.p['states_topic'], self.state, 10)
        self.pub = self.create_publisher(DiagnosticArray, self.p['health_topic'], self.p['qos_depth'])
        self.timer = self.create_timer(self.p['period'], self.tick)

    def pose(self, msg):
        now = self.observe_clock()
        if not 0 <= now-self.seconds(msg.measurement_stamp) <= self.p['long_timeout']:
            return
        old = self.poses.get(msg.target_id)
        if old is None or self.seconds(msg.measurement_stamp) >= self.seconds(old.measurement_stamp):
            self.poses[msg.target_id] = msg

    def state(self, msg):
        now = self.observe_clock()
        if not 0 <= now-self.seconds(msg.timestamp) <= self.p['long_timeout']:
            return
        old = self.states.get(msg.uav_id)
        if old is None or self.seconds(msg.timestamp) >= self.seconds(old.timestamp):
            self.states[msg.uav_id] = msg

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def tick(self):
        self.observe_clock()
        now = self.get_clock().now()
        output = DiagnosticArray()
        output.header.stamp = now.to_msg()
        for i in range(self.p['expected_uavs']):
            state = self.states.get(i)
            telemetry = data_status(now.nanoseconds * 1e-9,
                self.seconds(state.timestamp) if state else None,
                bool(state and state.valid), self.p['short_timeout'], self.p['long_timeout'])
            pose = self.poses.get(i)
            tracking = 'FOLLOWING' if i == 0 else data_status(now.nanoseconds * 1e-9,
                self.seconds(pose.measurement_stamp) if pose else None,
                bool(pose and pose.valid), self.p['short_timeout'], self.p['long_timeout'])
            status = 'HOLD' if 'HOLD' in (telemetry, tracking) else (
                'DEGRADED' if 'DEGRADED' in (telemetry, tracking) else 'FOLLOWING')
            if self.previous.get(i) != status:
                self.get_logger().info(f'uav_{i}: {status}; telemetry={telemetry}, lidar={tracking}, t={now.nanoseconds}')
                self.previous[i] = status
            item = DiagnosticStatus()
            item.name = f'uav_{i}'
            item.hardware_id = f'sitl_{i}'
            item.level = {'FOLLOWING': DiagnosticStatus.OK,
                          'DEGRADED': DiagnosticStatus.WARN,
                          'HOLD': DiagnosticStatus.ERROR}[status]
            item.message = status
            item.values = [KeyValue(key='telemetry', value=telemetry),
                           KeyValue(key='lidar', value=tracking)]
            output.status.append(item)
        self.pub.publish(output)

    def observe_clock(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.previous_clock is not None and now < self.previous_clock:
            self.poses.clear()
            self.states.clear()
            self.previous.clear()
        self.previous_clock = now
        return now

def main(args=None):
    rclpy.init(args=args)
    node = SwarmManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

