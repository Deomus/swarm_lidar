import rclpy
from rclpy.node import Node
from swarm_interfaces.msg import RelativePose, SwarmState, SwarmCommand
from .channel import DelayChannel

class MeshNode(Node):
    def __init__(self):
        super().__init__('mesh_interface')
        defaults = dict(delay=0., jitter=0., loss=0., capacity=1000, seed=42,
                        period=0.01, qos_depth=10,
                        poses_in='/swarm/relative_poses', poses_out='/mesh/relative_poses',
                        states_in='/swarm/states', states_out='/mesh/states',
                        commands_in='/swarm/commands', commands_out='/mesh/commands')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        self.channels, self.subs = [], []
        self.previous_time = None
        for msg_type, name in [(RelativePose, 'poses'), (SwarmState, 'states'), (SwarmCommand, 'commands')]:
            channel = DelayChannel(p['delay'], p['jitter'], p['loss'], p['capacity'], p['seed'])
            pub = self.create_publisher(msg_type, p[name + '_out'], p['qos_depth'])
            self.channels.append((channel, pub))
            self.subs.append(self.create_subscription(
                msg_type, p[name + '_in'],
                lambda msg, c=channel: c.send(msg, self.now()), p['qos_depth']))
        self.timer = self.create_timer(p['period'], self.flush)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def flush(self):
        now = self.now()
        if self.previous_time is not None and now < self.previous_time:
            for channel, _ in self.channels:
                channel.queue.clear()
        self.previous_time = now
        for channel, pub in self.channels:
            for msg in channel.receive(now):
                pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MeshNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

