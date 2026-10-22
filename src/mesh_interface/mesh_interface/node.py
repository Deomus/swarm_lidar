import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Trigger
from swarm_interfaces.msg import RelativePose, SwarmState, SwarmCommand
from .channel import DelayChannel

class MeshNode(Node):
    def __init__(self):
        super().__init__('mesh_interface')
        defaults = dict(delay=0., jitter=0., loss=0., capacity=1000, seed=42,
                        period=0.01, qos_depth=10, drop_duration=.7,
                        poses_in='/swarm/relative_poses', poses_out='/mesh/relative_poses',
                        states_in='/swarm/states', states_out='/mesh/states',
                        commands_in='/swarm/commands', commands_out='/mesh/commands')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        if not math.isfinite(p['drop_duration']) or p['drop_duration'] <= 0:
            raise ValueError('Некорректная длительность потери')
        self.drop_duration = p['drop_duration']
        self.channels, self.subs = [], []
        self.previous_time = None
        for index, (msg_type, name) in enumerate(
                [(RelativePose, 'poses'), (SwarmState, 'states'), (SwarmCommand, 'commands')]):
            # Свой seed канала: потери поз, состояний и команд не совпадают по номеру пакета.
            channel = DelayChannel(p['delay'], p['jitter'], p['loss'], p['capacity'], p['seed'] + index)
            pub = self.create_publisher(msg_type, p[name + '_out'], p['qos_depth'])
            self.channels.append((channel, pub))
            self.subs.append(self.create_subscription(
                msg_type, p[name + '_in'],
                lambda msg, c=channel: c.send(msg, self.now()), p['qos_depth']))
        self.drop_service = self.create_service(Trigger, '/mesh/drop_relative_poses', self.drop_poses)
        self.timer = self.create_timer(p['period'], self.flush)

    def drop_poses(self, request, response):
        # Параметр можно временно изменить для отдельного испытания. Значение
        # читается при запросе, а не из снимка, сделанного при запуске узла.
        duration = self.get_parameter('drop_duration').value
        response.success = False
        if (isinstance(duration, bool) or not isinstance(duration, (float, int))
                or not math.isfinite(duration) or duration <= 0):
            response.message = 'Некорректная длительность потери'
            return response
        try:
            self.channels[0][0].drop_for(self.now(), duration)
        except ValueError as error:
            response.message = str(error)
            return response
        response.success = True
        response.message = f'Передача LiDAR-поз приостановлена на {duration} с'
        return response

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

