import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import FormationPlan, RelativePose, SwarmState, SwarmCommand
from .channel import DelayChannel

class MeshNode(Node):
    def __init__(self):
        super().__init__('mesh_interface')
        defaults = dict(delay=0., jitter=0., loss=0., capacity=1000, seed=42,
                        period=0.01, qos_depth=10, drop_duration=.7, drop_target_id=0,
                        poses_in='/swarm/relative_poses', poses_out='/mesh/relative_poses',
                        states_in='/swarm/states', states_out='/mesh/states',
                        commands_in='/swarm/commands', commands_out='/mesh/commands',
                        plan_in='/swarm/formation_plan', plan_out='/mesh/formation_plan',
                        follower_status_in='/swarm/follower_status',
                        follower_status_out='/mesh/follower_status')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        if not math.isfinite(p['drop_duration']) or p['drop_duration'] <= 0:
            raise ValueError('Некорректная длительность потери')
        self.channels, self.subs = [], []
        self.previous_time = None
        self.target_drop = None
        for index, (msg_type, name) in enumerate(
                [(RelativePose, 'poses'), (SwarmState, 'states'), (SwarmCommand, 'commands'),
                 (FormationPlan, 'plan'), (String, 'follower_status')]):
            # Свой seed канала: потери поз, состояний и команд не совпадают по номеру пакета.
            channel = DelayChannel(p['delay'], p['jitter'], p['loss'], p['capacity'], p['seed'] + index)
            pub = self.create_publisher(msg_type, p[name + '_out'], p['qos_depth'])
            self.channels.append((channel, pub))
            receive = self.on_pose if name == 'poses' else (lambda msg, c=channel: c.send(msg, self.now()))
            self.subs.append(self.create_subscription(msg_type, p[name + '_in'], receive, p['qos_depth']))
        self.drop_service = self.create_service(Trigger, '/mesh/drop_relative_poses', self.drop_poses)
        self.timer = self.create_timer(p['period'], self.flush)

    def on_pose(self, msg):
        channel, now = self.channels[0][0], self.now()
        if self.target_drop is not None and msg.target_id == self.target_drop[0] and now < self.target_drop[1]:
            channel.dropped += 1
            return
        channel.send(msg, now)

    def drop_poses(self, request, response):
        # Параметры можно временно изменить для отдельного испытания. Значения
        # читаются при запросе, а не из снимка, сделанного при запуске узла.
        # drop_target_id=0 — все цели, 1..3 — только поза этого ведомого.
        duration = self.get_parameter('drop_duration').value
        target = self.get_parameter('drop_target_id').value
        response.success = False
        if (isinstance(duration, bool) or not isinstance(duration, (float, int))
                or not math.isfinite(duration) or duration <= 0):
            response.message = 'Некорректная длительность потери'
            return response
        if isinstance(target, bool) or target not in (0, 1, 2, 3):
            response.message = 'drop_target_id: 0 — все цели, 1..3 — один ведомый'
            return response
        try:
            if target:
                self.channels[0][0].observe_time(self.now())
                self.target_drop = (target, self.now() + duration)
            else:
                self.channels[0][0].drop_for(self.now(), duration)
        except ValueError as error:
            response.message = str(error)
            return response
        response.success = True
        response.message = (f'Передача LiDAR-позы uav_{target} приостановлена на {duration} с' if target
                            else f'Передача LiDAR-поз приостановлена на {duration} с')
        return response

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def flush(self):
        now = self.now()
        if self.previous_time is not None and now < self.previous_time:
            self.target_drop = None
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

