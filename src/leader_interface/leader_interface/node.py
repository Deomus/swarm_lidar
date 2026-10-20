"""ROS-интерфейс команд лидеру; arm/takeoff/Offboard выполняет внешний сценарий."""

import json
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import SwarmCommand, SwarmState

from .core import LeaderCore
from .formation_guard import FormationGuard


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class LeaderInterface(Node):
    def __init__(self):
        super().__init__('leader_interface')
        defaults = dict(states_topic='/mesh/states', manual_topic='/leader/cmd_vel',
                        path_topic='/mission/path', commands_topic='/swarm/commands',
                        status_topic='/mission/execution_status', frame_id='map',
                        period=.05, max_speed=1.5, max_acceleration=1., max_jerk=2.,
                        max_yaw_rate=.3,
                        manual_timeout=.25, state_timeout=.5, waypoint_tolerance=.4,
                        waypoint_gain=.25, waypoint_timeout=90.,
                        require_formation_ready=False,
                        formation_status_topic='/swarm/formation_status',
                        formation_status_timeout=.5, formation_unavailable_grace=1.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.core = LeaderCore(**{key: self.p[key] for key in (
            'period', 'max_speed', 'max_acceleration', 'max_jerk', 'manual_timeout',
            'state_timeout', 'waypoint_tolerance', 'waypoint_gain', 'waypoint_timeout',
            'max_yaw_rate')})
        self.formation_guard = FormationGuard(
            required=self.p['require_formation_ready'],
            status_timeout=self.p['formation_status_timeout'],
            grace=self.p['formation_unavailable_grace'])
        self.command_pub = self.create_publisher(SwarmCommand, self.p['commands_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        self.subscriptions_ = [
            self.create_subscription(SwarmState, self.p['states_topic'], self.state, 10),
            self.create_subscription(TwistStamped, self.p['manual_topic'], self.manual, 10),
            self.create_subscription(Path, self.p['path_topic'], self.path, 10),
        ]
        if self.p['require_formation_ready']:
            self.subscriptions_.append(self.create_subscription(
                String, self.p['formation_status_topic'], self.formation_status, 10))
        self.services_ = [self.create_service(
            Trigger, f'/leader/{mode.lower()}',
            lambda req, response, mode=mode: self.select(mode, response))
            for mode in ('HOLD', 'MANUAL', 'MISSION')]
        self.timer = self.create_timer(self.p['period'], self.tick)
        self.last_status = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def state(self, message):
        if message.uav_id != 0:
            return
        if message.frame_id != self.p['frame_id']:
            self.core.state_valid = False
            return
        p = message.pose.position
        valid = message.valid and math.isfinite(message.yaw) and math.isfinite(message.yaw_rate)
        self.core.state([p.x, p.y, p.z], seconds(message.timestamp), valid, self.now())

    def manual(self, message):
        if message.header.frame_id != self.p['frame_id']:
            self.get_logger().warning('Ручная команда отклонена: требуется frame map ENU')
            return
        angular = message.twist.angular
        if any(not math.isfinite(v) or abs(v) > 1e-9 for v in (angular.x, angular.y)) or not math.isfinite(angular.z):
            self.get_logger().warning('Разрешена только конечная угловая скорость yaw angular.z')
            return
        v = message.twist.linear
        self.core.manual([v.x, v.y, v.z], seconds(message.header.stamp), self.now(), angular.z)

    def path(self, message):
        if message.header.frame_id != self.p['frame_id']:
            self.get_logger().warning('Маршрут отклонён: требуется frame map ENU')
            return
        stamp = seconds(message.header.stamp)
        if not math.isfinite(stamp) or stamp > self.now():
            self.get_logger().warning('Маршрут отклонён: timestamp из будущего')
            return
        if not message.poses:
            self.core.revoke_path()
            self.get_logger().warning('Маршрут отозван планировщиком')
            return
        if any(pose.header.frame_id not in ('', self.p['frame_id']) for pose in message.poses):
            self.get_logger().warning('Маршрут отклонён: frame waypoint не совпадает')
            return
        try:
            self.core.set_path([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                for p in message.poses])
        except ValueError as error:
            self.get_logger().warning(f'Маршрут отклонён: {error}')

    def formation_status(self, message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            return
        self.formation_guard.update(value, self.now())

    def select(self, mode, response):
        try:
            now = self.now()
            if mode in ('MANUAL', 'MISSION'):
                self.formation_guard.require_ready(now)
            self.core.select_mode(mode, now)
        except ValueError as error:
            response.message = str(error)
            return response
        response.success = True
        response.message = f'Режим лидера: {mode}; arm/Offboard управляются отдельно'
        return response

    def tick(self):
        now = self.now()
        result = self.core.tick(now)
        if self.formation_guard.should_hold(self.core.mode in ('MANUAL', 'MISSION'), now):
            self.core.select_mode('HOLD', now)
            self.core.status = 'FORMATION_UNAVAILABLE'
            result = self.core.tick(now)
        message = SwarmCommand()
        message.uav_id = 0
        message.frame_id = self.p['frame_id']
        message.valid = result.valid
        message.safety_state = result.status
        stamp_ns = max(0, round(result.source_stamp * 1e9))
        message.source_stamp.sec, message.source_stamp.nanosec = divmod(stamp_ns, 10**9)
        v = result.velocity
        message.target_velocity.linear.x = float(v[0])
        message.target_velocity.linear.y = float(v[1])
        message.target_velocity.linear.z = float(v[2])
        message.target_velocity.angular.z = result.yaw_rate
        self.command_pub.publish(message)
        status = dict(mode=self.core.mode, status=result.status, waypoint=result.waypoint_index,
                      total=len(self.core.path) if self.core.path is not None else 0,
                      valid=result.valid, stamp=now,
                      formation_ready=self.formation_guard.ready(now),
                      formation_required=self.p['require_formation_ready'])
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))
        if result.status != self.last_status:
            self.get_logger().info(f'Лидер: {result.status}, waypoint={result.waypoint_index}')
            self.last_status = result.status


def main(args=None):
    rclpy.init(args=args)
    node = LeaderInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
