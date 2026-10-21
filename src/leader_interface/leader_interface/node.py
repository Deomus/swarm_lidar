"""ROS-интерфейс команд лидеру; arm/takeoff/Offboard выполняет внешний сценарий."""

import json
import math
import numpy as np

import rclpy
from geometry_msgs.msg import TwistStamped, PoseArray
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import SwarmCommand, SwarmState

from .core import LeaderCore
from .formation_guard import FormationGuard
from formation_controller.core import limit_group_motion


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
                        waypoint_gain=.6, waypoint_timeout=90., arrival_speed=.15,
                        require_formation_ready=False,
                        formation_status_topic='/swarm/formation_status',
                        formation_status_timeout=.5, formation_unavailable_grace=1.)
        defaults.update(require_slot_geometry=False, follower_max_speed=2., follower_speed_reserve=.15)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if (not math.isfinite(self.p['follower_max_speed']) or self.p['follower_max_speed'] <= 0
                or not 0 <= self.p['follower_speed_reserve'] < 1):
            raise ValueError('Некорректные ограничения скорости ведомых')
        self.core = LeaderCore(**{key: self.p[key] for key in (
            'period', 'max_speed', 'max_acceleration', 'max_jerk', 'manual_timeout',
            'state_timeout', 'waypoint_tolerance', 'waypoint_gain', 'waypoint_timeout',
            'max_yaw_rate', 'arrival_speed')})
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
        self.accepted_path_stamp = None
        self.slot_geometry = None
        self.heading = 0.
        self.create_subscription(PoseArray, '/swarm/target_offsets', self.offsets, 10)

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
        if valid:
            self.heading = message.yaw
        v = message.velocity.linear
        self.core.state([p.x, p.y, p.z], seconds(message.timestamp), valid, self.now(),
                        velocity=[v.x, v.y, v.z])

    def offsets(self, message):
        if message.header.frame_id != 'uav_0/heading' or len(message.poses) != 3:
            return
        stamp = seconds(message.header.stamp)
        points = np.array([[p.position.x, p.position.y, p.position.z] for p in message.poses])
        if not np.isfinite(points).all() or not 0 <= self.now()-stamp <= self.p['state_timeout']:
            return
        velocity = np.zeros((3, 3))
        old = self.slot_geometry
        if old is not None:
            if stamp <= old[0]:
                return
            if stamp-old[0] <= self.p['state_timeout']:
                velocity = (points-old[1]) / (stamp-old[0])
        self.slot_geometry = stamp, points, velocity

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
            self.accepted_path_stamp = None
            self.core.revoke_path()
            self.get_logger().warning('Маршрут отозван планировщиком')
            return
        if any(pose.header.frame_id not in ('', self.p['frame_id']) for pose in message.poses):
            self.get_logger().warning('Маршрут отклонён: frame waypoint не совпадает')
            return
        try:
            self.core.set_path([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                                for p in message.poses])
            self.accepted_path_stamp = [message.header.stamp.sec, message.header.stamp.nanosec]
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
        v, yaw_rate, motion_scale = result.velocity, result.yaw_rate, 1.
        if result.valid and self.p['require_slot_geometry']:
            try:
                if self.slot_geometry is None or not 0 <= now-self.slot_geometry[0] <= self.p['state_timeout']:
                    raise ValueError('Нет свежей геометрии строя')
                v, yaw_rate, motion_scale = limit_group_motion(v, yaw_rate,
                    self.slot_geometry[1], self.slot_geometry[2], self.heading,
                    self.p['follower_max_speed'] * (1.-self.p['follower_speed_reserve']))
            except ValueError:
                self.core.select_mode('HOLD', now)
                self.core.status = 'GROUP_MOTION_UNAVAILABLE'
                result = self.core.tick(now)
                v, yaw_rate, motion_scale = result.velocity, result.yaw_rate, 0.
        message = SwarmCommand()
        message.uav_id = 0
        message.frame_id = self.p['frame_id']
        message.valid = result.valid
        message.safety_state = result.status
        stamp_ns = max(0, round(result.source_stamp * 1e9))
        message.source_stamp.sec, message.source_stamp.nanosec = divmod(stamp_ns, 10**9)
        message.target_velocity.linear.x = float(v[0])
        message.target_velocity.linear.y = float(v[1])
        message.target_velocity.linear.z = float(v[2])
        message.target_velocity.angular.z = yaw_rate
        self.command_pub.publish(message)
        status = dict(mode=self.core.mode, status=result.status, waypoint=result.waypoint_index,
                      total=len(self.core.path) if self.core.path is not None else 0,
                      valid=result.valid, stamp=now,
                      accepted_path_stamp=self.accepted_path_stamp,
                      group_motion_scale=motion_scale,
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
