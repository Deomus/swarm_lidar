import math
from collections import deque
import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped, TwistStamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                         VehicleCommandAck, VehicleLocalPosition, VehicleOdometry, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from swarm_interfaces.msg import SwarmCommand, SwarmState
from .frames import enu_ned, quaternion_ned_frd_to_enu_flu, command_is_fresh, angular_state, stamp_seconds

class AutopilotBridge(Node):
    def __init__(self):
        super().__init__('autopilot_bridge')
        defaults = dict(uav_id=0, origin_enu=[0., 0., 0.], lidar_offset=[0., 0., 1.9],
                        map_frame='map', commands_topic='/mesh/commands',
                        states_topic='/swarm/states', odometry_topic='fmu/out/vehicle_odometry',
                        local_position_topic='fmu/out/vehicle_local_position_v1',
                        status_topic='fmu/out/vehicle_status_v4',
                        command_topic='fmu/in/vehicle_command',
                        ack_topic='fmu/out/vehicle_command_ack_v1',
                        offboard_topic='fmu/in/offboard_control_mode',
                        trajectory_topic='fmu/in/trajectory_setpoint',
                        period=.05, stale_timeout=.5, status_timeout=1.5, command_timeout=.5,
                        max_velocity=3., max_yaw_rate=.3, use_offboard=False)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.uid = self.p['uav_id']
        self.odom = self.status = self.local = self.command = None
        self.future_telemetry = {key: deque(maxlen=32) for key in ('odom', 'status', 'local')}
        self.ack = None
        self.stream_enabled = self.p['use_offboard']
        self.state_pub = self.create_publisher(SwarmState, self.p['states_topic'], 10)
        self.local_state_pub = self.create_publisher(SwarmState, 'autopilot/state', 10)
        # Наблюдаемая копия скорости, отправляемой PX4, в map ENU (ТЗ §6.2).
        self.setpoint_velocity_pub = self.create_publisher(TwistStamped, 'setpoint/velocity', 10)
        self.control_pub = self.create_publisher(OffboardControlMode, self.p['offboard_topic'], 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, self.p['trajectory_topic'], 10)
        self.command_pub = self.create_publisher(VehicleCommand, self.p['command_topic'], 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = f'uav_{self.uid}/base_link'
        transform.child_frame_id = f'uav_{self.uid}/lidar_link'
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = self.p['lidar_offset']
        transform.transform.rotation.w = 1.
        self.static_tf.sendTransform(transform)
        self.subs = [
            self.create_subscription(VehicleOdometry, self.p['odometry_topic'], self.on_odom, qos_profile_sensor_data),
            self.create_subscription(VehicleLocalPosition, self.p['local_position_topic'], self.on_local, qos_profile_sensor_data),
            self.create_subscription(VehicleStatus, self.p['status_topic'], self.on_status, qos_profile_sensor_data),
            self.create_subscription(VehicleCommandAck, self.p['ack_topic'], self.on_ack, qos_profile_sensor_data),
            self.create_subscription(SwarmCommand, self.p['commands_topic'], self.on_command, 10)]
        self.action_services = []
        for name in ['arm', 'takeoff', 'offboard', 'hold', 'land', 'enable_stream']:
            self.action_services.append(self.create_service(Trigger, name,
                                 lambda req, resp, action=name: self.action(action, resp)))
        self.timer = self.create_timer(self.p['period'], self.tick)
        self.last_safety = None
        self.last_time = None
        self.stream_started = None

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        self.accept_telemetry('odom', msg)

    def on_status(self, msg):
        self.accept_telemetry('status', msg)

    def on_local(self, msg):
        self.accept_telemetry('local', msg)

    def accept_telemetry(self, field, msg):
        now = self.now()
        if msg.timestamp * 1e-6 > now:
            if msg.timestamp * 1e-6 - now <= self.p['stale_timeout']:
                self.future_telemetry[field].append(msg)
            return
        previous = getattr(self, field)
        if previous is None or msg.timestamp >= previous.timestamp:
            setattr(self, field, msg)

    def advance_telemetry(self):
        now = self.now()
        for field, queue in self.future_telemetry.items():
            waiting = []
            while queue:
                msg = queue.popleft()
                if msg.timestamp * 1e-6 <= now:
                    self.accept_telemetry(field, msg)
                else:
                    waiting.append(msg)
            queue.extend(waiting)

    def on_ack(self, msg):
        self.ack = msg
        self.get_logger().info(f'PX4 ack command={msg.command}, result={msg.result}')

    def on_command(self, msg):
        if msg.uav_id != self.uid:
            return
        if not 0 <= self.now() - stamp_seconds(msg.source_stamp) <= self.p['command_timeout']:
            return
        if self.command is None or (msg.source_stamp.sec, msg.source_stamp.nanosec) >= (
                self.command.source_stamp.sec, self.command.source_stamp.nanosec):
            self.command = msg

    def telemetry_fresh(self):
        return (self.odom is not None and self.status is not None and self.local is not None and
                0 <= self.now() - self.status.timestamp * 1e-6 <= self.p['status_timeout'] and
                all(0 <= self.now() - msg.timestamp * 1e-6 <= self.p['stale_timeout']
                    for msg in [self.odom, self.local]))

    def send_command(self, command, **params):
        msg = VehicleCommand()
        msg.timestamp = int(self.now() * 1e6)
        msg.command = command
        msg.target_system = self.uid + 1
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 190
        msg.from_external = True
        for key, value in params.items():
            setattr(msg, key, float(value))
        self.command_pub.publish(msg)

    def action(self, action, response):
        if not self.telemetry_fresh():
            response.message = 'Нет свежей телеметрии PX4'
            return response
        if action == 'enable_stream':
            self.stream_enabled = True
            self.stream_started = self.now()
        elif action == 'arm':
            if not self.status.pre_flight_checks_pass:
                response.message = 'PX4 preflight check не пройден'
                return response
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.)
        elif action == 'takeoff':
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=math.nan)
        elif action == 'land':
            self.stream_enabled = False
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        elif action == 'hold':
            self.stream_enabled = False
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1., param2=4., param3=3.)
        elif action == 'offboard':
            if not self.stream_enabled or self.stream_started is None or self.now() - self.stream_started < 1.:
                response.message = 'Сначала enable_stream и минимум 1 с heartbeat'
                return response
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1., param2=6.)
        response.success = True
        response.message = 'Запрос отправлен; выполнение подтверждается PX4 ack и autopilot/state'
        return response

    def tick(self):
        if self.last_time is not None and self.now() < self.last_time:
            self.stream_enabled = False
            self.stream_started = None
            self.command = self.odom = self.status = self.local = None
            for queue in self.future_telemetry.values():
                queue.clear()
            self.get_logger().warning('Откат часов: Offboard stream отключён, требуется enable_stream')
        self.last_time = self.now()
        self.advance_telemetry()
        if self.odom is None:
            return
        now = self.get_clock().now()
        msg = SwarmState()
        msg.uav_id = self.uid
        msg.frame_id = self.p['map_frame']
        msg.timestamp = Time(sec=int(self.odom.timestamp // 1000000),
                             nanosec=int(self.odom.timestamp % 1000000 * 1000))
        msg.last_seen = now.to_msg()
        msg.valid = bool(self.telemetry_fresh() and self.local.xy_valid and self.local.z_valid and
                         self.local.v_xy_valid and self.local.v_z_valid and
                         self.odom.pose_frame == VehicleOdometry.POSE_FRAME_NED and
                         self.odom.velocity_frame == VehicleOdometry.VELOCITY_FRAME_NED)
        try:
            position = enu_ned(self.odom.position) + np.asarray(self.p['origin_enu'])
            velocity = enu_ned(self.odom.velocity)
            quaternion = quaternion_ned_frd_to_enu_flu(self.odom.q)
            if not np.isfinite(np.r_[position, velocity]).all():
                msg.valid = False
                raise ValueError('Нечисловые координаты')
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, position)
            msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quaternion)
            msg.velocity.linear.x, msg.velocity.linear.y, msg.velocity.linear.z = map(float, velocity)
            angular, msg.yaw, msg.yaw_rate = angular_state(quaternion, self.odom.angular_velocity)
            msg.velocity.angular.x, msg.velocity.angular.y, msg.velocity.angular.z = map(float, angular)
        except ValueError:
            msg.valid = False
        msg.armed = bool(self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        msg.mode = str(self.status.nav_state) if self.status else 'UNKNOWN'
        msg.health = 'OK' if msg.valid else 'STALE'
        self.state_pub.publish(msg)
        self.local_state_pub.publish(msg)
        if msg.valid:
            transform = TransformStamped()
            transform.header.stamp = msg.timestamp
            transform.header.frame_id = msg.frame_id
            transform.child_frame_id = f'uav_{self.uid}/base_link'
            transform.transform.translation.x = msg.pose.position.x
            transform.transform.translation.y = msg.pose.position.y
            transform.transform.translation.z = msg.pose.position.z
            transform.transform.rotation = msg.pose.orientation
            self.tf.sendTransform(transform)
        if self.stream_enabled:
            fresh = bool(msg.valid and self.command is not None and
                         command_is_fresh(self.command, self.now(), self.p['command_timeout'], self.p['map_frame']))
            desired = np.zeros(3)
            yaw_rate = 0.
            if fresh:
                v = self.command.target_velocity.linear
                desired = np.array([v.x, v.y, v.z])
                desired *= min(1., self.p['max_velocity'] / max(1e-9, np.linalg.norm(desired)))
                yaw_rate = float(np.clip(self.command.target_velocity.angular.z,
                                         -self.p['max_yaw_rate'], self.p['max_yaw_rate']))
            safety = 'COMMAND' if fresh else 'BRAKE'
            if safety != self.last_safety:
                self.get_logger().info(f'setpoint: {safety}')
                self.last_safety = safety
            control = OffboardControlMode(timestamp=int(self.now() * 1e6), velocity=True)
            self.control_pub.publish(control)
            sp = TrajectorySetpoint()
            sp.timestamp = control.timestamp
            sp.position = [math.nan] * 3
            sp.velocity = list(map(float, enu_ned(desired)))
            sp.acceleration = [math.nan] * 3
            sp.jerk = [math.nan] * 3
            sp.yaw = math.nan
            sp.yawspeed = -yaw_rate
            self.setpoint_pub.publish(sp)
            observed = TwistStamped()
            observed.header.stamp, observed.header.frame_id = now.to_msg(), self.p['map_frame']
            observed.twist.linear.x, observed.twist.linear.y, observed.twist.linear.z = map(float, desired)
            observed.twist.angular.z = yaw_rate
            self.setpoint_velocity_pub.publish(observed)

def main(args=None):
    rclpy.init(args=args)
    node = AutopilotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

