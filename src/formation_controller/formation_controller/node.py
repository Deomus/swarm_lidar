from itertools import combinations
import json
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import OccupancyGrid
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import FormationCommand, RelativePose, SwarmCommand, SwarmState
from .core import (FormationPlan, RelativeVelocityController, formation_offsets, heading_rate,
                   slot_feedforward, wrap_angle)
from .obstacles import check_transition_map
from .prediction import LeaderVelocityPredictor
from lidar_tracker.timing import OrientationHistory

OFFBOARD = '14'  # VehicleStatus.nav_state PX4, строкой в SwarmState.mode


class FormationController(Node):
    def __init__(self):
        super().__init__('formation_controller')
        defaults = dict(period=.05, kp=.3, stale_timeout=.5, min_confidence=.4,
                        max_speed=2., max_acceleration=4., max_jerk=8.,
                        min_separation=2., warning_distance=3., repulsion_gain=.5,
                        gap=4., surface_bias=0., sensor_offset=[0., 0., 1.9],
                        marker_offset_z=0., poses_topic='/mesh/relative_poses',
                        leader_prediction_horizon=1.9, leader_prediction_max_correction=.6,
                        leader_prediction_max_acceleration=1., leader_prediction_smoothing=.3,
                        predict_slot_motion=False, leader_status_topic='/mission/execution_status',
                        states_topic='/mesh/states', commands_topic='/swarm/commands',
                        formation_topic='/swarm/formation_command',
                        status_topic='/swarm/formation_status', target_topic='/swarm/target_offsets')
        defaults.update(require_transition_map=False, transition_map_margin=.75)
        defaults.update(vertical_kp=1., vertical_kd=1.2, vertical_derivative_time=.2)
        defaults.update(velocity_smoothing='exponential')
        defaults.update(match_leader_yaw=False, yaw_gain=1., max_yaw_rate=.3)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        if (not math.isfinite(self.p['yaw_gain']) or self.p['yaw_gain'] < 0
                or not math.isfinite(self.p['max_yaw_rate']) or self.p['max_yaw_rate'] <= 0):
            raise ValueError('Некорректные параметры удержания курса ведомых')
        self.plan = FormationPlan(self.p['min_separation'] + .2, self.p['gap'],
                                  max_slot_speed=self.p['max_speed'] * .75)
        self.controllers = self.new_controllers()
        self.predictors = [LeaderVelocityPredictor(
            horizon=self.p['leader_prediction_horizon'],
            max_correction=self.p['leader_prediction_max_correction'],
            max_acceleration=self.p['leader_prediction_max_acceleration'],
            smoothing_time=self.p['leader_prediction_smoothing'],
            stale_timeout=self.p['stale_timeout']) for _ in range(3)]
        self.poses, self.states = {}, {}
        self.enabled = False
        self.formation, self.scale = 'line', 1.
        self.previous_time = None
        self.last_status = None
        self.leader_intent = None
        self.map = None
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.orientation_history = OrientationHistory()
        self.last_world = {}
        self.reason = 'DISABLED'
        self.request_error = None
        self.request_stamp = -1.
        self.command_pub = self.create_publisher(SwarmCommand, self.p['commands_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        self.targets_pub = self.create_publisher(PoseArray, self.p['target_topic'], 10)
        # Целевые места в map только для наблюдения: управление идёт скоростью
        # по LiDAR-ошибке, абсолютные позиции ведомых здесь не используются.
        self.setpoint_pose_pubs = {i: self.create_publisher(PoseStamped, f'/uav_{i}/setpoint/pose', 10)
                                   for i in (1, 2, 3)}
        self.pose_sub = self.create_subscription(RelativePose, self.p['poses_topic'], self.on_pose, 10)
        self.state_sub = self.create_subscription(SwarmState, self.p['states_topic'], self.on_state, 10)
        self.formation_sub = self.create_subscription(FormationCommand, self.p['formation_topic'], self.on_formation, 10)
        self.leader_status_sub = self.create_subscription(String, self.p['leader_status_topic'], self.on_leader_status, 10)
        self.enable_service = self.create_service(Trigger, '/swarm/enable_formation', self.enable)
        self.hold_service = self.create_service(Trigger, '/swarm/hold', self.hold)
        self.timer = self.create_timer(self.p['period'], self.tick)

    def new_controller(self):
        return RelativeVelocityController(
            kp=self.p['kp'], stale_timeout=self.p['stale_timeout'],
            min_confidence=self.p['min_confidence'], max_speed=self.p['max_speed'],
            max_acceleration=self.p['max_acceleration'], max_jerk=self.p['max_jerk'],
            vertical_kp=self.p['vertical_kp'], vertical_kd=self.p['vertical_kd'],
            vertical_derivative_time=self.p['vertical_derivative_time'],
            velocity_smoothing=self.p['velocity_smoothing'],
            period=self.p['period'])

    def new_controllers(self):
        return [self.new_controller() for _ in range(3)]

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def on_pose(self, msg):
        old = self.poses.get(msg.target_id)
        if 0 <= self.now() - self.seconds(msg.measurement_stamp) <= self.p['stale_timeout']:
            if old is None or self.seconds(msg.measurement_stamp) >= self.seconds(old.measurement_stamp):
                self.poses[msg.target_id] = msg

    def on_state(self, msg):
        old = self.states.get(msg.uav_id)
        if 0 <= self.now() - self.seconds(msg.timestamp) <= self.p['stale_timeout']:
            if old is None or self.seconds(msg.timestamp) >= self.seconds(old.timestamp):
                self.states[msg.uav_id] = msg
                if msg.uav_id == 0 and msg.valid:
                    q = msg.pose.orientation
                    try:
                        self.orientation_history.add(self.seconds(msg.timestamp), [q.x, q.y, q.z, q.w])
                    except ValueError:
                        pass

    def on_leader_status(self, msg):
        try:
            value = json.loads(msg.data)
            if (isinstance(value['valid'], bool) and isinstance(value['status'], str)
                    and math.isfinite(value['stamp'])
                    and 0 <= self.now()-value['stamp'] <= self.p['stale_timeout']):
                self.leader_intent = value
        except (ValueError, TypeError, KeyError):
            pass

    def reset_prediction(self):
        for predictor in self.predictors:
            predictor.reset()

    def forward_velocity(self, leader, now, index=0, offset=None, offset_velocity=None):
        v = leader.velocity.linear
        velocity = np.array([v.x, v.y, v.z])
        intent = self.leader_intent
        if (intent is not None and intent['valid'] and intent['status'] in ('MANUAL', 'MISSION')
                and 0 <= now-intent['stamp'] <= self.p['stale_timeout']):
            predictor = self.predictors[index]
            slot = velocity
            if self.p['predict_slot_motion'] and offset is not None:
                slot = slot_feedforward(offset, offset_velocity, velocity,
                                        leader.yaw, leader.yaw_rate)
            predicted = predictor.update(slot, self.seconds(leader.timestamp), now)
            # Ядро ниже само добавляет вращение и перестроение: здесь только поправка.
            return velocity + (predicted - slot)
        # Шум оценки ускорения в HOLD не должен подталкивать ведомых.
        self.reset_prediction()
        return velocity

    def leader_ready(self):
        state, now = self.states.get(0), self.now()
        return (state is not None and state.valid and state.armed
                and 0 <= now - self.seconds(state.timestamp) <= self.p['stale_timeout'])

    def parked(self, i):
        """Данные ведомого годны, но его PX4 вне Offboard (Loiter после эскалации).

        Такой аппарат выбывает из строя, но это не отказ данных: мост не должен
        продолжать эскалацию до посадки, возврат — по решению оператора.
        """
        state = self.states.get(i)
        return (state is not None and state.armed and state.mode != OFFBOARD
                and self.follower_ready(i, require_offboard=False))

    def follower_ready(self, i, require_offboard=True):
        """Ведомый в контуре: свежая валидная LiDAR-поза и собственный PX4 в Offboard.

        Аппарат, ушедший из Offboard (например, в удержание PX4 по эскалации
        моста), выбывает: иначе строй считался бы готовым без него.
        """
        state, pose, now = self.states.get(i), self.poses.get(i), self.now()
        return (state is not None and state.valid and state.armed
                and (state.mode == OFFBOARD or not require_offboard)
                and 0 <= now - self.seconds(state.timestamp) <= self.p['stale_timeout']
                and pose is not None and pose.valid and pose.confidence >= self.p['min_confidence']
                and pose.frame_id == 'uav_0/lidar_link'
                and 0 <= now - self.seconds(pose.measurement_stamp) <= self.p['stale_timeout'])

    def data_ready(self):
        return self.leader_ready() and all(self.follower_ready(i) for i in (1, 2, 3))

    def enable(self, req, response):
        if not self.data_ready():
            response.message = str(dict(
                time=self.now(), states={i: dict(valid=s.valid, armed=s.armed,
                age=self.now()-self.seconds(s.timestamp)) for i, s in self.states.items()},
                poses={i: dict(valid=p.valid, confidence=p.confidence,
                age=self.now()-self.seconds(p.measurement_stamp)) for i, p in self.poses.items()}))
            return response
        self.enabled = True
        self.controllers = self.new_controllers()
        self.reset_prediction()
        response.success = True
        response.message = 'LiDAR-relative контур включён'
        return response

    def hold(self, req, response):
        self.enabled = False
        self.reason = 'HOLD'
        self.reset_prediction()
        response.success = True
        response.message = 'Контур отключён; ведомым выдаётся нулевая скорость'
        return response

    def on_formation(self, msg):
        self.request_stamp, self.request_error = self.now(), None
        try:
            if self.p['require_transition_map']:
                now = self.now()
                intent = self.leader_intent or {}
                if not 0 <= now-intent.get('stamp', -1) <= self.p['stale_timeout'] or intent.get('mode') != 'HOLD':
                    raise ValueError('Перед перестроением остановите ведущего через удержание')
                leader = self.states.get(0)
                if leader is None or not leader.valid or leader.frame_id != 'map' or not 0 <= now-self.seconds(leader.timestamp) <= self.p['stale_timeout']:
                    raise ValueError('Нет свежего положения ведущего для проверки карты')
                target = formation_offsets(msg.formation, msg.scale, self.p['gap'], self.p['min_separation']+.2)
                check_transition_map(self.map, [leader.pose.position.x, leader.pose.position.y],
                    self.plan.sample(now).offsets, target, self.p['transition_map_margin'])
            self.plan.request(msg.formation, msg.scale, msg.transition_time, self.now())
            self.formation, self.scale = msg.formation, msg.scale
            self.get_logger().info(f'Принято: {self.formation}, scale={self.scale}')
        except ValueError as error:
            self.request_error = str(error)
            self.get_logger().warning(f'Формация отклонена: {error}')

    def on_map(self, message):
        self.map = message

    def measured(self, leader, indices=(1, 2, 3)):
        heading = Rotation.from_euler('z', leader.yaw)
        relative = []
        for i in indices:
            p = self.poses[i].pose.position
            raw = np.array([p.x, p.y, p.z])
            # Модель видимой поверхности сферы, без ground truth.
            raw *= 1 + self.p['surface_bias'] / max(1e-9, np.linalg.norm(raw))
            measured_rotation = self.orientation_history.at(self.seconds(self.poses[i].measurement_stamp))
            target_enu = measured_rotation.apply(raw + self.p['sensor_offset']) - [0., 0., self.p['marker_offset_z']]
            # Короткое предсказание относительного движения только по
            # собственным скоростям PX4; абсолютные позиции не используются.
            age = max(0., self.seconds(leader.timestamp) - self.seconds(self.poses[i].measurement_stamp))
            follower_velocity = self.states[i].velocity.linear
            leader_velocity = leader.velocity.linear
            target_enu += age * np.array([follower_velocity.x-leader_velocity.x,
                follower_velocity.y-leader_velocity.y, follower_velocity.z-leader_velocity.z])
            relative.append(heading.inv().apply(target_enu))
        return np.asarray(relative)

    def lost_positions(self, leader, current, lost):
        """Последние измеренные места выбывших, пересчитанные к текущей позе ведущего.

        Место хранится в map по позиции ведущего из его PX4 на момент измерения:
        после остановки ведущего выбывший остаётся препятствием там, где был.
        """
        position = np.array([leader.pose.position.x, leader.pose.position.y, leader.pose.position.z])
        heading = Rotation.from_euler('z', leader.yaw)
        for i, relative in current.items():
            self.last_world[i] = position + heading.apply(relative)
        return {i: heading.inv().apply(self.last_world[i] - position) for i in lost if i in self.last_world}

    def publish_slot_targets(self, leader, offsets, active=(1, 2, 3)):
        """Место ведомого: позиция ведущего по его PX4 + R(yaw)·offset, курс ведущего."""
        position, yaw = leader.pose.position, leader.yaw
        quaternion = Rotation.from_euler('z', yaw).as_quat()
        for i, offset in zip((1, 2, 3), offsets):
            if i not in active:
                continue
            x, y, z = Rotation.from_euler('z', yaw).apply(offset)
            target = PoseStamped()
            target.header.frame_id = 'map'
            target.header.stamp = self.get_clock().now().to_msg()
            target.pose.position.x, target.pose.position.y, target.pose.position.z = (
                position.x + x, position.y + y, position.z + z)
            o = target.pose.orientation
            o.x, o.y, o.z, o.w = map(float, quaternion)
            self.setpoint_pose_pubs[i].publish(target)

    def tick(self):
        now = self.now()
        previous = self.previous_time
        if self.previous_time is not None and now < self.previous_time:
            self.enabled = False
            self.poses.clear()
            self.states.clear()
            self.orientation_history.clear()
            self.last_world.clear()
            self.controllers = self.new_controllers()
            self.reason = 'CLOCK_RESET'
        self.previous_time = now
        leader_ok = self.enabled and self.leader_ready()
        available = [i for i in (1, 2, 3) if leader_ok and self.follower_ready(i)]
        lost = [i for i in (1, 2, 3) if i not in available]
        # Частичная потеря — DEGRADED: остальные держат места, ведущий по
        # статусу встаёт в HOLD, перестроение замирает до возврата всех.
        state = 'DISABLED' if not self.enabled else 'DATA_HOLD' if not available else 'DEGRADED' if lost else None
        if state is not None and previous is not None and now >= previous:
            self.plan.pause(now - previous)
        sample = self.plan.sample(now)
        targets = PoseArray()
        targets.header.frame_id = 'uav_0/heading'
        targets.header.stamp = self.get_clock().now().to_msg()
        for point in sample.offsets:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, point)
            pose.orientation.w = 1.
            targets.poses.append(pose)
        self.targets_pub.publish(targets)
        minimum = None
        errors, yaw_errors = [None] * 3, [None] * 3
        velocities = [np.zeros(3)] * 3
        yaw_rates = [0.] * 3
        active = []
        if available:
            leader = self.states[0]
            try:
                current = dict(zip(available, self.measured(leader, available)))
                obstacles = self.lost_positions(leader, current, lost)
                points = {0: np.zeros(3), **current, **obstacles}
                minimum = min(float(np.linalg.norm(points[i]-points[j]))
                              for i, j in combinations(points, 2))
                if minimum < self.p['min_separation'] + .2:
                    state = 'COLLISION_STOP'
                    self.enabled = False
                    if previous is not None and now >= previous:
                        self.plan.pause(now - previous)
                else:
                    active = available
                    state = state or ('FORMATION_CHANGE' if sample.transitioning else 'FOLLOWING')
                for i in available:
                    index, enabled = i - 1, i in active
                    forward = self.forward_velocity(leader, now, index, sample.offsets[index],
                                                    sample.velocities[index])
                    offset = sample.offsets[index].copy()
                    for j, other in points.items():
                        if j == i:
                            continue
                        delta = current[i] - other
                        distance = np.linalg.norm(delta)
                        if distance < self.p['warning_distance'] and distance > 1e-6:
                            offset += self.p['repulsion_gain'] * (self.p['warning_distance']-distance) * delta / distance
                    pose = self.poses[i]
                    result = self.controllers[index].step(current[i], offset, sample.velocities[index],
                        forward, leader.yaw, leader.yaw_rate,
                        min(self.seconds(pose.measurement_stamp), self.seconds(leader.timestamp)),
                        now, enabled=enabled, confidence=pose.confidence)
                    velocities[index] = result.velocity_enu if enabled else np.zeros(3)
                    errors[index] = float(np.linalg.norm(result.error_flu))
                    # Курс ведомого — по собственному PX4, позиция по-прежнему только из LiDAR.
                    follower_yaw = self.states[i].yaw
                    yaw_errors[index] = float(wrap_angle(leader.yaw - follower_yaw))
                    if enabled and self.p['match_leader_yaw']:
                        yaw_rates[index] = heading_rate(leader.yaw, leader.yaw_rate, follower_yaw,
                                                        self.p['yaw_gain'], self.p['max_yaw_rate'])
            except (ValueError, FloatingPointError):
                active, state = [], 'INVALID_GEOMETRY'
        for i in (1, 2, 3):
            if i not in active:
                self.controllers[i - 1] = self.new_controller()
                self.predictors[i - 1].reset()
        if active:
            self.publish_slot_targets(self.states[0], sample.offsets, active)
        for i in [1, 2, 3]:
            ok = i in active
            # Выбывший при работающем строе получает DATA_HOLD: по нему мост
            # отличает потерю данных от намеренного удержания.
            reason = (state if ok or state in ('DISABLED', 'COLLISION_STOP', 'INVALID_GEOMETRY')
                      else 'PARKED' if self.parked(i) else 'DATA_HOLD')
            command = SwarmCommand(uav_id=i, frame_id='map', valid=ok, safety_state=reason)
            command.source_stamp = self.get_clock().now().to_msg()
            if ok:
                command.source_stamp = self.poses[i].measurement_stamp
            command.target_velocity.linear.x, command.target_velocity.linear.y, command.target_velocity.linear.z = map(float, velocities[i-1])
            command.target_velocity.angular.z = yaw_rates[i-1] if ok else 0.
            self.command_pub.publish(command)
        self.status_pub.publish(String(data=json.dumps(
            dict(mode=state, formation=self.formation, scale=self.scale,
                 min_distance=minimum, errors=errors, yaw_errors=yaw_errors, time=now,
                 lost=lost if self.enabled else [],
                 leader_mode=(self.leader_intent or {}).get('mode'),
                 leader_intent_stamp=(self.leader_intent or {}).get('stamp', -1),
                 request_error=self.request_error, request_stamp=self.request_stamp))))
        if (state, lost) != self.last_status:
            self.get_logger().info(f'{state}, выбыли={lost}, min_distance={minimum}, t={now:.3f}')
            self.last_status = (state, lost)

def main(args=None):
    rclpy.init(args=args)
    node = FormationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

