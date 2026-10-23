"""Контроллер строя на ведомом: своя команда по LiDAR-позе из mesh.

Каждый ведомый uav_1..3 запускает свой экземпляр. По mesh он получает
относительные позы от LiDAR ведущего, состояния аппаратов, команду ведущего
(упреждение) и план строя от координатора. Свою скорость он отдаёт собственному
мосту PX4 локально, а статус — координатору через mesh.
"""

from itertools import combinations
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from swarm_interfaces.msg import FormationPlan, RelativePose, SwarmCommand, SwarmState

from lidar_tracker.timing import OrientationHistory
from .core import RelativeVelocityController, heading_rate, sample_transition, wrap_angle

OFFBOARD = '14'  # VehicleStatus.nav_state PX4, строкой в SwarmState.mode
FOLLOWERS = (1, 2, 3)


class FormationController(Node):
    def __init__(self, **kwargs):
        super().__init__('formation_controller', **kwargs)
        defaults = dict(uav_id=1, period=.05, kp=.3, stale_timeout=.5, min_confidence=.4,
                        max_speed=2., max_acceleration=4., max_jerk=8.,
                        min_separation=2., warning_distance=3., repulsion_gain=.5,
                        surface_bias=0., sensor_offset=[0., 0., 1.9], marker_offset_z=0.,
                        vertical_kp=1., vertical_kd=1.2, vertical_derivative_time=.2,
                        velocity_smoothing='exponential',
                        match_leader_yaw=False, yaw_gain=1., max_yaw_rate=.3,
                        poses_topic='/mesh/relative_poses', states_topic='/mesh/states',
                        leader_commands_topic='/mesh/commands', plan_topic='/mesh/formation_plan',
                        command_topic='command', status_topic='/swarm/follower_status')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.uid = self.p['uav_id']
        if self.uid not in FOLLOWERS:
            raise ValueError('uav_id ведомого: 1, 2 или 3')
        if (not math.isfinite(self.p['yaw_gain']) or self.p['yaw_gain'] < 0
                or not math.isfinite(self.p['max_yaw_rate']) or self.p['max_yaw_rate'] <= 0):
            raise ValueError('Некорректные параметры удержания курса ведомых')
        self.controller = self.new_controller()
        self.poses, self.states = {}, {}
        self.plan = None
        self.epoch = None
        self.collision = False
        self.previous_time = None
        self.last_status = None
        self.orientation_history = OrientationHistory()
        self.last_world = {}
        self.leader_command = None
        self.command_pub = self.create_publisher(SwarmCommand, self.p['command_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        # Целевое место в map только для наблюдения: управление идёт скоростью
        # по LiDAR-ошибке, абсолютная позиция ведомого здесь не используется.
        self.setpoint_pose_pub = self.create_publisher(PoseStamped, 'setpoint/pose', 10)
        self.create_subscription(RelativePose, self.p['poses_topic'], self.on_pose, 10)
        self.create_subscription(SwarmState, self.p['states_topic'], self.on_state, 10)
        self.create_subscription(SwarmCommand, self.p['leader_commands_topic'], self.on_command, 10)
        self.create_subscription(FormationPlan, self.p['plan_topic'], self.on_plan, 10)
        self.create_timer(self.p['period'], self.tick)

    def new_controller(self):
        return RelativeVelocityController(
            kp=self.p['kp'], stale_timeout=self.p['stale_timeout'],
            min_confidence=self.p['min_confidence'], max_speed=self.p['max_speed'],
            max_acceleration=self.p['max_acceleration'], max_jerk=self.p['max_jerk'],
            vertical_kp=self.p['vertical_kp'], vertical_kd=self.p['vertical_kd'],
            vertical_derivative_time=self.p['vertical_derivative_time'],
            velocity_smoothing=self.p['velocity_smoothing'],
            period=self.p['period'])

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

    def on_command(self, msg):
        if msg.uav_id == 0:
            self.leader_command = msg

    def on_plan(self, msg):
        if (len(msg.start_offsets) == len(msg.end_offsets) == 3 and math.isfinite(msg.start_time)
                and math.isfinite(msg.duration) and msg.duration > 0
                and (self.plan is None or self.seconds(msg.stamp) >= self.seconds(self.plan.stamp))):
            self.plan = msg

    def current_plan(self, now):
        """Свежий план координатора; без него ведомый не знает своего места."""
        plan = self.plan
        if plan is None or not 0 <= now - self.seconds(plan.stamp) <= self.p['stale_timeout']:
            return None
        return plan

    def leader_motion(self, leader, now):
        """Упреждение: скорость (ENU) и yaw rate ведущего.

        Источник command — заданная ведущему команда из mesh: она уже
        ограничена по ускорению и рывку и опережает измеренную скорость на
        запаздывание PX4, одинаковое у всех аппаратов. Недействительная
        свежая команда означает торможение ведущего до нуля. Без свежей
        команды (потеря в mesh) — измеренная скорость ведущего.
        """
        command = self.leader_command
        if (command is not None and command.frame_id == 'map'
                and 0 <= now - self.seconds(command.source_stamp) <= self.p['stale_timeout']):
            if not command.valid:
                return np.zeros(3), 0.
            v = command.target_velocity
            return np.array([v.linear.x, v.linear.y, v.linear.z]), v.angular.z
        v = leader.velocity.linear
        return np.array([v.x, v.y, v.z]), leader.yaw_rate

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

    def measured(self, leader, indices=FOLLOWERS):
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

    def publish_slot_target(self, leader, offset):
        """Место ведомого: позиция ведущего по его PX4 + R(yaw)·offset, курс ведущего."""
        position, yaw = leader.pose.position, leader.yaw
        rotation = Rotation.from_euler('z', yaw)
        x, y, z = rotation.apply(offset)
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x, target.pose.position.y, target.pose.position.z = (
            position.x + x, position.y + y, position.z + z)
        o = target.pose.orientation
        o.x, o.y, o.z, o.w = map(float, rotation.as_quat())
        self.setpoint_pose_pub.publish(target)

    def tick(self):
        now = self.now()
        previous = self.previous_time
        if previous is not None and now < previous:
            self.poses.clear()
            self.states.clear()
            self.orientation_history.clear()
            self.last_world.clear()
            self.plan, self.epoch, self.collision = None, None, False
            self.controller = self.new_controller()
        self.previous_time = now
        i, index = self.uid, self.uid - 1
        plan = self.current_plan(now)
        if plan is not None and plan.epoch != self.epoch:
            # Новое включение строя: сброс регулятора и прошлой остановки.
            self.epoch, self.collision = plan.epoch, False
            self.controller = self.new_controller()
        ready = self.follower_ready(i)
        velocity, yaw_rate = np.zeros(3), 0.
        error = yaw_error = minimum = None
        if plan is None:
            state = 'DATA_HOLD'  # нет связи с координатором: места неизвестны
        elif not plan.enabled or self.collision:
            # Остановка по сближению держится до нового включения строя.
            state = 'COLLISION_STOP' if self.collision else 'DISABLED'
        elif not self.leader_ready() or not ready:
            state = 'PARKED' if self.parked(i) else 'DATA_HOLD'
        else:
            leader = self.states[0]
            others = [j for j in FOLLOWERS if j != i and self.follower_ready(j)]
            lost = [j for j in FOLLOWERS if j != i and j not in others]
            try:
                sample = sample_transition([[p.x, p.y, p.z] for p in plan.start_offsets],
                                           [[p.x, p.y, p.z] for p in plan.end_offsets],
                                           plan.start_time, plan.duration, now)
                indices = [i, *others]
                current = dict(zip(indices, self.measured(leader, indices)))
                obstacles = self.lost_positions(leader, current, lost)
                points = {0: np.zeros(3), **current, **obstacles}
                # Все пары, не только свои: при опасном сближении любых двух
                # аппаратов каждый ведомый останавливается сам, не дожидаясь
                # координатора.
                minimum = min(float(np.linalg.norm(points[a]-points[b])) for a, b in combinations(points, 2))
                if minimum < self.p['min_separation'] + .2:
                    state, self.collision = 'COLLISION_STOP', True
                else:
                    state = 'FORMATION_CHANGE' if sample.transitioning else 'FOLLOWING'
                    forward, leader_yaw_rate = self.leader_motion(leader, now)
                    offset = sample.offsets[index].copy()
                    for j, other in points.items():
                        if j == i:
                            continue
                        delta = current[i] - other
                        distance = np.linalg.norm(delta)
                        if distance < self.p['warning_distance'] and distance > 1e-6:
                            offset += self.p['repulsion_gain'] * (self.p['warning_distance']-distance) * delta / distance
                    pose = self.poses[i]
                    result = self.controller.step(current[i], offset, sample.velocities[index],
                        forward, leader.yaw, leader_yaw_rate,
                        min(self.seconds(pose.measurement_stamp), self.seconds(leader.timestamp)),
                        now, enabled=True, confidence=pose.confidence)
                    velocity = result.velocity_enu
                    error = float(np.linalg.norm(result.error_flu))
                    # Курс ведомого — по собственному PX4, позиция только из LiDAR.
                    yaw_error = float(wrap_angle(leader.yaw - self.states[i].yaw))
                    if self.p['match_leader_yaw']:
                        yaw_rate = heading_rate(leader.yaw, leader_yaw_rate, self.states[i].yaw,
                                                self.p['yaw_gain'], self.p['max_yaw_rate'])
                    self.publish_slot_target(leader, sample.offsets[index])
            except (ValueError, FloatingPointError):
                state = 'INVALID_GEOMETRY'
        ok = state in ('FOLLOWING', 'FORMATION_CHANGE')
        if not ok:
            self.controller = self.new_controller()
            velocity, yaw_rate = np.zeros(3), 0.
        command = SwarmCommand(uav_id=i, frame_id='map', valid=ok, safety_state=state)
        command.source_stamp = self.poses[i].measurement_stamp if ok else self.get_clock().now().to_msg()
        command.target_velocity.linear.x, command.target_velocity.linear.y, command.target_velocity.linear.z = map(float, velocity)
        command.target_velocity.angular.z = float(yaw_rate)
        self.command_pub.publish(command)
        self.status_pub.publish(String(data=json.dumps(dict(
            uav_id=i, time=now, epoch=self.epoch, mode=state, ready=ready, parked=self.parked(i),
            error=error, yaw_error=yaw_error, min_distance=minimum))))
        if state != self.last_status:
            self.get_logger().info(f'uav_{i}: {state}, min_distance={minimum}, t={now:.3f}')
            self.last_status = state


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
