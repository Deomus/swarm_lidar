import json
import math
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import FormationCommand, RelativePose, SwarmCommand, SwarmState
from .core import FormationPlan, RelativeVelocityController, slot_feedforward
from .prediction import LeaderVelocityPredictor
from lidar_tracker.timing import OrientationHistory

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
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
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
        self.orientation_history = OrientationHistory()
        self.reason = 'DISABLED'
        self.command_pub = self.create_publisher(SwarmCommand, self.p['commands_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        self.targets_pub = self.create_publisher(PoseArray, self.p['target_topic'], 10)
        self.pose_sub = self.create_subscription(RelativePose, self.p['poses_topic'], self.on_pose, 10)
        self.state_sub = self.create_subscription(SwarmState, self.p['states_topic'], self.on_state, 10)
        self.formation_sub = self.create_subscription(FormationCommand, self.p['formation_topic'], self.on_formation, 10)
        self.leader_status_sub = self.create_subscription(String, self.p['leader_status_topic'], self.on_leader_status, 10)
        self.enable_service = self.create_service(Trigger, '/swarm/enable_formation', self.enable)
        self.hold_service = self.create_service(Trigger, '/swarm/hold', self.hold)
        self.timer = self.create_timer(self.p['period'], self.tick)

    def new_controllers(self):
        return [RelativeVelocityController(
            kp=self.p['kp'], stale_timeout=self.p['stale_timeout'],
            min_confidence=self.p['min_confidence'], max_speed=self.p['max_speed'],
            max_acceleration=self.p['max_acceleration'], max_jerk=self.p['max_jerk'],
            period=self.p['period']) for _ in range(3)]

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

    def data_ready(self):
        now = self.now()
        return (all(i in self.states and self.states[i].valid and self.states[i].armed and
                    0 <= now - self.seconds(self.states[i].timestamp) <= self.p['stale_timeout']
                    for i in range(4)) and
                all(i in self.poses and self.poses[i].valid and
                    self.poses[i].confidence >= self.p['min_confidence'] and
                    self.poses[i].frame_id == 'uav_0/lidar_link' and
                    0 <= now - self.seconds(self.poses[i].measurement_stamp) <= self.p['stale_timeout']
                    for i in [1, 2, 3]))

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
        try:
            self.plan.request(msg.formation, msg.scale, msg.transition_time, self.now())
            self.formation, self.scale = msg.formation, msg.scale
            self.get_logger().info(f'Принято: {self.formation}, scale={self.scale}')
        except ValueError as error:
            self.get_logger().warning(f'Формация отклонена: {error}')

    def measured(self, leader):
        q = leader.pose.orientation
        rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])
        heading = Rotation.from_euler('z', leader.yaw)
        relative = []
        for i in [1, 2, 3]:
            p = self.poses[i].pose.position
            raw = np.array([p.x, p.y, p.z])
            # Модель видимой поверхности сферы, без ground truth.
            raw *= 1 + self.p['surface_bias'] / max(1e-9, np.linalg.norm(raw))
            measured_rotation = (self.orientation_history.at(self.seconds(self.poses[i].measurement_stamp))
                                 if hasattr(self, 'orientation_history') else rotation)
            target_enu = measured_rotation.apply(raw + self.p['sensor_offset']) - [0., 0., self.p['marker_offset_z']]
            if hasattr(self, 'orientation_history'):
                # Короткое предсказание относительного движения только по
                # собственным скоростям PX4; абсолютные позиции не используются.
                age = max(0., self.seconds(leader.timestamp) - self.seconds(self.poses[i].measurement_stamp))
                follower_velocity = self.states[i].velocity.linear
                leader_velocity = leader.velocity.linear
                target_enu += age * np.array([follower_velocity.x-leader_velocity.x,
                    follower_velocity.y-leader_velocity.y, follower_velocity.z-leader_velocity.z])
            relative.append(heading.inv().apply(target_enu))
        return np.asarray(relative)

    def tick(self):
        now = self.now()
        previous = self.previous_time
        if self.previous_time is not None and now < self.previous_time:
            self.enabled = False
            self.poses.clear()
            self.states.clear()
            self.orientation_history.clear()
            self.controllers = self.new_controllers()
            self.reason = 'CLOCK_RESET'
        self.previous_time = now
        ready = self.enabled and self.data_ready()
        if not ready and previous is not None and now >= previous:
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
        errors = []
        velocities = [np.zeros(3)] * 3
        state = 'DISABLED' if not self.enabled else 'DATA_HOLD'
        if ready:
            leader = self.states[0]
            try:
                relative = self.measured(leader)
                all_points = np.vstack([np.zeros(3), relative])
                minimum = min(float(np.linalg.norm(all_points[i]-all_points[j]))
                              for i in range(4) for j in range(i))
                ready = minimum >= self.p['min_separation'] + .2
                state = 'FORMATION_CHANGE' if sample.transitioning else 'FOLLOWING'
                if not ready:
                    state = 'COLLISION_STOP'
                    self.enabled = False
                    if previous is not None and now >= previous:
                        self.plan.pause(now - previous)
                for index, controller in enumerate(self.controllers):
                    forward = self.forward_velocity(leader, now, index, sample.offsets[index],
                                                    sample.velocities[index])
                    offset = sample.offsets[index].copy()
                    for j, other in enumerate(all_points):
                        if j == index + 1:
                            continue
                        delta = relative[index] - other
                        distance = np.linalg.norm(delta)
                        if distance < self.p['warning_distance'] and distance > 1e-6:
                            offset += self.p['repulsion_gain'] * (self.p['warning_distance']-distance) * delta / distance
                    pose = self.poses[index + 1]
                    result = controller.step(relative[index], offset, sample.velocities[index],
                        forward, leader.yaw, leader.yaw_rate,
                        min(self.seconds(pose.measurement_stamp), self.seconds(leader.timestamp)),
                        now, enabled=ready, confidence=pose.confidence)
                    velocities[index] = result.velocity_enu if ready else np.zeros(3)
                    errors.append(float(np.linalg.norm(result.error_flu)))
            except (ValueError, FloatingPointError):
                ready, state = False, 'INVALID_GEOMETRY'
        if not ready:
            self.controllers = self.new_controllers()
            self.reset_prediction()
        for i in [1, 2, 3]:
            command = SwarmCommand(uav_id=i, frame_id='map', valid=bool(ready), safety_state=state)
            command.source_stamp = self.get_clock().now().to_msg()
            if ready:
                command.source_stamp = self.poses[i].measurement_stamp
            command.target_velocity.linear.x, command.target_velocity.linear.y, command.target_velocity.linear.z = map(float, velocities[i-1])
            self.command_pub.publish(command)
        self.status_pub.publish(String(data=json.dumps(
            dict(mode=state, formation=self.formation, scale=self.scale,
                 min_distance=minimum, errors=errors, time=now))))
        if state != self.last_status:
            self.get_logger().info(f'{state}, min_distance={minimum}, t={now:.3f}')
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

