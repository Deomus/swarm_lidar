"""Координатор строя на ведущем: план строя ведомым по mesh и общий статус.

Ведомые сами считают свои команды (node.py) по LiDAR-позам из mesh. Здесь —
только то, что требует согласия всей группы: выбор назначения мест без
пересечения путей, проверка карты, включение и выключение строя, пауза
перехода при выбывании ведомого и сводный статус для ведущего, панели и
сценариев.
"""

import json
import math

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import FormationCommand, FormationPlan, SwarmState

from .core import FormationPlan as Plan, formation_offsets
from .obstacles import check_transition_map

FOLLOWERS = (1, 2, 3)
ACTIVE_MODES = ('FOLLOWING', 'FORMATION_CHANGE')


class FormationCoordinator(Node):
    def __init__(self):
        super().__init__('formation_coordinator')
        defaults = dict(period=.05, stale_timeout=.5, max_speed=2., min_separation=2., gap=4.,
                        require_transition_map=False, transition_map_margin=.75,
                        states_topic='/swarm/states',
                        follower_status_topic='/mesh/follower_status',
                        leader_status_topic='/mission/execution_status',
                        formation_topic='/swarm/formation_command',
                        plan_topic='/swarm/formation_plan',
                        status_topic='/swarm/formation_status', target_topic='/swarm/target_offsets')
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.p = {key: self.get_parameter(key).value for key in defaults}
        self.plan = Plan(self.p['min_separation'] + .2, self.p['gap'],
                         max_slot_speed=self.p['max_speed'] * .75)
        self.enabled, self.epoch = False, 0
        self.formation, self.scale = 'line', 1.
        self.leader = None
        self.followers = {}
        self.leader_intent = None
        self.map = None
        self.previous_time = None
        self.last_status = None
        self.request_error, self.request_stamp = None, -1.
        self.plan_pub = self.create_publisher(FormationPlan, self.p['plan_topic'], 10)
        self.status_pub = self.create_publisher(String, self.p['status_topic'], 10)
        self.targets_pub = self.create_publisher(PoseArray, self.p['target_topic'], 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(SwarmState, self.p['states_topic'], self.on_state, 10)
        self.create_subscription(String, self.p['follower_status_topic'], self.on_follower, 10)
        self.create_subscription(String, self.p['leader_status_topic'], self.on_leader_status, 10)
        self.create_subscription(FormationCommand, self.p['formation_topic'], self.on_formation, 10)
        self.create_service(Trigger, '/swarm/enable_formation', self.enable)
        self.create_service(Trigger, '/swarm/hold', self.hold)
        self.create_timer(self.p['period'], self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def fresh(self, stamp):
        return 0 <= self.now() - stamp <= self.p['stale_timeout']

    def on_map(self, message):
        self.map = message

    def on_state(self, msg):
        # Собственная телеметрия ведущего, локально, не через mesh.
        if msg.uav_id == 0 and self.fresh(self.seconds(msg.timestamp)):
            if self.leader is None or self.seconds(msg.timestamp) >= self.seconds(self.leader.timestamp):
                self.leader = msg

    def on_follower(self, msg):
        try:
            value = json.loads(msg.data)
            uav = value['uav_id']
            if uav in FOLLOWERS and math.isfinite(value['time']) and self.fresh(value['time']):
                old = self.followers.get(uav)
                if old is None or value['time'] >= old['time']:
                    self.followers[uav] = value
        except (ValueError, TypeError, KeyError):
            pass

    def on_leader_status(self, msg):
        try:
            value = json.loads(msg.data)
            if (isinstance(value['valid'], bool) and isinstance(value['status'], str)
                    and math.isfinite(value['stamp']) and self.fresh(value['stamp'])):
                self.leader_intent = value
        except (ValueError, TypeError, KeyError):
            pass

    def leader_ready(self):
        state = self.leader
        return (state is not None and state.valid and state.armed
                and self.fresh(self.seconds(state.timestamp)))

    def follower(self, i):
        """Свежий статус ведомого текущего включения строя или None."""
        status = self.followers.get(i)
        if status is None or not self.fresh(status['time']):
            return None
        return status

    def enable(self, request, response):
        ready = {i: bool((self.follower(i) or {}).get('ready')) for i in FOLLOWERS}
        if not self.leader_ready() or not all(ready.values()):
            response.message = str(dict(time=self.now(), leader=self.leader_ready(), followers=ready))
            return response
        # Новое включение: ведомые сбрасывают регуляторы и прошлую остановку.
        self.enabled, self.epoch = True, self.epoch + 1
        response.success = True
        response.message = 'LiDAR-relative контур включён'
        return response

    def hold(self, request, response):
        self.enabled = False
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
                leader = self.leader
                if leader is None or not leader.valid or leader.frame_id != 'map' or not self.fresh(self.seconds(leader.timestamp)):
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

    def group_state(self, sample):
        """Режим группы по статусам ведомых: остановка любого — остановка всех."""
        if not self.enabled:
            return 'DISABLED', [], []
        statuses = {i: self.follower(i) for i in FOLLOWERS}
        current = {i: s for i, s in statuses.items() if s is not None and s.get('epoch') == self.epoch}
        if any(s['mode'] == 'COLLISION_STOP' for s in current.values()):
            self.enabled = False
            return 'COLLISION_STOP', [], []
        if any(s['mode'] == 'INVALID_GEOMETRY' for s in current.values()):
            return 'INVALID_GEOMETRY', [], list(FOLLOWERS)
        active = [i for i, s in current.items() if s['mode'] in ACTIVE_MODES]
        lost = [i for i in FOLLOWERS if i not in active]
        if not self.leader_ready() or not active:
            return 'DATA_HOLD', [], list(FOLLOWERS)
        if lost:
            return 'DEGRADED', active, lost
        return ('FORMATION_CHANGE' if sample.transitioning else 'FOLLOWING'), active, lost

    def publish_plan(self, now):
        start, end, time, duration = self.plan.snapshot()
        msg = FormationPlan(formation=self.formation, scale=float(self.scale), enabled=self.enabled,
                            epoch=self.epoch, start_time=float(time), duration=float(duration),
                            start_offsets=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in start],
                            end_offsets=[Point(x=float(x), y=float(y), z=float(z)) for x, y, z in end])
        msg.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(msg)

    def tick(self):
        now = self.now()
        previous = self.previous_time
        if previous is not None and now < previous:
            self.enabled = False
            self.leader, self.followers = None, {}
        self.previous_time = now
        sample = self.plan.sample(now)
        state, active, lost = self.group_state(sample)
        # Выбывание, удержание данных или остановка замораживают переход у всех.
        if state not in ACTIVE_MODES and previous is not None and now >= previous:
            self.plan.pause(now - previous)
            sample = self.plan.sample(now)
        self.publish_plan(now)
        targets = PoseArray()
        targets.header.frame_id = 'uav_0/heading'
        targets.header.stamp = self.get_clock().now().to_msg()
        for point in sample.offsets:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, point)
            pose.orientation.w = 1.
            targets.poses.append(pose)
        self.targets_pub.publish(targets)
        statuses = {i: self.follower(i) or {} for i in FOLLOWERS}
        distances = [s['min_distance'] for s in statuses.values()
                     if s.get('epoch') == self.epoch and s.get('min_distance') is not None]
        self.status_pub.publish(String(data=json.dumps(dict(
            mode=state, formation=self.formation, scale=self.scale, time=now,
            min_distance=min(distances) if distances else None,
            errors=[statuses[i].get('error') if i in active else None for i in FOLLOWERS],
            yaw_errors=[statuses[i].get('yaw_error') if i in active else None for i in FOLLOWERS],
            lost=lost if self.enabled else [], epoch=self.epoch,
            leader_mode=(self.leader_intent or {}).get('mode'),
            leader_intent_stamp=(self.leader_intent or {}).get('stamp', -1),
            request_error=self.request_error, request_stamp=self.request_stamp))))
        if (state, lost) != self.last_status:
            self.get_logger().info(f'{state}, выбыли={lost}, t={now:.3f}')
            self.last_status = (state, lost)


def main(args=None):
    rclpy.init(args=args)
    node = FormationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
