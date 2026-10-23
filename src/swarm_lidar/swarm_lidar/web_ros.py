"""ROS-адаптер браузерной панели; HTTP не выполняет обработчики ROS."""

import json
import math
import queue
import threading
import time

from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
from swarm_interfaces.msg import FormationCommand, SwarmState

from formation_controller.core import formation_offsets
from .web_session import FlightSession


def formation_request(body):
    name = body.get('formation')
    if name not in ('line', 'column', 'wedge'):
        raise ValueError('Выберите линию, колонну или клин')
    scale, duration = body.get('scale'), body.get('duration')
    if (isinstance(scale, bool) or isinstance(duration, bool)
            or not isinstance(scale, (int, float)) or not isinstance(duration, (int, float))
            or not math.isfinite(scale) or not math.isfinite(duration)
            or not .75 <= scale <= 2. or not 2. <= duration <= 120.):
        raise ValueError('Масштаб: 0,75–2; время перестроения: 2–120 с')
    formation_offsets(name, scale, gap=4., min_separation=2.)
    return name, float(scale), float(duration)


def goal_request(body, current_altitude=None):
    """Без z рой летит на текущей высоте ведущего, а не на фиксированной."""
    values = [body.get(key) for key in ('x', 'y', 'z')]
    if values[2] is None:
        if current_altitude is None:
            raise ValueError('Нет свежей высоты ведущего для маршрута')
        values[2] = current_altitude
    if (any(isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) for value in values)
            or not all(-100. <= value <= 100. for value in values[:2])
            or not 1. <= values[2] <= 30.):
        raise ValueError('Точка: X/Y от −100 до 100 м, высота от 1 до 30 м')
    return tuple(float(value) for value in values)


MANUAL_SPEEDS = (.5, 1., 1.5)
MANUAL_VERTICAL_SPEED = .5
MANUAL_YAW_RATE = .3
MANUAL_REPEAT_TIMEOUT = .3
MANUAL_BRAKE_TIME = 3.
MANUAL_MIN_ALTITUDE = 1.
# Предел сбора строя (время симуляции) для цели, выбранной до взлёта.
GOAL_FORMING_TIMEOUT = 300.


def manual_request(body):
    """Кнопки пульта: направления −1/0/1 относительно курса ведущего и скорость."""
    axes = [body.get(key, 0) for key in ('forward', 'left', 'up', 'yaw')]
    speed = body.get('speed')
    if (any(isinstance(value, bool) or value not in (-1, 0, 1) for value in axes)
            or isinstance(speed, bool) or speed not in MANUAL_SPEEDS):
        raise ValueError('Направления −1/0/1 и скорость 0,5 / 1 / 1,5 м/с')
    return (*map(int, axes), float(speed))


def manual_velocity(command, yaw, altitude):
    """Скорость ведущего в map ENU и yaw_rate; ниже 1 м снижение не выдаётся."""
    forward, left, up, turn, speed = command
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = forward*c - left*s, forward*s + left*c
    norm = math.hypot(vx, vy)
    if norm:
        vx, vy = vx*speed/norm, vy*speed/norm
    vz = up*MANUAL_VERTICAL_SPEED if up > 0 or altitude > MANUAL_MIN_ALTITUDE else 0.
    return vx, vy, vz, turn*MANUAL_YAW_RATE


class PanelBackend(Node):
    def __init__(self, log_dir):
        super().__init__('swarm_web_panel', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.lock = threading.RLock()
        self.flight = FlightSession(log_dir)
        self.states = {}
        self.formation = None
        self.settled_since = None
        self.settled_target = None
        self.execution = None
        self.mission_status = None
        self.goal_waiting = False
        self.goal_id = None
        self.goal_result = None
        self.goal_deadline = 0.
        # Последняя точка карты, отклонённая планировщиком: панель показывает её под картой.
        self.goal_point = self.goal_error = None
        self.create_subscription(String, '/mission/plan_result', self.on_plan_result, 10)
        self.clock_previous = None
        self.clock_progress = 0.
        self.action = dict(state='idle', message='Запустите симуляцию и дождитесь подключения четырёх аппаратов.')
        self.commands = queue.Queue(maxsize=8)
        self.pending = None
        self.closing = False
        self.last_landing_request = 0.
        self.service_clients = {}
        self.create_subscription(SwarmState, '/mesh/states', self.on_state, 10)
        self.create_subscription(String, '/swarm/formation_status', self.on_formation, 10)
        self.create_subscription(String, '/mission/execution_status', self.on_execution, 10)
        self.create_subscription(String, '/mission/status', self.on_mission_status, 10)
        self.swarm_mode = None
        # Занятые клетки карты — для отрисовки препятствий в панели.
        self.obstacles = []
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/swarm/mode', lambda m: self.status_message(m, 'swarm_mode', 'time'), 10)
        self.publisher = self.create_publisher(FormationCommand, '/swarm/formation_command', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/mission/goal', 10)
        self.manual_publisher = self.create_publisher(TwistStamped, '/leader/cmd_vel', 10)
        self.manual = None
        self.manual_requested_until = 0.
        # Взлёты, нажатые до готовности сессии: отправляются исполнителю позже.
        self.queued_takeoffs = []
        # Цель с карты, выбранная до взлёта: отправляется планировщику после сбора строя.
        self.queued_goal = None
        self.create_timer(.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_state(self, msg):
        stamp = msg.timestamp.sec+msg.timestamp.nanosec*1e-9
        p, v = msg.pose.position, msg.velocity.linear
        values = [p.x, p.y, p.z, v.x, v.y, v.z, msg.yaw]
        if (msg.uav_id not in range(4) or not all(math.isfinite(x) for x in values)
                or not 0 <= self.now()-stamp <= .5):
            return
        with self.lock:
            old = self.states.get(msg.uav_id)
            if old is not None and stamp < old['stamp']:
                return
            self.states[msg.uav_id] = dict(id=msg.uav_id, position=values[:3], velocity=values[3:6],
                                         yaw=msg.yaw, armed=msg.armed, mode=msg.mode, health=msg.health,
                                         valid=msg.valid and msg.frame_id == 'map', stamp=stamp,
                                         received=time.monotonic())

    def status_message(self, msg, field, stamp_key):
        try:
            value = json.loads(msg.data)
            if (not isinstance(value, dict) or not math.isfinite(value[stamp_key])
                    or not 0 <= self.now()-value[stamp_key] <= .5):
                return
            # Не передаём NaN/Infinity из стороннего JSON в браузер.
            json.dumps(value, allow_nan=False)
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            old = getattr(self, field)
            if old is None or value[stamp_key] >= old[stamp_key]:
                setattr(self, field, value)

    def on_formation(self, msg):
        self.status_message(msg, 'formation', 'time')

    def on_execution(self, msg):
        self.status_message(msg, 'execution', 'stamp')

    def on_map(self, msg):
        info = msg.info
        cells = [(info.origin.position.x + (i % info.width + .5) * info.resolution,
                  info.origin.position.y + (i // info.width + .5) * info.resolution)
                 for i, value in enumerate(msg.data) if value >= 50][:2000]
        with self.lock:
            self.obstacles = dict(resolution=info.resolution, cells=cells)

    def on_mission_status(self, msg):
        with self.lock:
            self.mission_status = msg.data

    def on_plan_result(self, msg):
        try:
            result = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self.lock:
            if isinstance(result, dict) and self.goal_waiting and result.get('goal_id') == self.goal_id:
                self.goal_result = result

    def cancel_goal(self):
        self.queued_goal = None
        self.goal_waiting = False
        self.goal_id = self.goal_result = None
        while not self.commands.empty():
            self.commands.get_nowait()
        if self.pending and self.pending[1] is not None:
            self.pending[1].cancel()
        self.pending = None

    def fresh(self, row, now, wall):
        return (row['valid'] and 0 <= now-row['stamp'] <= .5
                and wall-row['received'] <= 1. and wall-self.clock_progress <= 1.)

    def snapshot(self):
        with self.lock:
            now, wall = self.now(), time.monotonic()
            rows = []
            for i in range(4):
                row = self.states.get(i)
                rows.append(dict(**{k: v for k, v in row.items() if k != 'received'},
                                 fresh=self.fresh(row, now, wall)) if row else dict(id=i, fresh=False))
            connected = all(row['fresh'] for row in rows)
            formation = self.formation
            formation_fresh = (formation is not None and 0 <= now-formation['time'] <= .5
                               and wall-self.clock_progress <= 1.)
            settled = (formation_fresh and connected and self.settled_since is not None
                       and now-self.settled_since >= 1.)
            manual = self.manual
            moving = bool(manual and wall-manual['wall'] <= MANUAL_REPEAT_TIMEOUT and any(manual['command'][:4]))
            return dict(connected=connected, time=now, drones=rows, formation_settled=settled,
                        manual=moving, obstacles=self.obstacles, goal_error=self.goal_error,
                        swarm_mode=self.swarm_mode if (self.swarm_mode is not None
                            and 0 <= now-self.swarm_mode['time'] <= 1.) else None,
                        formation=formation if formation_fresh else None,
                        execution=self.execution if (self.execution is not None
                            and 0 <= now-self.execution['stamp'] <= .5) else None,
                        flight=self.flight.snapshot(), action=self.action.copy())

    def submit(self, action, body):
        with self.lock:
            if self.closing:
                raise ValueError('Панель завершает работу и ожидает посадки')
            snapshot = self.snapshot()
            session = snapshot['flight']
            if action == 'prepare':
                if self.action['state'] == 'pending' or self.flight.active():
                    raise ValueError('Дождитесь завершения текущей операции')
                if not snapshot['connected'] or any(row.get('armed') for row in snapshot['drones']):
                    raise ValueError('Нужны свежие состояния четырёх аппаратов на земле')
                self.flight.start('operator', body.get('record', True))
                self.action = dict(state='ok', message='Подготовка началась. Ожидайте подтверждённого взлёта.')
                return
            if action in ('takeoff', 'land_uav'):
                uav = body.get('uav')
                if isinstance(uav, bool) or uav not in (0, 1, 2, 3):
                    raise ValueError('Номер аппарата: 0–3')
                if action == 'takeoff' and not self.flight.active():
                    # Первая кнопка «Взлететь» начинает сессию с записью; исполнитель
                    # сначала проверяет аппараты на земле, затем принимает взлёты.
                    if self.action['state'] == 'pending':
                        raise ValueError('Дождитесь завершения текущей операции')
                    if not snapshot['connected'] or any(row.get('armed') for row in snapshot['drones']):
                        raise ValueError('Нужны свежие состояния четырёх аппаратов на земле')
                    self.flight.start('operator', body.get('record', True))
                    self.queued_takeoffs = [uav]
                    self.action = dict(state='ok', message='Полёт начат: проверка аппаратов на земле, затем взлёт.')
                    return
                if action == 'takeoff' and session['phase'] in ('starting', 'running'):
                    if uav not in self.queued_takeoffs:
                        self.queued_takeoffs.append(uav)
                    return
                if session['phase'] != 'ready' or session['kind'] != 'operator':
                    raise ValueError('Команда доступна во время полёта, начатого из панели')
                self.commands.put_nowait((action, uav))
                return
            if action == 'land':
                if session['phase'] == 'landing':
                    return
                self.manual = None
                self.queued_takeoffs = []
                self.cancel_goal()
                if self.flight.active():
                    while not self.commands.empty():
                        self.commands.get_nowait()
                    if self.pending and self.pending[1] is not None:
                        self.pending[1].cancel()
                    self.pending = None
                    if session['phase'] == 'ready' and session['kind'] == 'operator':
                        self.flight.mark_landing()
                        self.commands.put_nowait(('finish', None))
                    else:
                        self.flight.stop()
                    self.action = dict(state='pending', message='Выполняется посадка. Ожидайте разоружения.')
                    return
                if snapshot['connected'] and all(not row.get('armed') for row in snapshot['drones']):
                    self.action = dict(state='ok', message='Все аппараты на земле и разоружены.')
                    return
                self.commands.put_nowait(('land', None))
                self.action = dict(state='pending', message='Запрошена посадка всех аппаратов; ожидайте подтверждения.')
                return
            grounded = [row['id'] for row in snapshot['drones'] if not row.get('armed')]
            if action == 'goal' and grounded and (not self.flight.active() or (
                    session['kind'] == 'operator' and session['phase'] in ('starting', 'running', 'ready'))):
                self.queue_goal_takeoff(body, snapshot, grounded)
                return
            if action in ('goal', 'altitude', 'manual', 'formation', 'hold') and (
                    session['phase'] != 'ready' or session['kind'] != 'operator' or not snapshot['connected']):
                raise ValueError('Сначала поднимите аппараты кнопками «Взлететь» и дождитесь готовности')
            leader = snapshot['drones'][0]
            if action == 'manual':
                command = manual_request(body)
                if (not leader.get('armed') or leader.get('mode') != '14'
                        or leader['position'][2] < MANUAL_MIN_ALTITUDE):
                    raise ValueError('Ручное управление — только ведущим в полёте: Offboard и выше 1 м')
                wall = time.monotonic()
                self.manual = dict(command=command, wall=wall)
                if (any(command[:4]) and (self.execution or {}).get('mode') != 'MANUAL'
                        and wall >= self.manual_requested_until):
                    self.cancel_goal()
                    self.commands.put_nowait(('manual', None))
                    self.manual_requested_until = wall + 2.
                    self.action = dict(state='pending', message='Включается ручное управление ведущим.')
                return
            if action in ('goal', 'altitude'):
                if self.action['state'] == 'pending':
                    raise ValueError('Дождитесь завершения предыдущей команды')
                if action == 'altitude':
                    # Цель в текущих X/Y: ведущий меняет только высоту, ведомые следуют.
                    x, y, z = goal_request(dict(x=leader['position'][0], y=leader['position'][1],
                                                z=body.get('z')))
                else:
                    x, y, z = goal_request(body, leader.get('position', [None]*3)[2])
                self.publish_goal(x, y, z, action)
                return
            if action == 'resume':
                # После эскалации моста (Loiter PX4) возврат только по решению оператора.
                targets = [row['id'] for row in snapshot['drones']
                           if row.get('armed') and row.get('mode') != '14' and row.get('health') != 'FAILSAFE_LAND']
                if not targets:
                    raise ValueError('Нет аппаратов в удержании PX4 для возврата в строй')
                if self.action['state'] == 'pending':
                    raise ValueError('Дождитесь завершения предыдущей команды')
                self.commands.put_nowait(('resume', targets))
                self.action = dict(state='pending', message='Возврат в строй: включается Offboard.')
                return
            if action not in ('formation', 'hold'):
                raise ValueError('Неизвестная команда')
            if self.action['state'] == 'pending' and action != 'hold':
                raise ValueError('Дождитесь подтверждения предыдущей команды')
            if action == 'hold':
                self.manual = None
                self.cancel_goal()
            values = formation_request(body) if action == 'formation' else None
            if action == 'formation' and (self.execution or {}).get('mode') in ('MISSION', 'MANUAL'):
                raise ValueError('Перед перестроением нажмите «Удерживать»')
            if action == 'formation' and not snapshot['formation_settled']:
                raise ValueError('Дождитесь установления текущего строя')
            self.commands.put_nowait((action, values))
            self.action = dict(state='pending', message='Команда отправляется; ожидайте подтверждения.')

    def queue_goal_takeoff(self, body, snapshot, grounded):
        """Сценарий C с земли: взлёт, сбор строя, затем маршрут к цели с карты."""
        goal_request(body, MANUAL_MIN_ALTITUDE)  # проверка X/Y до взлёта; высоту задаст ведущий
        if self.action['state'] == 'pending':
            raise ValueError('Дождитесь завершения текущей операции')
        if not self.flight.active():
            if not snapshot['connected'] or len(grounded) != 4:
                raise ValueError('Нужны свежие состояния четырёх аппаратов на земле')
            self.flight.start('operator', body.get('record', True))
        self.queued_takeoffs += [uav for uav in grounded if uav not in self.queued_takeoffs]
        self.manual = None
        message = (f'Взлёт и сбор строя, затем полёт к цели X={body["x"]:.1f}, Y={body["y"]:.1f}')
        self.goal_error = None
        self.queued_goal = dict(body=dict(body), deadline=self.now() + GOAL_FORMING_TIMEOUT, message=message)
        self.action = dict(state='pending', message=message)

    def publish_goal(self, x, y, z, action='goal'):
        self.manual = None
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
        goal.pose.orientation.w = 1.
        self.mission_status = None
        self.goal_publisher.publish(goal)
        self.goal_point = (x, y) if action == 'goal' else None
        if action == 'goal':
            self.goal_error = None
        self.goal_waiting = True
        self.goal_id = [goal.header.stamp.sec, goal.header.stamp.nanosec]
        self.goal_deadline = time.monotonic() + 10.
        self.goal_result = None
        self.action = dict(state='pending', message=(
            f'Высота роя меняется на {z:.1f} м' if action == 'altitude'
            else f'Маршрут строится: X={x:.1f}, Y={y:.1f}, высота={z:.1f} м'))

    def send_queued_goal(self, now):
        """Цель, выбранная до взлёта, уходит планировщику, когда строй собран."""
        queued = self.queued_goal
        snapshot = self.snapshot()
        session = snapshot['flight']
        if not self.flight.active() or session['kind'] != 'operator' or self.closing:
            self.queued_goal = None
            return
        if now >= queued['deadline']:
            self.queued_goal = None
            self.action = dict(state='error', message='Строй не собрался вовремя: полёт к цели отменён, рой ждёт команды')
            return
        rows = snapshot['drones']
        if (session['phase'] != 'ready' or not snapshot['formation_settled'] or self.queued_takeoffs
                or not all(row.get('armed') and row.get('mode') == '14' for row in rows)):
            return
        self.queued_goal = None
        try:
            self.publish_goal(*goal_request(queued['body'], rows[0]['position'][2]))
        except ValueError as error:
            self.action = dict(state='error', message=str(error))

    def call(self, path):
        client = self.service_clients.get(path)
        if client is None:
            client = self.create_client(Trigger, path)
            self.service_clients[path] = client
        if not client.service_is_ready():
            raise ValueError('Команда сейчас недоступна: '+path)
        return client.call_async(Trigger.Request())

    def publish_manual(self, now, wall):
        """Пока кнопка удерживается, браузер повторяет команду; без повторов —
        нули для плавного торможения, затем публикация прекращается и
        ведущий сам переходит в DEADMAN."""
        manual, leader = self.manual, self.states.get(0)
        if manual is None:
            return
        age = wall - manual['wall']
        if (age > MANUAL_BRAKE_TIME or self.flight.snapshot()['phase'] != 'ready'
                or leader is None or not self.fresh(leader, now, wall)):
            self.manual = None
            return
        command = manual['command'] if age <= MANUAL_REPEAT_TIMEOUT else (0, 0, 0, 0, manual['command'][4])
        vx, vy, vz, yaw_rate = manual_velocity(command, leader['yaw'], leader['position'][2])
        message = TwistStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = vx, vy, vz
        message.twist.angular.z = yaw_rate
        self.manual_publisher.publish(message)

    def tick(self):
        with self.lock:
            now, wall = self.now(), time.monotonic()
            if self.clock_previous is not None and now < self.clock_previous:
                self.cancel_goal()
                self.states.clear()
                self.formation = self.execution = None
                self.flight.stop()
            if now != self.clock_previous:
                self.clock_progress = wall
                self.clock_previous = now
            self.publish_manual(now, wall)
            if self.queued_takeoffs and self.flight.snapshot()['phase'] == 'ready':
                for uav in self.queued_takeoffs:
                    self.commands.put_nowait(('takeoff', uav))
                self.queued_takeoffs = []
            status = self.formation or {}
            errors = status.get('errors', [])
            close = (status.get('mode') == 'FOLLOWING' and 0 <= now-status.get('time', -1) <= .5
                     and all(i in self.states and self.fresh(self.states[i], now, wall) for i in range(4))
                     and isinstance(errors, list) and len(errors) == 3
                     and all(isinstance(e, (int, float)) and not isinstance(e, bool)
                             and math.isfinite(e) and 0 <= e < .6 for e in errors))
            target = (status.get('formation'), status.get('scale'))
            if not close:
                self.settled_since = self.settled_target = None
            elif self.settled_since is None or target != self.settled_target:
                self.settled_since, self.settled_target = now, target
            if self.pending is not None:
                name, future, deadline, target, sent_at = self.pending
                try:
                    if name == 'land':
                        rows = self.snapshot()['drones']
                        if all(row['fresh'] and not row.get('armed') for row in rows):
                            self.action = dict(state='ok', message='Все аппараты разоружены; посадка подтверждена.')
                            self.pending = None
                        elif wall-self.last_landing_request >= 3.:
                            self.last_landing_request = wall
                            for path in ['/leader/hold', '/swarm/hold', *[f'/uav_{i}/land' for i in range(4)]]:
                                try:
                                    self.call(path)
                                except ValueError:
                                    pass  # Продолжаем запросы остальным; подтверждение — свежая телеметрия.
                    elif name == 'resume':
                        rows = self.snapshot()['drones']
                        if all(row['fresh'] and row.get('mode') == '14' for row in rows if row.get('armed')):
                            self.pending = None
                            if (self.formation or {}).get('mode') in ('DISABLED', 'COLLISION_STOP'):
                                self.commands.put_nowait(('enable', None))
                            else:
                                self.action = dict(state='ok', message='Аппараты вернулись в Offboard и строй.')
                    elif name == 'formation':
                        status = self.formation or {}
                        if (status.get('request_error') and sent_at <= status.get('request_stamp', -1) <= now
                                and 0 <= now-status.get('time', -1) <= .5):
                            raise ValueError(status['request_error'])
                        matched = (status.get('formation') == target[0]
                                   and abs(status.get('scale', 0)-target[1]) < .01
                                   and sent_at <= status.get('time', -1) <= now
                                   and now-status['time'] <= .5
                                   and status.get('mode') in ('FOLLOWING', 'FORMATION_CHANGE'))
                        if matched:
                            self.action = dict(state='ok', message='Перестроение принято. Следите за состоянием строя.')
                            self.pending = None
                    elif future.done():
                        response = future.result()
                        if response is None or not response.success:
                            raise ValueError('Команда отклонена: '+str(getattr(response, 'message', 'нет ответа')))
                        execution = self.execution or {}
                        hold_confirmed = (execution.get('mode') == 'HOLD'
                            and sent_at <= execution.get('stamp', -1) <= now
                            and now-execution['stamp'] <= .5
                            and (self.formation or {}).get('leader_mode') == 'HOLD'
                            and sent_at <= (self.formation or {}).get('leader_intent_stamp', -1) <= now)
                        if name in ('takeoff', 'land_uav'):
                            self.action = (dict(state='ok', message=response.message) if self.queued_goal is None
                                           else dict(state='pending', message=self.queued_goal['message']))
                            self.pending = None
                        elif name != 'hold' or hold_confirmed:
                            self.action = dict(state='ok', message={
                                'mission': 'Режим миссии включён.', 'finish': 'Посадка запрошена.',
                                'manual': 'Ручное управление ведущим включено.',
                                'enable': 'Аппараты вернулись в Offboard, строй снова включён.'}.get(
                                name, 'Ведущий удерживает положение.'))
                            self.pending = None
                    if self.pending is not None and wall >= deadline:
                        raise ValueError('Не получено подтверждение команды')
                except Exception as error:
                    if name == 'finish':
                        self.flight.stop()
                    self.action = dict(state='error', message=str(error))
                    self.pending = None
            if self.queued_goal is not None:
                self.send_queued_goal(now)
            if self.goal_waiting:
                result = self.goal_result or {}
                session = self.flight.snapshot()
                allowed = session['phase'] == 'ready' and session['kind'] == 'operator' and not self.closing
                if not allowed or wall >= self.goal_deadline:
                    self.cancel_goal()
                    self.action = dict(state='error', message='Ожидание маршрута отменено или истёк срок подтверждения.')
                elif result.get('status') == 'PLANNED' and (self.execution or {}).get('accepted_path_stamp') == result.get('path_stamp'):
                    self.commands.put_nowait(('mission', None))
                    self.goal_waiting = False
                    self.action = dict(state='pending', message='Маршрут построен; ведущий начинает полёт.')
                elif result.get('status') == 'REJECTED':
                    self.goal_waiting = False
                    reason = str(result.get('reason', 'нет причины'))
                    self.action = dict(state='error', message='Маршрут отклонён: '+reason)
                    if self.goal_point is not None:
                        self.goal_error = dict(x=self.goal_point[0], y=self.goal_point[1], reason=reason)
            # Посадка не ждёт завершения запроса перестроения или HOLD.
            if not self.commands.empty():
                name, values = self.commands.get_nowait()
                try:
                    if name == 'mission':
                        if self.flight.snapshot()['phase'] != 'ready' or self.closing:
                            raise ValueError('Запуск миссии отменён')
                        future = self.call('/leader/mission')
                    elif name == 'manual':
                        future = self.call('/leader/manual')
                    elif name in ('takeoff', 'land_uav'):
                        future = self.call(f'/scenario/uav_{values}/{"takeoff" if name == "takeoff" else "land"}')
                    elif name == 'enable':
                        future = self.call('/swarm/enable_formation')
                    elif name == 'resume':
                        for i in values:
                            self.call(f'/uav_{i}/resume')
                        future = None
                    elif name == 'land':
                        self.last_landing_request = 0.
                        future = None
                    elif name == 'formation':
                        self.publisher.publish(FormationCommand(formation=values[0], scale=values[1],
                                                               transition_time=values[2], source='web_panel'))
                        future = None
                    else:
                        future = self.call('/scenario/finish' if name == 'finish' else '/leader/hold')
                    self.pending = (name, future, wall+{'land': 75., 'resume': 15.}.get(name, 5.), values, now)
                except Exception as error:
                    if name == 'finish':
                        self.flight.stop()
                    self.action = dict(state='error', message=str(error))
            if (self.action['state'] == 'pending' and not self.flight.active()
                    and self.pending is None and self.commands.empty()):
                phase = self.flight.snapshot()['phase']
                self.action = dict(state='error' if phase == 'error' else 'ok',
                                   message='Проверьте журнал полёта.' if phase == 'error'
                                   else 'Полёт завершён; посадка подтверждена.')
