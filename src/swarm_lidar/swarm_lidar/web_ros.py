"""ROS-адаптер браузерной панели; HTTP не выполняет обработчики ROS."""

import json
import math
import queue
import threading
import time

from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseStamped
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


def goal_request(body):
    values = [body.get(key) for key in ('x', 'y', 'z')]
    if (any(isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) for value in values)
            or not all(-100. <= value <= 100. for value in values[:2])
            or not 1. <= values[2] <= 30.):
        raise ValueError('Точка: X/Y от −100 до 100 м, высота от 1 до 30 м')
    return tuple(float(value) for value in values)


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
        self.publisher = self.create_publisher(FormationCommand, '/swarm/formation_command', 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/mission/goal', 10)
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
                                         yaw=msg.yaw, armed=msg.armed, mode=msg.mode,
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
            return dict(connected=connected, time=now, drones=rows, formation_settled=settled,
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
            if action in ('prepare', 's1', 's2', 's3'):
                if self.action['state'] == 'pending' or self.flight.active():
                    raise ValueError('Дождитесь завершения текущей операции')
                if not snapshot['connected'] or any(row.get('armed') for row in snapshot['drones']):
                    raise ValueError('Нужны свежие состояния четырёх аппаратов на земле')
                self.flight.start('operator' if action == 'prepare' else action, body.get('record', True))
                self.action = dict(state='ok', message='Подготовка началась. Ожидайте подтверждённого взлёта.')
                return
            if action == 'land':
                if session['phase'] == 'landing':
                    return
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
            if action == 'goal':
                if session['phase'] != 'ready' or session['kind'] != 'operator' or not snapshot['connected']:
                    raise ValueError('Сначала выполните «Подготовить и взлететь» и дождитесь готовности')
                if self.action['state'] == 'pending':
                    raise ValueError('Дождитесь завершения предыдущей команды')
                x, y, z = goal_request(body)
                goal = PoseStamped()
                goal.header.frame_id = 'map'
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose.position.x, goal.pose.position.y, goal.pose.position.z = x, y, z
                goal.pose.orientation.w = 1.
                self.goal_publisher.publish(goal)
                self.action = dict(state='ok', message=f'Маршрут задан: X={x:.1f}, Y={y:.1f}, высота={z:.1f} м')
                return
            if action not in ('formation', 'hold'):
                raise ValueError('Неизвестная команда')
            if session['phase'] != 'ready' or session['kind'] != 'operator' or not snapshot['connected']:
                raise ValueError('Сначала выполните «Подготовить и взлететь» и дождитесь готовности')
            if self.action['state'] == 'pending':
                raise ValueError('Дождитесь подтверждения предыдущей команды')
            values = formation_request(body) if action == 'formation' else None
            if action == 'formation' and not snapshot['formation_settled']:
                raise ValueError('Дождитесь установления текущего строя')
            self.commands.put_nowait((action, values))
            self.action = dict(state='pending', message='Команда отправляется; ожидайте подтверждения.')

    def call(self, path):
        client = self.service_clients.get(path)
        if client is None:
            client = self.create_client(Trigger, path)
            self.service_clients[path] = client
        if not client.service_is_ready():
            raise ValueError('Команда сейчас недоступна: '+path)
        return client.call_async(Trigger.Request())

    def tick(self):
        with self.lock:
            now, wall = self.now(), time.monotonic()
            if self.clock_previous is not None and now < self.clock_previous:
                self.states.clear()
                self.formation = self.execution = None
                self.flight.stop()
            if now != self.clock_previous:
                self.clock_progress = wall
                self.clock_previous = now
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
                    elif name == 'formation':
                        status = self.formation or {}
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
                        self.action = dict(state='ok', message='Посадка запрошена.' if name == 'finish'
                                           else 'Ведущий удерживает положение.')
                        self.pending = None
                    if self.pending is not None and wall >= deadline:
                        raise ValueError('Не получено подтверждение команды')
                except Exception as error:
                    if name == 'finish':
                        self.flight.stop()
                    self.action = dict(state='error', message=str(error))
                    self.pending = None
            # Посадка не ждёт завершения запроса перестроения или HOLD.
            if not self.commands.empty():
                name, values = self.commands.get_nowait()
                try:
                    if name == 'land':
                        self.last_landing_request = 0.
                        future = None
                    elif name == 'formation':
                        self.publisher.publish(FormationCommand(formation=values[0], scale=values[1],
                                                               transition_time=values[2], source='web_panel'))
                        future = None
                    else:
                        future = self.call('/scenario/finish' if name == 'finish' else '/leader/hold')
                    self.pending = (name, future, wall+(75. if name == 'land' else 5.), values, now)
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
