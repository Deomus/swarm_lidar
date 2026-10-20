"""Терминальное управление уже летящим ведущим, без arm и переключения PX4."""

from contextlib import contextmanager
import math
import os
import select
import signal
import sys
import termios
import time
import tty


class KeyboardLogic:
    """Клавиши и защита по двум часам; не зависит от ROS или терминала."""

    DIRECTIONS = {'w': (1., 0., 0., 0.), 's': (-1., 0., 0., 0.),
                  'a': (0., 1., 0., 0.), 'd': (0., -1., 0., 0.),
                  'r': (0., 0., 1., 0.), 'f': (0., 0., -1., 0.),
                  'q': (0., 0., 0., 1.), 'e': (0., 0., 0., -1.)}

    def __init__(self, deadman=.25, clock_timeout=.25, state_timeout=.5,
                 max_speed=1.5, max_yaw_rate=.3):
        values = (deadman, clock_timeout, state_timeout, max_speed, max_yaw_rate)
        if (not all(math.isfinite(v) and v > 0 for v in values)
                or max_speed > 1.5 or max_yaw_rate > .3):
            raise ValueError('Некорректные ограничения клавиатурного управления')
        self.deadman, self.clock_timeout, self.state_timeout = values[:3]
        self.max_speed, self.max_yaw_rate = max_speed, max_yaw_rate
        self.speed = min(.5, max_speed)
        self.clock = self.clock_progress_wall = None
        self.state_stamp = self.state_wall = None
        self.state_valid = False
        self.key_wall = None
        self.direction = (0., 0., 0., 0.)
        self.enabled = False
        self.clock_fault = None

    def clear_motion(self):
        self.direction = (0., 0., 0., 0.)
        self.key_wall = None

    def note_clock(self, stamp, wall):
        if not math.isfinite(stamp) or stamp < 0 or not math.isfinite(wall):
            self.clock_fault = 'CLOCK_INVALID'
        elif self.clock is not None and (stamp < self.clock or stamp - self.clock > .5):
            self.clock_fault = 'CLOCK_JUMP'
        if self.clock_fault is not None:
            self.enabled = False
            self.clear_motion()
        if math.isfinite(stamp) and stamp >= 0 and math.isfinite(wall):
            if self.clock is None or stamp != self.clock:
                self.clock_progress_wall = wall
            self.clock = stamp

    def note_state(self, stamp, valid, wall):
        self.state_stamp, self.state_valid, self.state_wall = stamp, valid, wall

    def problem(self, wall, include_fault=True):
        if not math.isfinite(wall):
            return 'WALL_CLOCK_INVALID'
        if include_fault and self.clock_fault is not None:
            return self.clock_fault
        if (self.clock is None or self.clock <= 0 or self.clock_progress_wall is None
                or not 0 <= wall - self.clock_progress_wall <= self.clock_timeout):
            return 'CLOCK_STOPPED'
        if (not self.state_valid or self.state_stamp is None or self.state_wall is None
                or not math.isfinite(self.state_stamp)
                or not 0 <= wall - self.state_wall <= self.state_timeout
                or not 0 <= self.clock - self.state_stamp <= self.state_timeout):
            return 'STATE_UNAVAILABLE'
        return None

    def key(self, key, wall):
        key = key.lower()
        if key in ('\x1b', '\x03'):
            self.enabled = False
            self.clear_motion()
            return 'EXIT'
        if key == ' ':
            self.enabled = False
            self.clear_motion()
            return 'HOLD'
        if key in ('\r', '\n'):
            self.enabled = False
            self.clear_motion()
            if self.problem(wall, include_fault=False) is None:
                self.clock_fault = None
                return 'MANUAL'
            return 'HOLD'
        if key in ('1', '2', '3'):
            self.speed = min({'1': .5, '2': 1., '3': 1.5}[key], self.max_speed)
            self.clear_motion()
        elif key in self.DIRECTIONS and self.enabled and self.problem(wall) is None:
            self.direction, self.key_wall = self.DIRECTIONS[key], wall
        return None

    def velocity(self, wall):
        if (not self.enabled or self.problem(wall) is not None or self.key_wall is None
                or not 0 <= wall - self.key_wall < self.deadman):
            return (0., 0., 0., 0.)
        return tuple(v * (self.max_yaw_rate if i == 3 else self.speed)
                     for i, v in enumerate(self.direction))


@contextmanager
def terminal_input(stream):
    descriptor = stream.fileno()
    if not os.isatty(descriptor):
        raise ValueError('Нужен интерактивный терминал; перенаправление ввода не поддерживается')
    saved = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield descriptor
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, saved)


class KeyboardController:
    def __init__(self, node):
        from geometry_msgs.msg import TwistStamped
        from rosgraph_msgs.msg import Clock
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_srvs.srv import Trigger
        from swarm_interfaces.msg import SwarmState

        self.node = node
        self.TwistStamped, self.Trigger = TwistStamped, Trigger
        defaults = dict(deadman=.25, clock_timeout=.25, state_timeout=.5,
                        max_speed=1.5, max_yaw_rate=.3, min_height=1.,
                        manual_topic='/leader/cmd_vel', states_topic='/mesh/states')
        for name, value in defaults.items():
            node.declare_parameter(name, value)
        self.p = {name: node.get_parameter(name).value for name in defaults}
        if not math.isfinite(self.p['min_height']) or self.p['min_height'] < 0:
            raise ValueError('Минимальная высота должна быть конечной и неотрицательной')
        self.logic = KeyboardLogic(**{name: self.p[name] for name in (
            'deadman', 'clock_timeout', 'state_timeout', 'max_speed', 'max_yaw_rate')})
        self.desired = 'MANUAL'
        self.confirmed = None
        self.future = None
        self.last_request = -math.inf
        self.last_publish = -math.inf
        self.had_live_data = False
        self.last_problem = None
        self.publisher = node.create_publisher(TwistStamped, self.p['manual_topic'], 10)
        self.clients = {mode: node.create_client(Trigger, f'/leader/{mode.lower()}')
                        for mode in ('HOLD', 'MANUAL')}
        self.subscriptions = [
            node.create_subscription(Clock, '/clock', self.on_clock,
                                     QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)),
            node.create_subscription(SwarmState, self.p['states_topic'], self.on_state, 10)]

    def on_clock(self, message):
        stamp = message.clock.sec + message.clock.nanosec * 1e-9
        self.logic.note_clock(stamp, time.monotonic())

    def on_state(self, message):
        if message.uav_id != 0:
            return
        position = message.pose.position
        valid = (message.valid and message.armed and message.mode == '14'
                 and message.frame_id == 'map'
                 and all(math.isfinite(v) for v in (position.x, position.y, position.z,
                                                     message.yaw, message.yaw_rate))
                 and position.z >= self.p['min_height'])
        self.logic.note_state(message.timestamp.sec + message.timestamp.nanosec * 1e-9,
                              valid, time.monotonic())

    def select(self, mode):
        self.desired = mode
        self.confirmed = None
        self.last_request = -math.inf
        self.logic.enabled = False
        self.logic.clear_motion()

    def publish(self, velocity):
        # Не освежаем сообщения по остановившимся часам даже для нулевой команды.
        if self.logic.problem(time.monotonic()) is not None:
            return
        message = self.TwistStamped()
        ns = round(self.logic.clock * 1e9)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(ns, 10**9)
        message.header.frame_id = 'map'
        v = message.twist.linear
        v.x, v.y, v.z = velocity[:3]
        message.twist.angular.z = velocity[3]
        self.publisher.publish(message)

    def tick(self, wall):
        problem = self.logic.problem(wall)
        if problem != self.last_problem:
            if problem is not None:
                print(f'Движение запрещено: {problem}. После восстановления нажмите Enter.', flush=True)
            self.last_problem = problem
        if problem is None:
            self.had_live_data = True
        elif (self.had_live_data or self.logic.clock_fault is not None) and self.desired != 'HOLD':
            self.select('HOLD')
        if self.future is not None and self.future[1].done():
            mode, future = self.future
            self.future = None
            try:
                response = future.result()
                if response.success and mode == self.desired:
                    self.confirmed = mode
                    self.logic.enabled = mode == 'MANUAL' and problem is None
                    print(f'Режим ведущего: {mode}', flush=True)
                elif not response.success:
                    print(f'Режим не принят: {response.message}', flush=True)
            except Exception as error:
                print(f'Ошибка запроса режима: {error}', flush=True)
        if (self.future is None and self.confirmed != self.desired
                and wall - self.last_request >= 1.
                and (self.desired == 'HOLD' or problem is None)):
            client = self.clients[self.desired]
            if client.service_is_ready():
                self.future = self.desired, client.call_async(self.Trigger.Request())
                self.last_request = wall
        if (self.confirmed == 'MANUAL' and problem is None
                and wall - self.last_publish >= .05):
            self.publish(self.logic.velocity(wall))
            self.last_publish = wall

    def shutdown(self, executor):
        self.publish((0., 0., 0., 0.))
        self.select('HOLD')
        self.last_request = -math.inf
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline and self.confirmed != 'HOLD':
            executor.spin_once(timeout_sec=.02)
            self.tick(time.monotonic())
        if self.confirmed != 'HOLD':
            # Даже если предшествующий MANUAL завис, отправляем заключительный HOLD.
            client = self.clients['HOLD']
            if client.service_is_ready():
                client.call_async(self.Trigger.Request())
                executor.spin_once(timeout_sec=.05)
            print('Ответ HOLD не получен; проверьте состояние ведущего.', flush=True)


HELP = '''Клавиатура ведущего, координаты карты ENU:
W/S: +X/-X   A/D: +Y/-Y   R/F: вверх/вниз   Q/E: поворот влево/вправо
1/2/3: 0.5/1/1.5 м/с   Space: HOLD   Enter: MANUAL   Esc: выход
Без нового символа 0.25 с запрашивается нулевая скорость. Работает автоповтор.
Начальная скорость 0.5 м/с. Нужен взлетевший ведущий в Offboard выше 1 м.
'''


def main(args=None):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.signals import SignalHandlerOptions

    # Обработчики Python оставляют ROS живым до запроса HOLD и восстановления TTY.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('keyboard_control', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    node.set_parameters([Parameter('use_sim_time', value=True)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    controller = None
    old_handlers = {}

    def stop(signum, frame):
        raise KeyboardInterrupt

    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        old_handlers[signum] = signal.signal(signum, stop)
    try:
        with terminal_input(sys.stdin) as descriptor:
            controller = KeyboardController(node)
            print(HELP, flush=True)
            running = True
            while running:
                executor.spin_once(timeout_sec=.02)
                wall = time.monotonic()
                if select.select([descriptor], [], [], 0)[0]:
                    data = os.read(descriptor, 64)
                    if not data:
                        break
                    # Esc завершает сеанс, включая escape-последовательности стрелок.
                    for key in data.decode('ascii', errors='ignore'):
                        action = controller.logic.key(key, wall)
                        if action == 'EXIT':
                            running = False
                            break
                        if action is not None:
                            controller.publish((0., 0., 0., 0.))
                            controller.select(action)
                controller.tick(wall)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if controller is not None:
                controller.shutdown(executor)
        finally:
            for signum, previous in old_handlers.items():
                signal.signal(signum, previous)
            node.destroy_node()
            executor.shutdown()
            if rclpy.ok():
                rclpy.shutdown()
