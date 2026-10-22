"""Запуск и завершение только собственных процессов Gazebo/PX4."""
import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import urllib.request
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

def open_panel(url):
    """Открывает панель в браузере, когда она отвечает; True — больше не пытаться.

    В WSL нет xdg-open, но explorer.exe открывает адрес в браузере Windows.
    """
    try:
        urllib.request.urlopen(url, timeout=.3).close()
    except OSError:
        return False
    opener = shutil.which('wslview') or shutil.which('xdg-open') or shutil.which('explorer.exe')
    if opener:
        subprocess.Popen([opener, url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-dir', default=str(Path.home() / 'PX4-Autopilot'))
    parser.add_argument('--gui', choices=['true', 'false'], default='true')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--agent-port', type=int, default=8889)
    parser.add_argument('--lidar', choices=['true', 'false'], default='true')
    # Мир, мосты и трекер рассчитаны ровно на лидера и трёх ведомых.
    parser.add_argument('--uav-count', choices=['4'], default='4')
    parser.add_argument('--stack', choices=['true', 'false'], default='true')
    parser.add_argument('--video', choices=['true', 'false'], default='false')
    parser.add_argument('--web', choices=['true', 'false'], default='true')
    parser.add_argument('--web-port', type=int, default=8080)
    parser.add_argument('--open-browser', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    if args.video == 'true' and args.lidar != 'true':
        parser.error('Обзорная камера требует lidar:=true для системы датчиков мира')
    if args.web == 'true' and (args.lidar != 'true' or args.stack != 'true'):
        parser.error('Панель управления требует lidar:=true и stack:=true')
    # launch передаёт аргументы без shell, поэтому `~` не раскрывается
    # автоматически как в интерактивной оболочке.
    px4 = Path(args.px4_dir).expanduser().resolve()
    build = px4 / 'build/px4_sitl_default'
    if not (build / 'bin/px4').is_file():
        parser.error('Сначала соберите PX4: make px4_sitl')
    runtime = Path.cwd() / 'log' / ('sitl_' + time.strftime('%Y%m%d_%H%M%S'))
    runtime.mkdir(parents=True)
    env = os.environ.copy()
    env.update(PX4_GZ_MODELS=str(px4 / 'Tools/simulation/gz/models'),
               PX4_GZ_WORLDS=str(px4 / 'Tools/simulation/gz/worlds'),
               PX4_GZ_STANDALONE='1', PX4_GZ_WORLD='swarm',
               PX4_SYS_AUTOSTART='4001', PX4_SIM_MODEL='gz_x500',
               GZ_PARTITION='swarm_lidar', GZ_IP='127.0.0.1',
               ROS_LOG_DIR=str(runtime / 'ros'))
    models = px4 / 'Tools/simulation/gz/models'
    share = Path(get_package_share_directory('swarm_lidar'))
    env['GZ_SIM_RESOURCE_PATH'] = str(share / 'models') + ':' + str(models) + ':' + env.get('GZ_SIM_RESOURCE_PATH', '')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = str(build / 'src/modules/simulation/gz_plugins')
    processes, handles = [], []
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    def stop_signal(proc, signum, group=True):
        try:
            if group:
                os.killpg(proc.pid, signum)
            else:
                proc.send_signal(signum)
        except ProcessLookupError:
            pass  # Процесс мог завершиться между poll/wait и отправкой сигнала.
    def start(name, cmd, cwd=None, extra=None, stdin=False):
        log = open(runtime / (name + '.log'), 'w')
        handles.append(log)
        proc = subprocess.Popen(cmd, cwd=cwd, env=dict(env, **(extra or {})),
                                stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                start_new_session=True, text=True)
        processes.append((name, proc))
        return proc
    # Второй мир в той же партиции подменяет /clock уже идущего полёта:
    # часы откатываются, оценки PX4 расходятся. Отказываемся до запуска.
    try:
        services = subprocess.run(['gz', 'service', '-l'], env=env, capture_output=True,
                                  text=True, timeout=10).stdout
    except subprocess.TimeoutExpired:
        services = ''
    if any(line.startswith('/world/') for line in services.splitlines()):
        parser.error('В партиции Gazebo swarm_lidar уже работает мир. Остановите прежний запуск '
                     '(Ctrl+C в его терминале) и повторите.')
    try:
        world = share / 'worlds' / ('swarm_lidar.sdf' if args.lidar == 'true' else 'swarm.sdf')
        if args.video == 'true':
            document = ET.parse(world)
            include = ET.SubElement(document.getroot().find('world'), 'include')
            ET.SubElement(include, 'uri').text = 'model://spectator_camera'
            world = runtime / 'world.sdf'
            document.write(world, encoding='utf-8', xml_declaration=True)
        start('gazebo', ['gz', 'sim', '-r', '-s', '--headless-rendering', str(world)])
        agent = start('agent', ['MicroXRCEAgent', 'udp4', '-p', str(args.agent_port)])
        time.sleep(.2)
        if agent.poll() is not None:
            raise RuntimeError(
                f'MicroXRCEAgent не запустился на порту {args.agent_port}; '
                f'проверьте, не занят ли порт другим запуском. Лог: {runtime / "agent.log"}')
        if args.gui == 'true':
            start('gui', ['gz', 'sim', '-g'])
        # Ждём готовности world service вместо произвольной паузы.
        deadline = time.monotonic() + 45
        while time.monotonic() < deadline:
            info = subprocess.run(['gz', 'service', '-i', '-s', '/world/swarm/scene/info'],
                                  env=env, capture_output=True, text=True, timeout=5)
            if 'Service providers' in info.stdout:
                break
            time.sleep(.5)
        else:
            raise RuntimeError('Gazebo не готов за 45 секунд')
        if args.stack == 'true':
            start('ros_stack', ['ros2', 'launch', 'swarm_lidar', 'runtime.launch.py',
                                'lidar:=' + args.lidar])
        for i in range(4):
            working = runtime / f'uav_{i}'
            working.mkdir()
            startup = working / 'startup.sh'
            startup.write_text('. etc/init.d-posix/rcS\n'
                               'uxrce_dds_client stop\n'
                               'param set UXRCE_DDS_SYNCT 0\n'
                               f'uxrce_dds_client start -t udp -p {args.agent_port} -n uav_{i}\n')
            proc = start(f'px4_{i}', [str(build / 'bin/px4'), '-i', str(i),
                                    '-s', str(startup), '-w', str(working), str(build / 'etc')], cwd=working, stdin=True,
                         extra=dict(PX4_GZ_MODEL_NAME=f'uav_{i}',
                                    PX4_UXRCE_DDS_NS=f'uav_{i}',
                                    PX4_UXRCE_DDS_PORT=str(args.agent_port)))
        print(f'Четыре PX4 запущены. Логи: {runtime}', flush=True)
        if args.web == 'true':
            # Панель — из корня запуска: её журналы полётов в log/web.
            start('web_panel', [str(Path(sys.executable)), '-m', 'swarm_lidar.web_panel',
                                '--port', str(args.web_port)], cwd=Path.cwd())
            print(f'Панель управления: http://localhost:{args.web_port} '
                  '(дождитесь подключения четырёх аппаратов)', flush=True)
        deadline = time.monotonic() + args.duration if args.duration else float('inf')
        panel_reported = False
        browser_pending = args.web == 'true' and args.open_browser == 'true'
        while time.monotonic() < deadline:
            if browser_pending:
                browser_pending = not open_panel(f'http://localhost:{args.web_port}')
            for name, proc in processes:
                if proc.poll() is None:
                    continue
                if name == 'web_panel':
                    # Без панели симуляция остаётся рабочей; причина — в её журнале.
                    if not panel_reported:
                        print(f'Панель завершилась ({proc.returncode}); см. {runtime / "web_panel.log"}',
                              flush=True)
                        panel_reported = True
                    continue
                raise RuntimeError(f'{name} завершился: {proc.returncode}; см. {runtime}')
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        # Терминал и launch могут доставить SIGINT повторно во время очистки.
        # Дожидаемся собственных детей даже после второго сигнала.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # Сначала панель: при активном полёте она сажает группу, пока PX4 и
        # стек ещё работают. Только после неё останавливается остальное.
        for name, proc in processes:
            if name == 'web_panel' and proc.poll() is None:
                print('Остановка панели: при активном полёте сначала посадка…', flush=True)
                stop_signal(proc, signal.SIGINT)
                try:
                    proc.wait(timeout=150)
                except subprocess.TimeoutExpired:
                    stop_signal(proc, signal.SIGKILL)
                    proc.wait()
        for name, proc in reversed(processes):
            if proc.poll() is None:
                # ros2 launch сам передаёт сигнал узлам. Сигнал всей группе
                # приводил к повторному Ctrl+C во время destroy_node.
                if name == 'ros_stack':
                    stop_signal(proc, signal.SIGINT, group=False)
                else:
                    stop_signal(proc, signal.SIGINT)
        for _, proc in reversed(processes):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                stop_signal(proc, signal.SIGTERM)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    stop_signal(proc, signal.SIGKILL)
                    proc.wait()
        for handle in handles:
            handle.close()
        print(f'Процессы запуска остановлены. Логи: {runtime}', flush=True)

if __name__ == '__main__':
    main()

