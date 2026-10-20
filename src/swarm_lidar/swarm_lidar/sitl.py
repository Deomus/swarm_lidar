"""Запуск и завершение только собственных процессов Gazebo/PX4."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

from ament_index_python.packages import get_package_share_directory

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--px4-dir', default=str(Path.home() / 'PX4-Autopilot'))
    parser.add_argument('--gui', choices=['true', 'false'], default='true')
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--agent-port', type=int, default=8889)
    parser.add_argument('--lidar', choices=['true', 'false'], default='false')
    parser.add_argument('--stack', choices=['true', 'false'], default='true')
    args = parser.parse_args()
    px4 = Path(args.px4_dir).resolve()
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
    def start(name, cmd, cwd=None, extra=None, stdin=False):
        log = open(runtime / (name + '.log'), 'w')
        handles.append(log)
        proc = subprocess.Popen(cmd, cwd=cwd, env=dict(env, **(extra or {})),
                                stdout=log, stderr=subprocess.STDOUT,
                                stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                start_new_session=True, text=True)
        processes.append((name, proc))
        return proc
    try:
        world = share / 'worlds' / ('swarm_lidar.sdf' if args.lidar == 'true' else 'swarm.sdf')
        start('gazebo', ['gz', 'sim', '-r', '-s', '--headless-rendering', str(world)])
        start('agent', ['MicroXRCEAgent', 'udp4', '-p', str(args.agent_port)])
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
        deadline = time.monotonic() + args.duration if args.duration else float('inf')
        while time.monotonic() < deadline:
            for name, proc in processes:
                if proc.poll() is not None:
                    raise RuntimeError(f'{name} завершился: {proc.returncode}; см. {runtime}')
            time.sleep(.5)
    except KeyboardInterrupt:
        pass
    finally:
        for _, proc in reversed(processes):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
        for _, proc in reversed(processes):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait()
        for handle in handles:
            handle.close()
        print(f'Процессы запуска остановлены. Логи: {runtime}', flush=True)

if __name__ == '__main__':
    main()

