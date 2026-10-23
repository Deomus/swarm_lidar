import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('duration', default_value='0'),
        # Полный контур LiDAR → строй → PX4 по умолчанию: одна команда по ТЗ §12.
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('stack', default_value='true'),
        DeclareLaunchArgument('video', default_value='false'),
        DeclareLaunchArgument('web', default_value='true',
                              description='Браузерная панель управления на localhost'),
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('open_browser', default_value='true',
                              description='Открыть панель в браузере, когда она ответит'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz2: TF, облако LiDAR, карта, маршрут, места строя'),
        DeclareLaunchArgument('gpu', default_value='NVIDIA',
                              description='WSL: видеоадаптер OpenGL через D3D12; none — не трогать'),
        DeclareLaunchArgument('uav_count', default_value='4',
                              description='Число аппаратов; поддерживается только 4'),
        ExecuteProcess(cmd=[str(Path(get_package_prefix('swarm_lidar')) / 'lib/swarm_lidar/sitl'),
                            '--px4-dir', LaunchConfiguration('px4_dir'),
                            '--gui', LaunchConfiguration('gui'),
                            '--duration', LaunchConfiguration('duration'),
                            '--lidar', LaunchConfiguration('lidar'),
                            '--stack', LaunchConfiguration('stack'),
                            '--video', LaunchConfiguration('video'),
                            '--uav-count', LaunchConfiguration('uav_count'),
                            '--web', LaunchConfiguration('web'),
                            '--web-port', LaunchConfiguration('web_port'),
                            '--open-browser', LaunchConfiguration('open_browser'),
                            '--rviz', LaunchConfiguration('rviz'),
                            '--gpu', LaunchConfiguration('gpu')],
                       # Панель при Ctrl+C сначала сажает группу: до 150 с.
                       output='screen', sigterm_timeout='180', sigkill_timeout='10'),
    ])

