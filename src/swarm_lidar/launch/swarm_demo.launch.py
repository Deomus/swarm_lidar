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
        DeclareLaunchArgument('uav_count', default_value='4',
                              description='Число аппаратов; поддерживается только 4'),
        ExecuteProcess(cmd=[str(Path(get_package_prefix('swarm_lidar')) / 'lib/swarm_lidar/sitl'),
                            '--px4-dir', LaunchConfiguration('px4_dir'),
                            '--gui', LaunchConfiguration('gui'),
                            '--duration', LaunchConfiguration('duration'),
                            '--lidar', LaunchConfiguration('lidar'),
                            '--stack', LaunchConfiguration('stack'),
                            '--video', LaunchConfiguration('video'),
                            '--uav-count', LaunchConfiguration('uav_count')],
                       output='screen', sigterm_timeout='30', sigkill_timeout='10'),
    ])

