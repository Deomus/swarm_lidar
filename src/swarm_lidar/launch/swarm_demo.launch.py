import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_dir', default_value=os.path.expanduser('~/PX4-Autopilot')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('duration', default_value='0'),
        ExecuteProcess(cmd=['ros2', 'run', 'swarm_lidar', 'sitl',
                            '--px4-dir', LaunchConfiguration('px4_dir'),
                            '--gui', LaunchConfiguration('gui'),
                            '--duration', LaunchConfiguration('duration')],
                       output='screen'),
    ])

