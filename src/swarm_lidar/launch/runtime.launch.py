from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def runtime(context):
    lidar = LaunchConfiguration('lidar').perform(context) == 'true'
    share = Path(get_package_share_directory('swarm_lidar'))
    config = share / 'config' / ('lidar_bridge.yaml' if lidar else 'clock_bridge.yaml')
    nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': str(config)}], output='screen')]
    bridge_config = str(Path(get_package_share_directory('autopilot_bridge')) / 'config/bridge.yaml')
    origins = [[0., 0., 0.], [-4., -4., 0.], [-4., 0., 0.], [-4., 4., 0.]]
    for i, origin in enumerate(origins):
        nodes.append(Node(package='autopilot_bridge', executable='bridge_node', namespace=f'uav_{i}',
                          parameters=[bridge_config, {'uav_id': i, 'origin_enu': origin,
                                                     'use_sim_time': True}], output='screen'))
    # Программная GCS посылает настоящий MAVLink heartbeat, preflight не отключается.
    nodes.append(Node(package='autopilot_bridge', executable='gcs_heartbeat', output='screen'))
    for package, executable, config_name in [('mesh_interface', 'mesh_node', 'mesh.yaml'),
                                              ('swarm_manager', 'manager_node', 'manager.yaml')]:
        path = Path(get_package_share_directory(package)) / 'config' / config_name
        nodes.append(Node(package=package, executable=executable,
                          parameters=[str(path), {'use_sim_time': True}], output='screen'))
    if lidar:
        nodes.append(Node(package='evaluation', executable='ground_truth_node',
                          parameters=[{'use_sim_time': True}], output='screen'))
        path = Path(get_package_share_directory('lidar_tracker')) / 'config/tracker.yaml'
        nodes.append(Node(package='lidar_tracker', executable='tracker_node',
                          parameters=[str(path), {'use_sim_time': True}], output='screen'))
        for package, executable, config_name in [('formation_controller', 'controller_node', 'controller.yaml'),
                                                  ('leader_interface', 'leader_node', 'leader.yaml'),
                                                  ('mission_planner', 'planner_node', 'mission.yaml')]:
            path = Path(get_package_share_directory(package)) / 'config' / config_name
            nodes.append(Node(package=package, executable=executable,
                              parameters=[str(path), {'use_sim_time': True}], output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=lambda event, ctx: [] if ctx.is_shutdown else [
            EmitEvent(event=Shutdown(reason='Один из обязательных ROS-узлов завершился'))]))
        for node in nodes]
    return handlers + nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('lidar', default_value='false'),
                              OpaqueFunction(function=runtime)])
