from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
import yaml

from swarm_lidar.start_geometry import load_origins, tracker_seeds


def runtime(context):
    share = Path(get_package_share_directory('swarm_lidar'))
    config = share / 'config' / 'lidar_bridge.yaml'
    nodes = [Node(package='ros_gz_bridge', executable='parameter_bridge',
                  parameters=[{'config_file': str(config)}], output='screen')]
    bridge_config = str(Path(get_package_share_directory('autopilot_bridge')) / 'config/bridge.yaml')
    origins = load_origins(share / 'config' / 'swarm.yaml')
    for i, origin in enumerate(origins):
        # Команда своему PX4 приходит локально: ведущему от leader_interface,
        # ведомому от его контроллера строя. По mesh идут позы, состояния,
        # команда ведущего (упреждение) и план строя.
        commands = '/swarm/commands' if i == 0 else f'/uav_{i}/command'
        nodes.append(Node(package='autopilot_bridge', executable='bridge_node', namespace=f'uav_{i}',
                          parameters=[bridge_config, {'uav_id': i, 'origin_enu': origin,
                                                     'commands_topic': commands,
                                                     'use_sim_time': True}], output='screen'))
    # Программная GCS посылает настоящий MAVLink heartbeat, preflight не отключается.
    nodes.append(Node(package='autopilot_bridge', executable='gcs_heartbeat', output='screen'))
    for package, executable, config_name in [('mesh_interface', 'mesh_node', 'mesh.yaml'),
                                              ('swarm_manager', 'manager_node', 'manager.yaml')]:
        path = Path(get_package_share_directory(package)) / 'config' / config_name
        nodes.append(Node(package=package, executable=executable,
                          parameters=[str(path), {'use_sim_time': True}], output='screen'))
    nodes.append(Node(package='swarm_lidar', executable='training_map',
                      parameters=[{'use_sim_time': True}], output='screen'))
    tracker_config = str(Path(get_package_share_directory('lidar_tracker')) / 'config/tracker.yaml')
    nodes.append(Node(package='swarm_lidar', executable='cloud_source',
                      parameters=[{'filter_config': tracker_config, 'use_sim_time': True}],
                      output='screen'))
    # Пассивная оценка не является обязательным элементом управления.
    evaluation_node = Node(package='evaluation', executable='ground_truth_node',
                           parameters=[{'use_sim_time': True}], output='screen')
    path = Path(get_package_share_directory('lidar_tracker')) / 'config/tracker.yaml'
    tracker = yaml.safe_load(path.read_text())['lidar_tracker']['ros__parameters']
    seeds = tracker_seeds(origins, tracker['sensor_offset'])
    nodes.append(Node(package='lidar_tracker', executable='tracker_node',
                      parameters=[str(path), {'initial_positions': seeds, 'use_sim_time': True}],
                      output='screen'))
    formation_config = Path(get_package_share_directory('formation_controller')) / 'config'
    nodes.append(Node(package='formation_controller', executable='coordinator_node',
                      parameters=[str(formation_config / 'coordinator.yaml'), {'use_sim_time': True}],
                      output='screen'))
    for i in (1, 2, 3):
        nodes.append(Node(package='formation_controller', executable='controller_node', namespace=f'uav_{i}',
                          parameters=[str(formation_config / 'controller.yaml'),
                                      {'uav_id': i, 'use_sim_time': True}], output='screen'))
    for package, executable, config_name in [('leader_interface', 'leader_node', 'leader.yaml'),
                                              ('mission_planner', 'planner_node', 'mission.yaml')]:
        path = Path(get_package_share_directory(package)) / 'config' / config_name
        nodes.append(Node(package=package, executable=executable,
                          parameters=[str(path), {'use_sim_time': True}], output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=lambda event, ctx: [] if ctx.is_shutdown else [
            EmitEvent(event=Shutdown(reason='Один из обязательных ROS-узлов завершился'))]))
        for node in nodes]
    return handlers + nodes + [evaluation_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=runtime)])
