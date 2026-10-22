"""Ориентация лидера для трекера не проходит через моделируемый канал mesh."""

from pathlib import Path

import rclpy
import yaml

from lidar_tracker.timing import OrientationHistory
from lidar_tracker.tracker_node import LidarTracker

SRC = Path(__file__).resolve().parents[2]


def parameters(package, name):
    config = yaml.safe_load((SRC / package / 'config' / name).read_text())
    return next(iter(config.values()))['ros__parameters']


def test_tracker_reads_local_leader_states_not_mesh_output():
    tracker = parameters('lidar_tracker', 'tracker.yaml')
    mesh = parameters('mesh_interface', 'mesh.yaml')
    assert tracker['orientation_topic'] == mesh['states_in'] == '/swarm/states'
    assert tracker['orientation_topic'] != mesh['states_out']


def test_node_default_and_yaml_subscribe_to_same_topic():
    rclpy.init(args=['--ros-args', '--params-file',
                     str(SRC / 'lidar_tracker/config/tracker.yaml')])
    try:
        node = LidarTracker()
        topics = [subscription.topic_name for subscription in node.subscriptions]
        assert '/swarm/states' in topics and '/mesh/states' not in topics
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_mesh_sized_delay_would_leave_scan_without_orientation():
    """Почему важен источник: задержка 0.1 с выводит скан за допуск истории."""
    history = OrientationHistory()
    for step in range(20):
        history.add(step * .05, [0., 0., 0., 1.])
    history.at(.95 + .05)
    delayed = OrientationHistory()
    for step in range(20):
        if step * .05 + .1 <= 1.:
            delayed.add(step * .05, [0., 0., 0., 1.])
    try:
        delayed.at(1.)
    except ValueError:
        return
    raise AssertionError('Задержанная история не должна покрывать время скана')
