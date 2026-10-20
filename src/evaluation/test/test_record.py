import csv
import json

from geometry_msgs.msg import PoseArray, Pose, TransformStamped
import pytest
import rclpy
from swarm_interfaces.msg import RelativePose, SwarmState
from tf2_msgs.msg import TFMessage
from rclpy.parameter import Parameter
from std_msgs.msg import String

from evaluation.record import Recorder


@pytest.fixture
def recorder(tmp_path):
    rclpy.init()
    node = Recorder(tmp_path)
    node.set_parameters([Parameter('sensor_z', value=.45)])
    node.p['sensor_z'] = .45
    yield node, tmp_path
    node.finish()
    node.destroy_node()
    rclpy.shutdown()


def test_synthetic_gt_and_invalid_share(recorder):
    node, output = recorder
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for i in range(4):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.child_frame_id = f'uav_{i}'
        transform.transform.translation.x = float(-i * 2)
        transform.transform.rotation.w = 1.
        transforms.append(transform)
    # Вложенный frame не заменяет world pose модели.
    duplicate = TransformStamped()
    duplicate.header.stamp = stamp
    duplicate.child_frame_id = 'uav_1/base_link'
    duplicate.transform.translation.x = 900.
    transforms.append(duplicate)
    node.on_gt(TFMessage(transforms=transforms))
    offsets = PoseArray()
    offsets.header.stamp = stamp
    offsets.header.frame_id = 'uav_0/heading'
    for i in range(1, 4):
        pose = Pose()
        pose.position.x = float(-i * 2)
        offsets.poses.append(pose)
        relative = RelativePose()
        relative.target_id = i
        relative.measurement_stamp = stamp
        relative.frame_id = 'uav_0/lidar_link'
        relative.valid = True
        relative.pose.position.x = float(-i * 2)
        node.on_relative(relative)
    node.on_offsets(offsets)
    state = SwarmState()
    state.uav_id, state.valid, state.timestamp = 0, True, stamp
    node.on_state(state)
    node.tick()
    assert len(node.rows) == 3
    assert all(row['localization_error_m'] == pytest.approx(0., abs=1e-12) for row in node.rows)
    assert all(row['formation_error_m'] == 0. for row in node.rows)
    assert all(row['min_pair_distance_m'] == 2. for row in node.rows)
    relative.valid = False
    node.on_relative(relative)
    node.tick()
    assert node.rows[-1]['relative_stale'] == 1
    assert len(list(csv.DictReader((output / 'samples.csv').open()))) == 6


def test_missing_streams_and_json(recorder):
    node, output = recorder
    node.tick()
    assert all(row['gt_missing'] == row['relative_stale'] == 1 for row in node.rows)
    # finish вызывается fixture; JSON проверяется через статистику отдельно.
    from evaluation.metrics import summarize
    result = summarize(node.rows)
    assert result['relative_stale_share'] == 1.
    assert json.loads(json.dumps(result))['localization_error_m']['mean'] is None


def test_control_phases_exclude_takeoff_hold_and_stale_status(recorder):
    from evaluation.metrics import summarize
    node, _ = recorder
    node.tick()  # до включения контроллера
    now = node.get_clock().now().nanoseconds * 1e-9
    node.on_status(String(data=json.dumps(dict(time=now, scale=1., formation='wedge', mode='FORMATION_CHANGE'))))
    node.tick()
    node.on_status(String(data='invalid json'))
    now = node.get_clock().now().nanoseconds * 1e-9
    node.on_status(String(data=json.dumps(dict(time=now, scale=1., formation='wedge', mode='FOLLOWING'))))
    node.tick()
    node.control_status['time'] -= 10.
    node.tick()  # устаревший статус не засчитывается как работа контроллера
    result = summarize(node.rows)
    assert result['rows'] == 12
    assert result['active_control']['rows'] == 6
    assert result['following_by_formation']['wedge']['rows'] == 3
    assert result['following_by_formation']['column']['rows'] == 0
