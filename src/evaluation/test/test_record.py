import csv
import json

from geometry_msgs.msg import PoseArray, Pose, TransformStamped
import pytest
import rclpy
from swarm_interfaces.msg import RelativePose, SwarmState
from tf2_msgs.msg import TFMessage
from rclpy.parameter import Parameter
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from evaluation.record import Recorder


@pytest.fixture
def recorder(tmp_path):
    rclpy.init()
    node = Recorder(tmp_path)
    node.set_parameters([Parameter('sensor_z', value=0.)])
    node.p['sensor_z'] = 0.
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
    metrics = []
    node.metrics_pub = type('Sink', (), dict(publish=lambda self, m: metrics.append(json.loads(m.data))))()
    node.tick()
    assert len(node.rows) == 3
    assert metrics[-1]['min_pair_distance_m'] == 2.
    assert metrics[-1]['targets']['2']['formation_error_m'] == 0. and metrics[-1]['targets']['2']['relative_stale'] == 0
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


def test_formation_metric_uses_offsets_at_truth_time(recorder):
    node, _ = recorder
    from evaluation.metrics import Pose as MetricPose
    node.gt.clear()
    node.gt.append((9.86, {i: MetricPose((float(-i*2)+.86, 0., 0.), (0., 0., 0., 1.))
                          for i in range(4)}))
    for stamp, shift in [(9.8, 0.), (10., .2)]:
        offsets = PoseArray()
        offsets.header.frame_id = 'uav_0/heading'
        offsets.header.stamp.sec = int(stamp)
        offsets.header.stamp.nanosec = round((stamp-int(stamp))*1e9)
        for i in range(1, 4):
            p = Pose()
            p.position.x = float(-i*2)+shift
            offsets.poses.append(p)
        node.on_offsets(offsets)
    # За 0,06 с после первого задания каждый ведомый сместился на 0,06 м.
    leader = node.gt[0][1][0]
    for i in range(1, 4):
        node.gt[0][1][i] = MetricPose((leader.position[0]-i*2+.06, 0., 0.), (0., 0., 0., 1.))
    desired = node.offsets_at(9.86)
    assert desired[0][0] == pytest.approx(-1.94)
    from evaluation.metrics import relative_heading
    actual = relative_heading(node.gt[0][1][0], node.gt[0][1][1])
    assert abs(actual[0]-desired[0][0]) < 1e-10


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


def test_route_error_uses_truth_and_empty_path_revokes_measurement(recorder):
    node, _ = recorder
    stamp = node.get_clock().now().to_msg()
    transforms = []
    for index in range(4):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.child_frame_id = f'uav_{index}'
        transform.transform.translation.x = float(-index * 2)
        transform.transform.translation.y = 1.
        transform.transform.rotation.w = 1.
        transforms.append(transform)
    node.on_gt(TFMessage(transforms=transforms))
    path = Path()
    path.header.frame_id = 'map'
    for x in (0., 2.):
        point = PoseStamped()
        point.pose.position.x, point.pose.position.z = x, .24
        path.poses.append(point)
    node.on_path(path)
    node.on_mission_status(String(data=json.dumps(dict(stamp=stamp.sec+stamp.nanosec*1e-9, status='MISSION'))))
    node.tick()
    assert all(row['route_error_m'] == pytest.approx(1.) for row in node.rows)
    assert all(row['goal_error_m'] == pytest.approx(5**.5) for row in node.rows)
    path.poses = []
    node.on_path(path)
    node.tick()
    assert all(row['mission_active'] == 0 for row in list(node.rows)[-3:])
