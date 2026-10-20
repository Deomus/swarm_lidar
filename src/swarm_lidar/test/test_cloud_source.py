"""Организованное облако, padding, endian и точное сохранение измерений."""

import struct

from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
import numpy as np
import pytest
from sensor_msgs_py import point_cloud2

from swarm_lidar.cloud_source import compact_cloud, load_filter_config


CONFIG = dict(min_range=.5, max_range=80., min_z=-2.9, max_z=-1., expected_frame='uav_0/lidar_link')


def cloud(points, width=2, big=False):
    message = PointCloudPacked(width=width, height=len(points)//width, point_step=20,
                               row_step=width*20+8, is_bigendian=big)
    message.header.stamp.sec, message.header.stamp.nsec = 42, 123456789
    message.header.data.add(key='frame_id', value=['uav_0/lidar_link'])
    for name, offset in [('x', 4), ('y', 8), ('z', 12)]:
        message.field.add(name=name, offset=offset, datatype=PointCloudPacked.Field.FLOAT32, count=1)
    data = bytearray(message.row_step*message.height)
    for index, point in enumerate(points):
        row, col = divmod(index, width)
        struct.pack_into(('>' if big else '<')+'fff', data,
                         row*message.row_step+col*message.point_step+4, *point)
    message.data = bytes(data)
    return message


@pytest.mark.parametrize('big', [False, True])
def test_organized_padding_and_endian_preserve_exact_xyz_and_source_stamp(big):
    points = [[-4., -4., -1.9], [1., 2., -2.], [4., 5., -1.], [-3., -2., -2.9]]
    message = compact_cloud(cloud(points, big=big), CONFIG)
    result = point_cloud2.read_points_numpy(message, field_names=('x', 'y', 'z'))
    np.testing.assert_array_equal(result, np.asarray(points, np.float32))
    assert message.height == 1 and message.width == 4
    assert message.point_step == 12 and message.row_step == 48
    assert message.header.stamp.sec == 42 and message.header.stamp.nanosec == 123456789
    assert message.header.frame_id == CONFIG['expected_frame']
    assert message.is_dense and not message.is_bigendian


def test_nonfinite_and_outside_points_are_removed_without_invented_replacements():
    points = [[-4., 0., -1.9], [float('nan'), 0., -1.9], [0., float('inf'), -1.9],
              [81., 0., -2.], [4., 0., -.9], [4., 0., -3.]]
    output = compact_cloud(cloud(points), CONFIG)
    assert output.width == 1
    np.testing.assert_array_equal(point_cloud2.read_points_numpy(output), np.array([points[0]], np.float32))


def test_all_removed_still_publishes_empty_measurement_with_original_stamp():
    output = compact_cloud(cloud([[1., 0., 0.], [float('inf'), 0., 0.]]), CONFIG)
    assert output.width == 0 and output.row_step == 0 and bytes(output.data) == b''
    assert output.header.stamp.sec == 42


@pytest.mark.parametrize('mutation', ['short_data', 'short_row', 'short_step', 'missing_xyz',
                                      'duplicate_xyz', 'overlap', 'bad_type', 'bad_count',
                                      'offset', 'frame', 'timestamp'])
def test_invalid_layout_is_rejected(mutation):
    message = cloud([[-4., 0., -1.9]]*2)
    if mutation == 'short_data':
        message.data = message.data[:-1]
    elif mutation == 'short_row':
        message.row_step = 1
    elif mutation == 'short_step':
        message.point_step = 1
    elif mutation == 'missing_xyz':
        del message.field[-1]
    elif mutation == 'duplicate_xyz':
        message.field.add().CopyFrom(message.field[0])
    elif mutation == 'overlap':
        message.field[1].offset = 5
    elif mutation == 'bad_type':
        message.field[0].datatype = PointCloudPacked.Field.FLOAT64
    elif mutation == 'bad_count':
        message.field[0].count = 2
    elif mutation == 'offset':
        message.field[0].offset = 19
    elif mutation == 'frame':
        message.header.data[0].value[0] = 'world'
    else:
        message.header.stamp.nsec = 1_000_000_000
    with pytest.raises(ValueError):
        compact_cloud(message, CONFIG)


def test_limits_are_loaded_from_tracker_yaml_without_second_copy(tmp_path):
    config = tmp_path/'tracker.yaml'
    config.write_text('lidar_tracker:\n  ros__parameters:\n    min_range: 1.0\n    max_range: 9.0\n'
                      '    min_z: -3.0\n    max_z: -1.5\n    expected_frame: sensor\n'
                      '    input_topic: /points\n')
    result = load_filter_config(config)
    assert result['min_range'] == 1. and result['max_range'] == 9.
    assert result['expected_frame'] == 'sensor' and result['input_topic'] == '/points'


def test_distance_and_height_boundaries_are_inclusive_like_tracker():
    config = dict(CONFIG, min_range=2., max_range=3., min_z=-3., max_z=0.)
    points = [[2., 0., 0.], [3., 0., 0.], [1.9999, 0., 0.], [3.0001, 0., 0.],
              [0., 0., -3.], [0., 0., -3.0001]]
    output = compact_cloud(cloud(points), config)
    np.testing.assert_array_equal(point_cloud2.read_points_numpy(output),
                                  np.array([points[0], points[1], points[4]], np.float32))
