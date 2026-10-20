from math import pi, sin, cos

import numpy as np
import pytest

from evaluation.metrics import (Pose, min_pair_distance, model_id, model_to_base, nearest,
                                relative_heading, relative_lidar, scalar_stats)


def pose(x, y, z=0., yaw=0.):
    return Pose((x, y, z), (0., 0., sin(yaw / 2), cos(yaw / 2)))


def test_relative_sensor_marker_offsets():
    assert np.allclose(relative_lidar(pose(0, 0), pose(-4, 2)), [-4, 2, 0])
    assert np.allclose(relative_lidar(pose(0, 0), pose(-4, 2), marker_offset=(0, 0, 0)), [-4, 2, -.45])


def test_yaw_and_heading():
    leader, target = pose(10, 20, yaw=pi / 2), pose(10, 24)
    assert np.allclose(relative_lidar(leader, target), [4, 0, 0], atol=1e-12)
    assert np.allclose(relative_heading(leader, target), [4, 0, 0], atol=1e-12)


def test_tilted_marker():
    leader = pose(0, 0)
    target = Pose((2, 0, 0), (sin(pi / 4), 0., 0., cos(pi / 4)))
    assert np.allclose(relative_lidar(leader, target), [2, -.45, -.45])


def test_model_root_offset_rotates_with_tilt():
    root = Pose((2, 0, 0), (sin(pi / 4), 0., 0., cos(pi / 4)))
    base = model_to_base(root)
    assert np.allclose(base.position, [2, -.24, 0], atol=1e-12)
    leader = model_to_base(pose(0, 0))
    assert np.allclose(relative_lidar(leader, base), [2, -.69, -.69])
    # Base-origin minpair differs from root-origin minpair under unequal tilt.
    poses = {0: leader, 1: base, 2: model_to_base(pose(10, 0)),
             3: model_to_base(pose(20, 0))}
    assert min_pair_distance(poses) == pytest.approx((4 + 2 * .24 ** 2) ** .5)


def test_pair_all_four():
    poses = {i: pose(i * 2, 0) for i in range(4)}
    assert min_pair_distance(poses) == 2.
    del poses[3]
    assert np.isnan(min_pair_distance(poses))


def test_timestamp_match():
    samples = [(1., 'a'), (1.2, 'b')]
    assert nearest(samples, 1.16, .1) == samples[1]
    assert nearest(samples, 1.5, .1) is None


def test_scoped_names_ignore_link_duplicates():
    assert model_id('uav_0') == (0, 0)
    assert model_id('swarm::uav_3') == (3, 1)
    assert model_id('/world/swarm/uav_1') == (1, 1)
    assert model_id('uav_0/base_link') is None
    assert model_id('uav_4') is None


def test_stats_rms_and_missing():
    stats = scalar_stats([3., 4., float('nan')])
    assert stats['mean'] == 3.5
    assert stats['rms'] == pytest.approx((12.5) ** .5)
    assert stats['max'] == 4.
    assert scalar_stats([])['rms'] is None
