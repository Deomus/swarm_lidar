"""Стартовая расстановка задаётся одним YAML и совпадает с мирами Gazebo."""

from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from swarm_lidar.start_geometry import load_origins, tracker_seeds

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config' / 'swarm.yaml'


def world_poses(name):
    world = ET.parse(PACKAGE / 'worlds' / name).getroot().find('world')
    return {include.findtext('name'): [float(v) for v in include.findtext('pose').split()]
            for include in world.findall('include')
            if (include.findtext('name') or '').startswith('uav_')}


@pytest.mark.parametrize('world', ['swarm_lidar.sdf'])
def test_world_models_start_at_yaml_origins(world):
    origins = load_origins(CONFIG)
    poses = world_poses(world)
    assert sorted(poses) == [f'uav_{i}' for i in range(4)]
    heights = set()
    for i, origin in enumerate(origins):
        pose = poses[f'uav_{i}']
        assert pose[:2] == origin[:2] and pose[3:] == [0.] * 3
        heights.add(pose[2] - origin[2])
    assert len(heights) == 1


def test_tracker_seeds_are_followers_in_lidar_frame():
    tracker = yaml.safe_load((PACKAGE.parent / 'lidar_tracker/config/tracker.yaml').read_text())
    offset = tracker['lidar_tracker']['ros__parameters']['sensor_offset']
    assert tracker_seeds(load_origins(CONFIG), offset) == [
        -4., -4., -1.9, -4., 0., -1.9, -4., 4., -1.9]
    assert 'initial_positions' not in tracker['lidar_tracker']['ros__parameters']


@pytest.mark.parametrize('origins', [
    [[0., 0., 0.]] * 3, [[0., 0.]] * 4, [[0., 0., float('nan')]] * 4, 'line'])
def test_invalid_origins_are_rejected(tmp_path, origins):
    path = tmp_path / 'swarm.yaml'
    path.write_text(yaml.safe_dump({'uav_origins_enu': origins}))
    with pytest.raises(ValueError):
        load_origins(path)
