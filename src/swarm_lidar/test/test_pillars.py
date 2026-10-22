"""Столбы: один YAML, миры Gazebo и карта A* совпадают."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from mission_planner.grid import astar, inflated_grid, shortcut_path
from swarm_lidar.training_map import load_obstacles, training_map

PACKAGE = Path(__file__).resolve().parents[1]
CONFIG = PACKAGE / 'config' / 'obstacles.yaml'


@pytest.mark.parametrize('world', ['swarm.sdf', 'swarm_lidar.sdf'])
def test_worlds_place_pillars_from_yaml(world):
    pillars, radius = load_obstacles(CONFIG)
    root = ET.parse(PACKAGE / 'worlds' / world).getroot().find('world')
    placed = [tuple(float(v) for v in include.findtext('pose').split()[:2])
              for include in root.findall('include') if include.findtext('uri') == 'model://pillar']
    assert placed == pillars
    model = ET.parse(PACKAGE / 'models' / 'pillar' / 'model.sdf').getroot()
    assert float(model.find('.//collision/geometry/cylinder/radius').text) == radius


def test_route_east_goes_around_pillar_wall_with_formation_margin():
    pillars, radius = load_obstacles(CONFIG)
    grid = training_map(pillars, radius)
    assert grid.data[(100 + 4) * 200 + 100 + 18] == 100  # клетка столба (18; 4)
    assert grid.data[100 * 200 + 100 + 18] == 0  # промежуток стены без inflation
    blocked = inflated_grid(200, 200, 1., grid.data, 6.75)
    cells = astar(200, 200, blocked, (100, 100), (135, 100))
    points = shortcut_path([(100.5, 100.5), *[(x + .5, y + .5) for x, y in cells], (135.5, 100.5)],
                           200, 200, blocked)
    assert len(points) > 2  # прямой путь перекрыт, есть поворот
    for x, y in cells:
        assert min(math.hypot(x - 100 + .5 - px, y - 100 + .5 - py) for px, py in pillars) > 6.75
