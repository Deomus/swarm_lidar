import pytest

from mission_planner.grid import PlanningError, astar, inflated_grid, segment_cells, shortcut_path


def test_diagonal_path():
    assert astar(4, 4, set(), (0, 0), (3, 3)) == [(0, 0), (1, 1), (2, 2), (3, 3)]


def test_no_corner_cutting():
    with pytest.raises(PlanningError, match='недостижима'):
        astar(2, 2, {(1, 0), (0, 1)}, (0, 0), (1, 1))


def test_wall_detour():
    blocked = {(2, y) for y in range(4)}
    path = astar(5, 5, blocked, (0, 0), (4, 0))
    assert (2, 4) in path
    assert not set(path) & blocked


def test_inflation_unknown_and_border():
    data = [0] * 49
    data[3 * 7 + 3] = -1
    blocked = inflated_grid(7, 7, 1., data, 1.)
    assert {(3, 3), (2, 3), (4, 3), (3, 2), (3, 4), (0, 0)} <= blocked
    assert (1, 1) not in blocked


@pytest.mark.parametrize('start,goal,reason', [((-1, 0), (2, 2), 'границами'),
                                             ((0, 0), (3, 2), 'границами'),
                                             ((1, 1), (2, 2), 'препятствии'),
                                             ((0, 0), (1, 1), 'препятствии')])
def test_invalid_endpoints(start, goal, reason):
    with pytest.raises(PlanningError, match=reason):
        astar(3, 3, {(1, 1)}, start, goal)


def test_invalid_map():
    with pytest.raises(PlanningError):
        inflated_grid(3, 3, 0., [0] * 9, 1.)
    with pytest.raises(PlanningError):
        inflated_grid(3, 3, 1., [0] * 8, 1.)


def test_segment_through_grid_node_counts_both_side_cells():
    assert {(1, 0), (0, 1)} <= segment_cells((.5, .5), (1.5, 1.5))
    assert segment_cells((.2, .5), (2.7, .5)) == {(0, 0), (1, 0), (2, 0)}


def test_open_map_staircase_becomes_single_segment():
    # Цель из браузерного полёта: A* дал 21 точку с поворотами через метр.
    blocked = inflated_grid(200, 200, 1., [0] * 40000, 6.75)
    start, goal = (100.04, 100.06), (114.85, 120.33)
    cells = astar(200, 200, blocked, (100, 100), (114, 120))
    points = shortcut_path([start, *[(x + .5, y + .5) for x, y in cells], goal], 200, 200, blocked)
    assert len(cells) > 10 and points == [start, goal]


def test_shortcut_keeps_detour_corners_clear_of_wall():
    blocked = {(2, y) for y in range(4)}
    cells = astar(5, 5, blocked, (0, 0), (4, 0))
    points = shortcut_path([(x + .5, y + .5) for x, y in cells], 5, 5, blocked)
    assert 2 < len(points) < len(cells)
    for a, b in zip(points, points[1:]):
        assert not segment_cells(a, b) & blocked
        assert all(0 <= x < 5 and 0 <= y < 5 for x, y in segment_cells(a, b))
