import pytest

from mission_planner.grid import PlanningError, astar, inflated_grid


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
