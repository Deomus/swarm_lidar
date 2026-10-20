"""Проверки расстояния до трёхмерной ломаной без симулятора."""

import math

import numpy as np
import pytest
from evaluation.route_metrics import distance_to_path


@pytest.mark.parametrize('point,expected', [
    ((2., 0., 0.), 0.), ((2., 3., 0.), 3.),
    ((-2., 0., 0.), 2.), ((6., 0., 0.), 2.),
    ((6., 3., 0.), math.sqrt(13)),
])
def test_segment_projection_and_ends(point, expected):
    assert distance_to_path(point, [(0., 0., 0.), (4., 0., 0.)]) == pytest.approx(expected)


def test_turn_uses_nearest_segment_not_nearest_vertex():
    path = [(0., 0., 0.), (4., 0., 0.), (4., 4., 0.)]
    assert distance_to_path((3., 2., 0.), path) == pytest.approx(1.)
    assert distance_to_path((2., 1., 0.), path) == pytest.approx(1.)


def test_height_and_diagonal_3d_projection():
    assert distance_to_path((2., 0., 3.), [(0., 0., 0.), (4., 0., 0.)]) == pytest.approx(3.)
    assert distance_to_path((1., 2., 1.), [(0., 0., 0.), (2., 2., 2.)]) == pytest.approx(
        math.sqrt(2. / 3.))
    assert distance_to_path((0., 0., 5.), [(0., 0., 1.), (0., 0., 3.)]) == pytest.approx(2.)


def test_single_point_and_repeated_vertices():
    assert distance_to_path((1., 2., 2.), [(0., 0., 0.)]) == pytest.approx(3.)
    assert distance_to_path((1., 2., 2.), [(0., 0., 0.)] * 3) == pytest.approx(3.)
    assert distance_to_path((1., 1., 0.), [(0., 0., 0.), (0., 0., 0.),
                                         (2., 0., 0.), (2., 0., 0.)]) == pytest.approx(1.)
    assert distance_to_path((0., 0., 0.), [(0., 0., 0.)] * 2) == 0.


def test_numpy_inputs_remain_unchanged_and_result_is_float():
    point = np.array([1., 2., 3.])
    path = np.array([[0., 0., 0.], [4., 0., 0.]])
    point_copy, path_copy = point.copy(), path.copy()
    result = distance_to_path(point, path)
    assert isinstance(result, float)
    assert result == pytest.approx(math.sqrt(13.))
    np.testing.assert_array_equal(point, point_copy)
    np.testing.assert_array_equal(path, path_copy)


@pytest.mark.parametrize('scale', [1e-200, 1e200])
def test_large_and_small_coordinate_scales(scale):
    actual = distance_to_path(np.array([1., 2., 0.]) * scale,
                              np.array([[0., 0., 0.], [2., 0., 0.]]) * scale)
    assert actual / scale == pytest.approx(2.)


@pytest.mark.parametrize('point,path', [
    ((0., 0., 0.), []), ((0., 0., 0.), [0., 0., 0.]),
    ((0., 0., 0.), [(0., 0.)]), ((0., 0.), [(0., 0., 0.)]),
    (((0., 0., 0.),), [(0., 0., 0.)]),
    ((float('nan'), 0., 0.), [(0., 0., 0.)]),
    ((0., 0., 0.), [(0., float('inf'), 0.)]),
    ((0., 0., 0.), [(0., 0., 0.), (0., 0.)]),
    (('не число', 0., 0.), [(0., 0., 0.)]),
])
def test_invalid_values_raise_value_error(point, path):
    with pytest.raises(ValueError):
        distance_to_path(point, path)
