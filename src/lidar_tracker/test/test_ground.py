import numpy as np

from lidar_tracker.tracking import remove_ground


def test_tilted_ground_is_removed_and_bodies_retained():
    rng = np.random.default_rng(42)
    xy = rng.uniform(-20, 20, (6000, 2))
    ground = np.column_stack([xy, -2.14 + .1 * xy[:, 0] - .15 * xy[:, 1]])
    bodies = rng.normal([-4., 0., -1.85], .06, (200, 3))
    result = remove_ground(np.vstack([ground, bodies]))
    np.testing.assert_allclose(result, bodies)


def test_small_coplanar_body_cloud_is_not_ground():
    rng = np.random.default_rng(42)
    bodies = rng.normal([-4., 0., -2.2], [.2, .2, .002], (300, 3))
    np.testing.assert_array_equal(remove_ground(bodies), bodies)


def test_wall_is_not_ground():
    rng = np.random.default_rng(42)
    wall = np.column_stack([np.full(3000, -4.), rng.uniform(-10, 10, (3000, 2))])
    np.testing.assert_array_equal(remove_ground(wall), wall)


def test_scattered_returns_are_not_ground():
    points = np.random.default_rng(42).uniform(-10, 10, (3000, 3))
    np.testing.assert_array_equal(remove_ground(points), points)
