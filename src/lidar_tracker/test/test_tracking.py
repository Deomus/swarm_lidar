import numpy as np
from lidar_tracker.tracking import TargetTracker, cluster_points


def test_noise_and_nonfinite():
    rng = np.random.default_rng(4)
    a = rng.normal([2, 0, 0], .02, (20, 3))
    b = rng.normal([4, 0, 0], .02, (20, 3))
    clusters = cluster_points(np.vstack([a, b, [9, 9, 9], [np.nan, 0, 0]]), .15, 5)
    assert len(clusters) == 2


def test_ids_survive_order_and_distance_change():
    tracker = TargetTracker([[1, 0, 0], [3, 0, 0]], alpha=1.)
    tracker.update([[3, 0, 0], [1, 0, 0]], 1.)
    results = tracker.update([[3, 1, 0], [1, -1, 0]], 1.5)
    np.testing.assert_allclose(results[0][0], [1, -1, 0])
    np.testing.assert_allclose(results[1][0], [3, 1, 0])


def test_missing_target_does_not_reassign_other_id():
    tracker = TargetTracker([[1, 0, 0], [5, 0, 0]], gate=1.)
    results = tracker.update([[5, 0, 0]], 1.)
    assert results[0] is None
    assert results[1] is not None


def test_ambiguous_close_targets_are_invalid():
    tracker = TargetTracker([[0, 0, 0], [1, 0, 0]], gate=2.)
    assert tracker.update([[.5, 0, 0]], 1.) == [None, None]


def test_long_loss_and_old_timestamp_do_not_rebind():
    tracker = TargetTracker([[0, 0, 0]], timeout=1.)
    tracker.update([[0, 0, 0]], 1.)
    assert tracker.update([[0, 0, 0]], .5) == [None]
    assert tracker.update([[0, 0, 0]], 3.) == [None]
