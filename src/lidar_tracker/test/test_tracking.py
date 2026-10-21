import numpy as np
from lidar_tracker.tracking import TargetTracker, cluster_points


def test_dense_oversize_component_rejects_whole_frame_without_false_fragments():
    dense = np.random.default_rng(7).normal(0., .001, (3000, 3))
    assert cluster_points(dense, .5, 3, 500) == []


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


def test_restart_after_column_landing_preserves_lidar_ids():
    line = np.array([[-4., -4., -1.9], [-4., 0., -1.9], [-4., 4., -1.9]])
    column = np.array([[-8., 0., -1.9], [-4., 0., -1.9], [-12., 0., -1.9]])
    tracker = TargetTracker(line, alpha=1.)
    for step in range(61):
        positions = line + (column-line) * step/60
        assert all(r is not None for r in tracker.update(positions[::-1], 1.+step*.1))
    assert tracker.update(column, 30.) == [None]*3
    assert tracker.reset(31.)
    results = tracker.update(column[::-1], 31.1)
    np.testing.assert_allclose([r[0] for r in results], column)
    np.testing.assert_allclose(tracker.velocity, 0.)


def test_reset_uses_complete_frame_not_partial_landing_measurements():
    seeds = np.array([[-8., 0., -1.9], [-4., 0., -1.9], [-12., 0., -1.9]])
    tracker = TargetTracker(seeds, alpha=1.)
    tracker.update(seeds, 1.)
    tracker.update([[-8., 0., -1.]], 1.1)
    assert tracker.reset(10.)
    np.testing.assert_allclose(tracker.positions, seeds)
    assert tracker.reset(11.)  # Повторный reset не теряет сохранённый кадр.
    np.testing.assert_allclose(tracker.positions, seeds)


def test_reset_without_measurements_and_after_clock_rewind_uses_initial_slots():
    tracker = TargetTracker([[0., 0., 0.]], alpha=1.)
    assert not tracker.reset(1.)
    tracker.update([[1., 0., 0.]], 10.)
    assert not tracker.reset(0.)
    np.testing.assert_allclose(tracker.positions, [[0., 0., 0.]])
