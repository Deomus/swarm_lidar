import unittest

import numpy as np

from lidar_tracker.tracker_node import cluster_points


class TestClusterPoints(unittest.TestCase):
    def test_separates_targets(self):
        points = np.array([
            [1.0, 0.0, 0.0], [1.1, 0.0, 0.0], [1.0, 0.1, 0.0],
            [5.0, 0.0, 0.0], [5.1, 0.0, 0.0], [5.0, 0.1, 0.0],
        ])
        clusters = cluster_points(points, tolerance=0.3, min_size=3)
        self.assertEqual(len(clusters), 2)

    def test_discards_noise(self):
        points = np.array([[1.0, 0.0, 0.0], [5.0, 0.0, 0.0]])
        self.assertEqual(cluster_points(points, tolerance=0.3, min_size=3), [])
