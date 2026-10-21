import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from lidar_tracker.timing import OrientationHistory


def test_orientation_is_interpolated_at_measurement_time():
    history = OrientationHistory()
    history.add(10., Rotation.from_euler('z', 0.).as_quat())
    history.add(10.3, Rotation.from_euler('z', .09).as_quat())
    np.testing.assert_allclose(history.at(10.15).apply([12., 0., 0.]),
        Rotation.from_euler('z', .045).apply([12., 0., 0.]), atol=1e-10)
    np.testing.assert_allclose(history.at(10.).apply([12., 0., 0.]), [12., 0., 0.])
    with pytest.raises(ValueError, match='согласована'):
        history.at(9.)
