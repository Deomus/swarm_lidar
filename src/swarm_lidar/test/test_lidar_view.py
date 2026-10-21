"""Обзор должен покрывать штатные места при вертикальном отставании."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from formation_controller.core import formation_offsets


@pytest.mark.parametrize('formation', ['line', 'column', 'wedge'])
@pytest.mark.parametrize('scale', [.75, 1., 2.])
def test_lidar_view_covers_formation_during_climb_and_small_tilt(formation, scale):
    model = Path(__file__).parents[1]/'models/swarm_x500_leader/model.sdf'
    vertical = ET.parse(model).find('.//scan/vertical')
    low, high = [float(vertical.findtext(key)) for key in ['min_angle', 'max_angle']]
    for offset in formation_offsets(formation, scale):
        for roll, pitch in [(0., 0.), (6., 0.), (-6., 0.), (0., 6.), (0., -6.)]:
            rotation = Rotation.from_euler('xy', [roll, pitch], degrees=True)
            for dz in [-.9, 0., .7]:
                relative = offset.copy()
                relative[2] += dz
                sensor = rotation.inv().apply(relative) - [0., 0., 1.9]
                angle = math.atan2(sensor[2], np.linalg.norm(sensor[:2]))
                assert low <= angle <= high, (formation, scale, roll, pitch, dz, angle)
