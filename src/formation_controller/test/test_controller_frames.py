from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from swarm_interfaces.msg import RelativePose, SwarmState

from formation_controller.node import FormationController


def test_sensor_offset_and_tilt_are_removed_before_heading_control():
    rotation = Rotation.from_euler('xyz', [.1, -.2, .4])
    leader = SwarmState(yaw=.4)
    q = rotation.as_quat()
    leader.pose.orientation.x, leader.pose.orientation.y, leader.pose.orientation.z, leader.pose.orientation.w = q
    world_relative = np.array([-4., -3., .2])
    raw = rotation.inv().apply(world_relative + [0., 0., .45]) - [0., 0., 1.9]
    poses = {}
    for i in [1, 2, 3]:
        msg = RelativePose()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = raw
        poses[i] = msg
    probe = SimpleNamespace(poses=poses, p=dict(surface_bias=0., sensor_offset=[0., 0., 1.9], marker_offset_z=.45))
    result = FormationController.measured(probe, leader)
    expected = Rotation.from_euler('z', .4).inv().apply(world_relative)
    np.testing.assert_allclose(result, np.tile(expected, (3, 1)), atol=1e-7)
