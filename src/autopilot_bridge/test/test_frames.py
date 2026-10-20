import numpy as np
from scipy.spatial.transform import Rotation
from autopilot_bridge.frames import enu_ned, quaternion_ned_frd_to_enu_flu, command_is_fresh, angular_state
from swarm_interfaces.msg import SwarmCommand

def test_axes_and_round_trip():
    np.testing.assert_allclose(enu_ned([1, 2, 3]), [2, 1, -3])
    np.testing.assert_allclose(enu_ned(enu_ned([1, 2, 3])), [1, 2, 3])

def test_north_heading_frd_becomes_flu_north():
    q = quaternion_ned_frd_to_enu_flu([1, 0, 0, 0])
    np.testing.assert_allclose(Rotation.from_quat(q).apply([1, 0, 0]), [0, 1, 0], atol=1e-10)
    np.testing.assert_allclose(Rotation.from_quat(q).apply([0, 0, 1]), [0, 0, 1], atol=1e-10)

def test_east_heading():
    q = Rotation.from_euler('z', np.pi/2).as_quat()
    result = quaternion_ned_frd_to_enu_flu(q[[3, 0, 1, 2]])
    np.testing.assert_allclose(Rotation.from_quat(result).as_matrix(), np.eye(3), atol=1e-10)

def test_commands_expire_and_reject_wrong_frame():
    cmd = SwarmCommand(valid=True, frame_id='map')
    cmd.source_stamp.sec = 2
    assert command_is_fresh(cmd, 2.1, .3)
    assert not command_is_fresh(cmd, 2.4, .3)
    assert not command_is_fresh(cmd, 1.9, .3)
    cmd.frame_id = 'uav_0/base_link'
    assert not command_is_fresh(cmd, 2.1, .3)

def test_yaw_rate_from_body_rates_with_tilt():
    roll, pitch, yaw = .2, .3, .7
    q = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()
    world, heading, rate = angular_state(q, [0., -.4, -.6])
    assert abs(heading - yaw) < 1e-9
    assert abs(rate - (.4*np.sin(roll)+.6*np.cos(roll))/np.cos(pitch)) < 1e-9
    np.testing.assert_allclose(world, Rotation.from_quat(q).apply([0., .4, .6]))

