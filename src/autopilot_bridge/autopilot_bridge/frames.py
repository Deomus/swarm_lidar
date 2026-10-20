"""Преобразования без неявной подмены систем координат."""
import numpy as np
from scipy.spatial.transform import Rotation

ENU_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
FRD_FLU = np.diag([1., -1., -1.])

def enu_ned(vector):
    return ENU_NED @ np.asarray(vector, dtype=float)

def quaternion_ned_frd_to_enu_flu(wxyz):
    q = np.asarray(wxyz, dtype=float)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-6:
        raise ValueError('Некорректный quaternion PX4')
    ned_frd = Rotation.from_quat(q[[1, 2, 3, 0]]).as_matrix()
    return Rotation.from_matrix(ENU_NED @ ned_frd @ FRD_FLU).as_quat()

def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

def angular_state(quaternion_xyzw, angular_body_frd):
    rotation = Rotation.from_quat(quaternion_xyzw)
    roll, pitch, yaw = rotation.as_euler('xyz')
    body_flu = np.asarray(angular_body_frd, dtype=float) * [1., -1., -1.]
    if not np.isfinite(body_flu).all() or abs(np.cos(pitch)) < .1:
        raise ValueError('Недостоверная угловая скорость или вырожденный yaw')
    yaw_rate = (body_flu[1] * np.sin(roll) + body_flu[2] * np.cos(roll)) / np.cos(pitch)
    return rotation.apply(body_flu), float(yaw), float(yaw_rate)

def command_is_fresh(command, now, timeout, frame='map'):
    velocity = command.target_velocity.linear
    return (command.valid and command.frame_id == frame and
            0 <= now - stamp_seconds(command.source_stamp) <= timeout and
            np.isfinite([velocity.x, velocity.y, velocity.z,
                         command.target_velocity.angular.z]).all())

