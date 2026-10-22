from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation
from swarm_interfaces.msg import RelativePose, SwarmState

from formation_controller.node import FormationController
from formation_controller.prediction import LeaderVelocityPredictor


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
    from lidar_tracker.timing import OrientationHistory
    history = OrientationHistory()
    history.add(0., q)
    probe = SimpleNamespace(poses=poses, states={i: SwarmState() for i in [1, 2, 3]},
                            orientation_history=history, seconds=FormationController.seconds,
                            p=dict(surface_bias=0., sensor_offset=[0., 0., 1.9], marker_offset_z=.45))
    result = FormationController.measured(probe, leader)
    expected = Rotation.from_euler('z', .4).inv().apply(world_relative)
    np.testing.assert_allclose(result, np.tile(expected, (3, 1)), atol=1e-7)


def test_prediction_requires_fresh_active_leader_intent_and_resets_in_hold():
    probe = SimpleNamespace(p=dict(stale_timeout=.5, predict_slot_motion=False),
                            predictors=[LeaderVelocityPredictor() for _ in range(3)],
                            leader_intent=dict(stamp=10., status='MANUAL', valid=True),
                            seconds=FormationController.seconds)
    probe.reset_prediction = lambda: FormationController.reset_prediction(probe)
    leader = SwarmState()
    leader.timestamp.sec = 10
    np.testing.assert_array_equal(FormationController.forward_velocity(probe, leader, 10.), [0, 0, 0])
    leader.timestamp.nanosec = 100000000
    leader.velocity.linear.x = .1
    assert FormationController.forward_velocity(probe, leader, 10.1)[0] > .1
    probe.leader_intent.update(status='HOLD', valid=False)
    np.testing.assert_array_equal(FormationController.forward_velocity(probe, leader, 10.1), [.1, 0, 0])
    probe.leader_intent.update(status='MANUAL', valid=True)
    # После HOLD первая скорость не вызывает оценки ускорения через разрыв.
    leader.velocity.linear.x = 1.
    np.testing.assert_array_equal(FormationController.forward_velocity(probe, leader, 10.1), [1., 0, 0])
    np.testing.assert_array_equal(FormationController.forward_velocity(probe, leader, 11.), [1., 0, 0])


def test_old_scan_uses_old_orientation_before_current_heading_transform():
    from lidar_tracker.timing import OrientationHistory
    history = OrientationHistory()
    old = Rotation.from_euler('z', .2)
    current = Rotation.from_euler('z', .29)
    history.add(10., old.as_quat())
    history.add(10.3, current.as_quat())
    leader = SwarmState(yaw=.29)
    leader.timestamp.sec, leader.timestamp.nanosec = 10, 300000000
    q = leader.pose.orientation
    q.x, q.y, q.z, q.w = current.as_quat()
    world_relative = np.array([-12., 0., 0.])
    raw = old.inv().apply(world_relative) - [0., 0., 1.9]
    poses = {}
    for i in [1, 2, 3]:
        msg = RelativePose()
        msg.measurement_stamp.sec = 10
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = raw
        poses[i] = msg
    probe = SimpleNamespace(poses=poses, states={i: SwarmState() for i in [1, 2, 3]},
        orientation_history=history, seconds=FormationController.seconds,
        p=dict(surface_bias=0., sensor_offset=[0., 0., 1.9], marker_offset_z=0.))
    expected = current.inv().apply(world_relative)
    np.testing.assert_allclose(FormationController.measured(probe, leader),
                               np.tile(expected, (3, 1)), atol=1e-8)


def test_slot_prediction_has_separate_history_and_does_not_double_turn_velocity():
    probe = SimpleNamespace(p=dict(stale_timeout=.5, predict_slot_motion=True),
                            predictors=[LeaderVelocityPredictor() for _ in range(3)],
                            leader_intent=dict(stamp=10., status='MANUAL', valid=True),
                            seconds=FormationController.seconds)
    probe.reset_prediction = lambda: FormationController.reset_prediction(probe)
    leader = SwarmState(yaw_rate=.08)
    leader.timestamp.sec = 10
    offsets = [np.array([-4., -4., 0.]), np.array([-4., 0., 0.])]
    for i, offset in enumerate(offsets):
        # Первый отсчёт только запоминает движение места, поправка ещё нулевая.
        np.testing.assert_array_equal(FormationController.forward_velocity(
            probe, leader, 10., i, offset, np.zeros(3)), np.zeros(3))
    leader.timestamp.nanosec = 100000000
    leader.yaw = .008
    outputs = [FormationController.forward_velocity(probe, leader, 10.1, i, offset, np.zeros(3))
               for i, offset in enumerate(offsets)]
    assert not np.allclose(outputs[0], outputs[1])
    assert all(0 < np.linalg.norm(output) <= .6 for output in outputs)
    probe.leader_intent.update(status='HOLD', valid=False)
    np.testing.assert_array_equal(FormationController.forward_velocity(
        probe, leader, 10.1, 0, offsets[0], np.zeros(3)), np.zeros(3))
    assert all(predictor._stamp is None for predictor in probe.predictors)
