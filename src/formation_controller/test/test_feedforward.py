"""Упреждение по команде ведущего: без запаздывания фильтра, с пределом ускорения."""

import numpy as np
import pytest

from formation_controller.core import RelativeVelocityController


def controller():
    return RelativeVelocityController(kp=.3, stale_timeout=.5, min_confidence=.4, max_speed=2.,
                                      max_acceleration=4., max_jerk=8., period=.05,
                                      velocity_smoothing='windowed')


def steps(ctl, leader_velocity, count):
    offset = np.array([-4., 0., 0.])
    out = None
    for k in range(count):
        out = ctl.step(offset, offset, np.zeros(3), leader_velocity, 0., 0., k*.05, k*.05)
    return out.velocity_enu


def test_feedforward_reaches_leader_command_without_filter_delay():
    # Без поправки (ошибка 0) за 0,5 с выход достигает команды ведущего:
    # прежнее окно сглаживания давало здесь меньше 0,5 м/с.
    assert steps(controller(), [1., 0., 0.], 10)[0] == pytest.approx(1.)


def test_output_respects_acceleration_and_speed_limits():
    ctl = controller()
    previous = np.zeros(3)
    for k in range(60):
        out = ctl.step([-4., 0., 0.], [-4., 0., 0.], np.zeros(3), [5., 0., 0.], 0., 0., k*.05, k*.05)
        assert np.linalg.norm(out.velocity_enu - previous) <= 4.*.05 + 1e-9
        assert np.linalg.norm(out.velocity_enu) <= 2. + 1e-9
        previous = out.velocity_enu


def test_node_takes_fresh_leader_command_and_falls_back_to_measured():
    from types import SimpleNamespace
    from swarm_interfaces.msg import SwarmCommand, SwarmState
    from formation_controller.node import FormationController
    leader = SwarmState(yaw_rate=.05)
    leader.velocity.linear.x = .4
    probe = SimpleNamespace(p=dict(stale_timeout=.5),
                            seconds=FormationController.seconds, leader_command=None)
    command = SwarmCommand(uav_id=0, frame_id='map', valid=True)
    command.source_stamp.sec = 10
    command.target_velocity.linear.x, command.target_velocity.angular.z = 1., .2
    FormationController.on_command(probe, command)
    velocity, yaw_rate = FormationController.leader_motion(probe, leader, 10.1)
    assert velocity[0] == 1. and yaw_rate == .2
    command.valid = False  # ведущий в HOLD: мост тормозит его до нуля
    velocity, yaw_rate = FormationController.leader_motion(probe, leader, 10.1)
    assert not velocity.any() and yaw_rate == 0.
    velocity, yaw_rate = FormationController.leader_motion(probe, leader, 11.)  # команда устарела
    assert velocity[0] == .4 and yaw_rate == .05
