"""Сервис читает действующую длительность и не вводит неверный отказ."""

from types import SimpleNamespace

import pytest

from mesh_interface.channel import DelayChannel
from mesh_interface.node import MeshNode


def make_node(duration):
    node = MeshNode.__new__(MeshNode)
    node.now = lambda: 10.
    node.get_parameter = lambda name: SimpleNamespace(
        value=duration[0] if name == 'drop_duration' else duration[1] if len(duration) > 1 else 0)
    node.channels = [(DelayChannel(), None)]
    node.target_drop = None
    return node


def test_service_uses_changed_parameter_instead_of_startup_snapshot():
    duration = [.7]
    node = make_node(duration)
    response = node.drop_poses(None, SimpleNamespace())
    assert response.success
    assert node.channels[0][0].blocked_until == pytest.approx(10.7)
    duration[0] = 3.
    response = node.drop_poses(None, SimpleNamespace())
    assert response.success
    assert node.channels[0][0].blocked_until == 13.


@pytest.mark.parametrize('duration', [0., -1., float('nan'), float('inf'), True, '3'])
def test_invalid_dynamic_duration_does_not_start_loss(duration):
    node = make_node([duration])
    response = node.drop_poses(None, SimpleNamespace())
    assert not response.success
    assert node.channels[0][0].blocked_until is None


def test_channels_have_independent_loss_sequences():
    import rclpy
    rclpy.init()
    try:
        node = MeshNode()
        draws = [tuple(channel.random.random() for _ in range(5)) for channel, _ in node.channels]
        assert len(set(draws)) == len(node.channels) == 5
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_single_target_drop_keeps_other_poses():
    from swarm_interfaces.msg import RelativePose
    node = make_node([.7, 2])
    assert node.drop_poses(None, SimpleNamespace()).success
    channel = node.channels[0][0]
    for target in (1, 2, 3):
        node.on_pose(RelativePose(target_id=target))
    assert [m.target_id for m in channel.receive(11.)] == [1, 3] and channel.dropped == 1
    node.now = lambda: 10.8
    node.on_pose(RelativePose(target_id=2))
    assert [m.target_id for m in channel.receive(11.)] == [2]


def test_invalid_drop_target_is_rejected():
    response = make_node([.7, 5]).drop_poses(None, SimpleNamespace())
    assert not response.success and 'drop_target_id' in response.message
