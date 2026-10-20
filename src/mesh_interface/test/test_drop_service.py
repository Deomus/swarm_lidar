"""Сервис читает действующую длительность и не вводит неверный отказ."""

from types import SimpleNamespace

import pytest

from mesh_interface.channel import DelayChannel
from mesh_interface.node import MeshNode


def make_node(duration):
    node = MeshNode.__new__(MeshNode)
    node.now = lambda: 10.
    node.get_parameter = lambda _: SimpleNamespace(value=duration[0])
    node.channels = [(DelayChannel(), None)]
    node.drop_duration = .7
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
