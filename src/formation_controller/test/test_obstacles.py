import pytest
from nav_msgs.msg import OccupancyGrid

from formation_controller.obstacles import check_transition_map
from formation_controller.core import formation_offsets


def test_growth_of_formation_is_rejected_before_follower_can_hit_obstacle():
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 50
    grid.info.resolution = 1.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 2500
    grid.data[21*50+8] = 100
    line, column = formation_offsets('line'), formation_offsets('column')
    check_transition_map(grid, [20.5, 20.5], line, line)
    with pytest.raises(ValueError, match='свободного места'):
        check_transition_map(grid, [20.5, 20.5], line, column)
    grid.data = [0] * 2500
    check_transition_map(grid, [20.5, 20.5], line, column)


def test_transition_cannot_expand_beyond_map_or_use_missing_map():
    points = formation_offsets('column')
    with pytest.raises(ValueError, match='карты'):
        check_transition_map(None, [0., 0.], points, points)


def test_coordinator_keeps_plan_when_mission_or_map_rejects_transition():
    import rclpy
    from swarm_interfaces.msg import SwarmState, FormationCommand
    from formation_controller.coordinator import FormationCoordinator
    rclpy.init()
    node = FormationCoordinator()
    node.now = lambda: 10.
    node.p['require_transition_map'] = True
    node.leader_intent = dict(mode='MISSION', stamp=10.)
    state = SwarmState(uav_id=0, valid=True, frame_id='map')
    state.timestamp.sec = 10
    state.pose.position.x = state.pose.position.y = 20.5
    node.leader = state
    command = FormationCommand(formation='column', scale=1., transition_time=25.)
    try:
        node.on_formation(command)
        assert 'удержание' in node.request_error
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.width = grid.info.height = 50
        grid.info.resolution = 1.
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*2500
        grid.data[21*50+8] = 100
        node.map = grid
        node.leader_intent['mode'] = 'HOLD'
        node.on_formation(command)
        assert 'свободного места' in node.request_error
        assert node.formation == 'line'
        grid.data = [0]*2500
        node.on_formation(command)
        assert node.request_error is None and node.formation == 'column'
    finally:
        node.destroy_node()
        rclpy.shutdown()
