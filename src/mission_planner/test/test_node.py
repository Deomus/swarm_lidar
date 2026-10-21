import pytest
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.parameter import Parameter
from swarm_interfaces.msg import SwarmState

from mission_planner.grid import PlanningError
from mission_planner.node import MissionPlanner


@pytest.fixture
def planner():
    rclpy.init()
    node = MissionPlanner()
    node.set_parameters([Parameter('formation_radius', value=0.),
                         Parameter('safety_margin', value=0.)])
    yield node
    node.destroy_node()
    rclpy.shutdown()


def inputs(node):
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.width = grid.info.height = 10
    grid.info.resolution = 1.
    grid.info.origin.orientation.w = 1.
    grid.data = [0] * 100
    node.on_map(grid)
    state = SwarmState()
    state.uav_id = 0
    state.valid = True
    state.frame_id = 'map'
    state.timestamp = node.get_clock().now().to_msg()
    state.pose.position.x = state.pose.position.y = 1.5
    node.on_state(state)
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = goal.pose.position.y = 8.5
    return goal


def test_requires_map(planner):
    with pytest.raises(PlanningError, match='отсутствует'):
        planner.plan(PoseStamped())


def test_pose_age_and_frame(planner):
    goal = inputs(planner)
    planner.state.timestamp.sec -= 2
    with pytest.raises(PlanningError, match='устарела'):
        planner.plan(goal)
    planner.state.timestamp = planner.get_clock().now().to_msg()
    goal.header.frame_id = 'world'
    with pytest.raises(PlanningError, match='frame map'):
        planner.plan(goal)


def test_path_height_and_invalid_pose(planner):
    goal = inputs(planner)
    path = planner.plan(goal)
    assert path.header.frame_id == 'map'
    assert path.poses[-1].pose.position.x == 8.5
    assert all(p.pose.position.z == 3. for p in path.poses)
    planner.state.valid = False
    with pytest.raises(PlanningError, match='валидной'):
        planner.plan(goal)


def test_path_finishes_at_requested_point_instead_of_cell_center(planner):
    goal = inputs(planner)
    goal.pose.position.x, goal.pose.position.y = 8.1, 7.2
    path = planner.plan(goal)
    assert path.poses[-1].pose.position.x == pytest.approx(8.1)
    assert path.poses[-1].pose.position.y == pytest.approx(7.2)


def test_requested_altitude_and_rejection_outside_limits(planner):
    goal = inputs(planner)
    goal.pose.position.z = 12.
    assert all(p.pose.position.z == 12. for p in planner.plan(goal).poses)
    goal.pose.position.z = 31.
    with pytest.raises(PlanningError, match='Высота'):
        planner.plan(goal)


def test_column_geometry_blocks_path_near_obstacle(planner):
    goal = inputs(planner)
    grid = planner.map
    grid.info.width = grid.info.height = 50
    grid.data = [0] * 2500
    grid.data[21*50+8] = 100
    planner.state.pose.position.x, planner.state.pose.position.y = 20.5, 20.5
    goal.pose.position.x, goal.pose.position.y = 20.5, 22.5
    offsets = PoseArray()
    offsets.header.frame_id = 'uav_0/heading'
    offsets.header.stamp = planner.get_clock().now().to_msg()
    for x in [-4., -8., -12.]:
        p = Pose()
        p.position.x = x
        offsets.poses.append(p)
    planner.on_offsets(offsets)
    with pytest.raises(PlanningError, match='препятствии'):
        planner.plan(goal)
