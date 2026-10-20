"""ROS-адаптер A*: только путь, без команд автопилоту."""
from math import floor, isfinite

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from swarm_interfaces.msg import SwarmState

from mission_planner.grid import PlanningError, astar, inflated_grid


class MissionPlanner(Node):
    def __init__(self):
        super().__init__('mission_planner')
        for key, value in {'working_altitude': 3., 'formation_radius': 6.,
                           'safety_margin': .75, 'pose_timeout': .5,
                           'occupancy_threshold': 50, 'max_map_cells': 40000}.items():
            self.declare_parameter(key, value)
        for key in ['working_altitude', 'formation_radius', 'safety_margin', 'pose_timeout']:
            value = self.get_parameter(key).value
            if not isfinite(value) or value < 0 or (key == 'pose_timeout' and value == 0):
                raise ValueError(f'Некорректный параметр {key}')
        self.map = None
        self.state = None
        self.path_pub = self.create_publisher(Path, '/mission/path', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(SwarmState, '/mesh/states', self.on_state, 10)
        self.create_subscription(PoseStamped, '/mission/goal', self.on_goal, 10)

    def on_map(self, msg):
        self.map = msg

    def on_state(self, msg):
        if msg.uav_id == 0:
            self.state = msg

    def fail(self, reason):
        # Пустой путь явно отзываeт прежний результат при неуспешной новой цели.
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=f'REJECTED: {reason}'))
        self.get_logger().warning(reason)

    def on_goal(self, goal):
        try:
            path = self.plan(goal)
        except (PlanningError, ValueError, OverflowError) as error:
            self.fail(str(error))
            return
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=f'PLANNED: {len(path.poses)} точек'))

    def plan(self, goal):
        grid, state = self.map, self.state
        if grid is None:
            raise PlanningError('Карта /map отсутствует')
        if state is None or not state.valid:
            raise PlanningError('Нет валидной позы лидера')
        if any(frame != 'map' for frame in [grid.header.frame_id, state.frame_id, goal.header.frame_id]):
            raise PlanningError('Карта, лидер и цель должны иметь frame map (ENU)')
        stamp = state.timestamp.sec + state.timestamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if age < 0 or age > self.get_parameter('pose_timeout').value:
            raise PlanningError('Поза лидера устарела или имеет будущее время')
        info = grid.info
        if info.width * info.height > self.get_parameter('max_map_cells').value:
            raise PlanningError('Карта превышает max_map_cells')
        q = info.origin.orientation
        if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6 or abs(abs(q.w) - 1.) > 1e-6:
            raise PlanningError('Поддерживается карта с identity orientation origin')
        origin = info.origin.position
        positions = [state.pose.position, goal.pose.position]
        if not all(isfinite(v) for p in positions + [origin] for v in [p.x, p.y, p.z]):
            raise PlanningError('Координаты содержат NaN или infinity')
        radius = (self.get_parameter('formation_radius').value
                  + self.get_parameter('safety_margin').value)
        blocked = inflated_grid(info.width, info.height, info.resolution, grid.data,
                                radius, self.get_parameter('occupancy_threshold').value)
        start, target = [(floor((p.x - origin.x) / info.resolution),
                          floor((p.y - origin.y) / info.resolution)) for p in positions]
        cells = astar(info.width, info.height, blocked, start, target)
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = origin.x + (x + .5) * info.resolution
            pose.pose.position.y = origin.y + (y + .5) * info.resolution
            pose.pose.position.z = self.get_parameter('working_altitude').value
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        return path


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
