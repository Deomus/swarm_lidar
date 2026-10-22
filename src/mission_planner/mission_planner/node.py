"""ROS-адаптер A*: только путь, без команд автопилоту."""
from math import floor, isfinite, sqrt
import json

from geometry_msgs.msg import PoseStamped, PoseArray
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
                           'occupancy_threshold': 50, 'max_map_cells': 40000,
                           'require_formation_geometry': False, 'geometry_timeout': .5,
                           'min_altitude': 1., 'max_altitude': 30.}.items():
            self.declare_parameter(key, value)
        for key in ['working_altitude', 'formation_radius', 'safety_margin', 'pose_timeout']:
            value = self.get_parameter(key).value
            if not isfinite(value) or value < 0 or (key == 'pose_timeout' and value == 0):
                raise ValueError(f'Некорректный параметр {key}')
        self.map = None
        self.state = None
        self.geometry = None
        self.planned_radius = None
        self.path_pub = self.create_publisher(Path, '/mission/path', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)
        self.result_pub = self.create_publisher(String, '/mission/plan_result', 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(SwarmState, '/mesh/states', self.on_state, 10)
        self.create_subscription(PoseStamped, '/mission/goal', self.on_goal, 10)
        self.create_subscription(PoseArray, '/swarm/target_offsets', self.on_offsets, 10)

    def on_map(self, msg):
        if self.map is not None and msg != self.map and self.planned_radius is not None:
            self.fail('Карта изменилась; требуется новый маршрут')
        self.map = msg

    def on_offsets(self, msg):
        if msg.header.frame_id != 'uav_0/heading' or len(msg.poses) != 3:
            return
        values = [[p.position.x, p.position.y, p.position.z] for p in msg.poses]
        if not all(isfinite(v) for point in values for v in point):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if not 0 <= now-stamp <= self.get_parameter('geometry_timeout').value:
            return
        radius = max(sqrt(sum(v*v for v in point)) for point in values)
        self.geometry = stamp, radius
        if self.planned_radius is not None and radius > self.planned_radius + 1e-6:
            self.fail('Габариты строя выросли; требуется новый маршрут')

    def on_state(self, msg):
        if msg.uav_id == 0:
            self.state = msg

    def fail(self, reason):
        self.planned_radius = None
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
            self.result_pub.publish(String(data=json.dumps(dict(status='REJECTED',
                goal_id=[goal.header.stamp.sec, goal.header.stamp.nanosec], reason=str(error)))))
            return
        self.path_pub.publish(path)
        self.status_pub.publish(String(data=f'PLANNED: {len(path.poses)} точек'))
        self.result_pub.publish(String(data=json.dumps(dict(status='PLANNED',
            goal_id=[goal.header.stamp.sec, goal.header.stamp.nanosec],
            path_stamp=[path.header.stamp.sec, path.header.stamp.nanosec]))))

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
        now = self.get_clock().now().nanoseconds * 1e-9
        geometry_valid = self.geometry is not None and 0 <= now-self.geometry[0] <= self.get_parameter('geometry_timeout').value
        if self.get_parameter('require_formation_geometry').value and not geometry_valid:
            raise PlanningError('Нет свежей геометрии строя')
        formation_radius = max(self.get_parameter('formation_radius').value,
                               self.geometry[1] if geometry_valid else 0.)
        radius = formation_radius + self.get_parameter('safety_margin').value
        # Нулевая высота цели означает рабочую высоту по умолчанию.
        altitude = goal.pose.position.z
        if altitude == 0.:
            altitude = self.get_parameter('working_altitude').value
        if not self.get_parameter('min_altitude').value <= altitude <= self.get_parameter('max_altitude').value:
            raise PlanningError('Высота цели вне разрешённого диапазона')
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
            pose.pose.position.z = altitude
            pose.pose.orientation.w = 1.
            path.poses.append(pose)
        # Последняя точка — точная цель внутри безопасной клетки, не её центр.
        path.poses[-1].pose.position.x = goal.pose.position.x
        path.poses[-1].pose.position.y = goal.pose.position.y
        self.planned_radius = formation_radius
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
