import heapq
import math
import os
import random
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, CameraInfo, Image, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from topology_msgs.msg import MapProperty, NavPoint, RouteInfo, TopologyMap, TopologyRoute
from topology_msgs.srv import NavToTopologyPoint


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.z = math.sin(yaw * 0.5)
    quat.w = math.cos(yaw * 0.5)
    return quat


def read_pgm(path: str) -> Tuple[int, int, List[int]]:
    with open(path, "rb") as handle:
        magic = handle.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Unsupported map format in {path}: {magic!r}")

        tokens: List[bytes] = []
        while len(tokens) < 3:
            line = handle.readline()
            if not line:
                raise ValueError(f"Malformed PGM header in {path}")
            if line.startswith(b"#"):
                continue
            tokens.extend(line.split())

        width, height, maxval = (int(token) for token in tokens[:3])
        if maxval > 255:
            raise ValueError(f"Unsupported PGM depth in {path}: {maxval}")

        data = list(handle.read(width * height))
        if len(data) != width * height:
            raise ValueError(f"Incomplete PGM data in {path}")
        return width, height, data


@dataclass
class GridMap:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_theta: float
    data: List[int]

    def index(self, gx: int, gy: int) -> int:
        return gy * self.width + gx

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def is_free(self, gx: int, gy: int) -> bool:
        if not self.in_bounds(gx, gy):
            return False
        value = self.data[self.index(gx, gy)]
        return 0 <= value < 50

    def inflated(self, radius_m: float) -> "GridMap":
        radius_cells = max(0, int(math.ceil(radius_m / self.resolution)))
        if radius_cells <= 0:
            return GridMap(
                resolution=self.resolution,
                width=self.width,
                height=self.height,
                origin_x=self.origin_x,
                origin_y=self.origin_y,
                origin_theta=self.origin_theta,
                data=list(self.data),
            )

        inflated_data = list(self.data)
        radius_sq = radius_cells * radius_cells
        occupied_cells = [
            (idx % self.width, idx // self.width)
            for idx, value in enumerate(self.data)
            if value < 0 or value >= 50
        ]
        for ox, oy in occupied_cells:
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy > radius_sq:
                        continue
                    nx = ox + dx
                    ny = oy + dy
                    if self.in_bounds(nx, ny):
                        inflated_data[self.index(nx, ny)] = 100

        return GridMap(
            resolution=self.resolution,
            width=self.width,
            height=self.height,
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            origin_theta=self.origin_theta,
            data=inflated_data,
        )

    def world_to_grid(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if not self.in_bounds(gx, gy):
            return None
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        wx = self.origin_x + (gx + 0.5) * self.resolution
        wy = self.origin_y + (gy + 0.5) * self.resolution
        return wx, wy

    def nearest_free(self, gx: int, gy: int, max_radius: int = 12) -> Optional[Tuple[int, int]]:
        if self.is_free(gx, gy):
            return gx, gy

        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    nx = gx + dx
                    ny = gy + dy
                    if self.is_free(nx, ny):
                        return nx, ny
        return None

    def to_message(self, frame_id: str) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation = yaw_to_quaternion(self.origin_theta)
        msg.data = self.data
        return msg

    def local_window(self, center_x: float, center_y: float, size_m: float) -> "GridMap":
        cells = max(2, int(size_m / self.resolution))
        if cells % 2 != 0:
            cells += 1
        half = cells // 2
        center = self.world_to_grid(center_x, center_y)
        if center is None:
            center = (self.width // 2, self.height // 2)
        cx, cy = center

        new_data: List[int] = []
        for gy in range(cy - half, cy + half):
            for gx in range(cx - half, cx + half):
                if self.in_bounds(gx, gy):
                    new_data.append(self.data[self.index(gx, gy)])
                else:
                    new_data.append(-1)

        origin_x = self.origin_x + (cx - half) * self.resolution
        origin_y = self.origin_y + (cy - half) * self.resolution
        return GridMap(
            resolution=self.resolution,
            width=cells,
            height=cells,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_theta=0.0,
            data=new_data,
        )


class DemoServer(Node):
    def __init__(self) -> None:
        super().__init__("astar_demo_server")

        self.declare_parameter("map_yaml", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("scan_frame", "base_scan")
        self.declare_parameter("update_rate", 20.0)
        self.declare_parameter("map_publish_rate", 1.0)
        self.declare_parameter("scan_publish_rate", 5.0)
        self.declare_parameter("camera_topic", "/camera/rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/rgb/camera_info")
        self.declare_parameter("camera_frame", "camera_rgb_optical_frame")
        self.declare_parameter("camera_width", 320)
        self.declare_parameter("camera_height", 240)
        self.declare_parameter("camera_publish_rate", 5.0)
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("obstacle_inflation_radius", 0.35)
        self.declare_parameter("obstacle_stop_distance", 0.16)
        self.declare_parameter("obstacle_slow_distance", 0.75)
        self.declare_parameter("max_linear_speed", 0.45)
        self.declare_parameter("max_angular_speed", 1.3)

        map_yaml = self.get_parameter("map_yaml").get_parameter_value().string_value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.scan_frame = self.get_parameter("scan_frame").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.camera_width = max(64, int(self.get_parameter("camera_width").value))
        self.camera_height = max(48, int(self.get_parameter("camera_height").value))
        self.robot_radius = max(0.05, float(self.get_parameter("robot_radius").value))
        self.obstacle_inflation_radius = max(
            self.robot_radius,
            float(self.get_parameter("obstacle_inflation_radius").value),
        )
        self.obstacle_stop_distance = max(0.0, float(self.get_parameter("obstacle_stop_distance").value))
        self.obstacle_slow_distance = max(
            self.obstacle_stop_distance + 0.05,
            float(self.get_parameter("obstacle_slow_distance").value),
        )
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        if not map_yaml:
            raise RuntimeError("Parameter 'map_yaml' is required.")

        self.grid_map = self._load_map(map_yaml)
        self.collision_map = self.grid_map.inflated(self.obstacle_inflation_radius)
        self.map_msg = self.grid_map.to_message(self.map_frame)
        self.global_costmap_msg = self.collision_map.to_message(self.map_frame)

        qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        qos_camera = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos_transient)
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid, "/global_costmap/costmap", qos_transient
        )
        self.local_costmap_pub = self.create_publisher(
            OccupancyGrid, "/local_costmap/costmap", qos_transient
        )
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.local_plan_pub = self.create_publisher(Path, "/local_plan", 10)
        self.trace_pub = self.create_publisher(Path, "/transformed_global_plan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 20)
        self.odom_alias_pub = self.create_publisher(Odometry, "/wheel/odometry", 20)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_sensor)
        self.camera_pub = self.create_publisher(Image, self.camera_topic, qos_camera)
        self.camera_info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_status", 10)
        self.footprint_pub = self.create_publisher(
            PolygonStamped, "/local_costmap/published_footprint", 10
        )
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.topology_pub = self.create_publisher(TopologyMap, "/map/topology", qos_transient)

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal_pose, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initial_pose, 10)
        self.create_subscription(OccupancyGrid, "/map/update", self._on_map_update, 10)
        self.create_subscription(TopologyMap, "/map/topology/update", self._on_topology_update, 10)

        self.topology_service = self.create_service(
            NavToTopologyPoint, "/nav_to_topology_point", self._on_topology_goal
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.pose_x, self.pose_y = self._pick_start_pose()
        self.pose_yaw = 0.0
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_w = 0.0
        self.applied_vx = 0.0
        self.applied_vy = 0.0
        self.applied_w = 0.0
        self.mode = "idle"
        self.goal_pose: Optional[Tuple[float, float, float]] = None
        self.global_plan: List[Tuple[float, float, float]] = []
        self.trace_points: List[Tuple[float, float, float]] = []
        self.topology_map = TopologyMap(
            map_name=os.path.splitext(os.path.basename(map_yaml))[0],
            map_property=MapProperty(support_controllers=["a_star"]),
            points=[],
            routes=[],
        )

        self.last_update_time = self.get_clock().now()
        self.last_map_publish_time = self.get_clock().now()
        self.last_scan_publish_time = self.get_clock().now()
        self.last_camera_publish_time = self.get_clock().now()
        self.last_status_publish_time = self.get_clock().now()

        update_rate = float(self.get_parameter("update_rate").value)
        self.map_publish_period = 1.0 / float(self.get_parameter("map_publish_rate").value)
        self.scan_publish_period = 1.0 / float(self.get_parameter("scan_publish_rate").value)
        camera_publish_rate = max(0.5, float(self.get_parameter("camera_publish_rate").value))
        self.camera_publish_period = 1.0 / camera_publish_rate
        self.update_timer = self.create_timer(1.0 / update_rate, self._on_timer)

        self._publish_static_topics(force=True)
        self.get_logger().info("A* demo server started.")

    def _load_map(self, yaml_path: str) -> GridMap:
        with open(yaml_path, "r", encoding="utf-8") as handle:
            meta = yaml.safe_load(handle)

        image_path = meta["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        width, height, pixels = read_pgm(image_path)
        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))
        resolution = float(meta["resolution"])
        origin = meta.get("origin", [0.0, 0.0, 0.0])

        data = [-1] * (width * height)
        for gy in range(height):
            source_row = height - gy - 1
            for gx in range(width):
                pixel = pixels[source_row * width + gx]
                occ = (255.0 - pixel) / 255.0
                if negate:
                    occ = 1.0 - occ
                index = gy * width + gx
                if occ > occupied_thresh:
                    data[index] = 100
                elif occ < free_thresh:
                    data[index] = 0
                else:
                    data[index] = -1

        return GridMap(
            resolution=resolution,
            width=width,
            height=height,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_theta=float(origin[2]),
            data=data,
        )

    def _pick_start_pose(self) -> Tuple[float, float]:
        for gy in range(self.collision_map.height // 2, self.collision_map.height):
            for gx in range(self.collision_map.width // 4, self.collision_map.width):
                if self.collision_map.is_free(gx, gy):
                    return self.collision_map.grid_to_world(gx, gy)
        return self.collision_map.grid_to_world(self.collision_map.width // 2, self.collision_map.height // 2)

    def _publish_static_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _on_cmd_vel(self, msg: Twist) -> None:
        self.cmd_vx = float(msg.linear.x)
        self.cmd_vy = float(msg.linear.y)
        self.cmd_w = float(msg.angular.z)

        if abs(self.cmd_vx) > 1e-3 or abs(self.cmd_vy) > 1e-3 or abs(self.cmd_w) > 1e-3:
            self.mode = "manual"
            self.goal_pose = None
            self.global_plan = []

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        goal_yaw = self._quat_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        self._set_new_goal(msg.pose.position.x, msg.pose.position.y, goal_yaw)

    def _on_initial_pose(self, msg: PoseWithCovarianceStamped) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = self._quat_to_yaw(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        cell = self.grid_map.world_to_grid(x, y)
        if cell is None:
            self.get_logger().warning("Ignored initial pose outside map.")
            return
        nearest = self.collision_map.nearest_free(*cell)
        if nearest is None:
            self.get_logger().warning("Ignored initial pose in blocked area.")
            return

        self.pose_x, self.pose_y = self.grid_map.grid_to_world(*nearest)
        self.pose_yaw = yaw
        self.mode = "idle"
        self.goal_pose = None
        self.global_plan = []
        self.trace_points = []
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_w = 0.0
        self.applied_vx = 0.0
        self.applied_vy = 0.0
        self.applied_w = 0.0
        self._publish_runtime_topics(force=True)

    def _on_map_update(self, msg: OccupancyGrid) -> None:
        if msg.info.width == 0 or msg.info.height == 0 or not msg.data:
            return

        self.grid_map = GridMap(
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_theta=0.0,
            data=[int(value) for value in msg.data],
        )
        self.collision_map = self.grid_map.inflated(self.obstacle_inflation_radius)
        self.map_msg = self.grid_map.to_message(self.map_frame)
        self.global_costmap_msg = self.collision_map.to_message(self.map_frame)
        self._replan_if_needed()
        self._publish_static_topics(force=True)

    def _on_topology_update(self, msg: TopologyMap) -> None:
        self.topology_map = msg
        self.topology_pub.publish(self.topology_map)

    def _on_topology_goal(self, request: NavToTopologyPoint.Request, response: NavToTopologyPoint.Response):
        for point in self.topology_map.points:
            if point.name == request.point_name:
                self._set_new_goal(point.x, point.y, point.theta)
                response.is_success = True
                response.message = f"Goal '{request.point_name}' accepted."
                return response

        response.is_success = False
        response.message = f"Unknown topology point: {request.point_name}"
        return response

    def _set_new_goal(self, goal_x: float, goal_y: float, goal_yaw: float) -> None:
        plan = self._plan_path((self.pose_x, self.pose_y), (goal_x, goal_y))
        if not plan:
            self.get_logger().warning("A* failed to find a path to the requested goal.")
            self.mode = "idle"
            self.goal_pose = None
            self.global_plan = []
            return

        safe_goal_x, safe_goal_y, _ = plan[-1]
        plan[-1] = (safe_goal_x, safe_goal_y, goal_yaw)
        self.goal_pose = (safe_goal_x, safe_goal_y, goal_yaw)
        if math.hypot(safe_goal_x - goal_x, safe_goal_y - goal_y) > self.collision_map.resolution:
            self.get_logger().info(
                "Adjusted goal to nearest collision-free pose: "
                f"({safe_goal_x:.2f}, {safe_goal_y:.2f})."
            )
        self.global_plan = plan
        self.mode = "auto"
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_w = 0.0
        self._publish_paths()

    def _replan_if_needed(self) -> None:
        if self.goal_pose is None:
            return
        self._set_new_goal(*self.goal_pose)

    def _plan_path(
        self, start_world: Tuple[float, float], goal_world: Tuple[float, float]
    ) -> List[Tuple[float, float, float]]:
        start_cell = self.collision_map.world_to_grid(*start_world)
        goal_cell = self.collision_map.world_to_grid(*goal_world)
        if start_cell is None or goal_cell is None:
            return []

        start_cell = self.collision_map.nearest_free(*start_cell)
        goal_cell = self.collision_map.nearest_free(*goal_cell)
        if start_cell is None or goal_cell is None:
            return []

        motions = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (-1, -1, math.sqrt(2.0)),
        ]

        frontier: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(frontier, (0.0, 0.0, start_cell))
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        cost_so_far: Dict[Tuple[int, int], float] = {start_cell: 0.0}

        while frontier:
            _, _, current = heapq.heappop(frontier)
            if current == goal_cell:
                break

            for dx, dy, step_cost in motions:
                nx = current[0] + dx
                ny = current[1] + dy
                if not self.collision_map.is_free(nx, ny):
                    continue

                if dx != 0 and dy != 0:
                    if not self.collision_map.is_free(current[0] + dx, current[1]) or not self.collision_map.is_free(
                        current[0], current[1] + dy
                    ):
                        continue

                new_cost = cost_so_far[current] + step_cost
                neighbor = (nx, ny)
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    heuristic = math.hypot(goal_cell[0] - nx, goal_cell[1] - ny)
                    heapq.heappush(frontier, (new_cost + heuristic, random.random(), neighbor))
                    came_from[neighbor] = current

        if goal_cell not in came_from and goal_cell != start_cell:
            return []

        cells = [goal_cell]
        current = goal_cell
        while current != start_cell:
            current = came_from[current]
            cells.append(current)
        cells.reverse()

        poses: List[Tuple[float, float, float]] = []
        for index, (gx, gy) in enumerate(cells):
            wx, wy = self.collision_map.grid_to_world(gx, gy)
            if index + 1 < len(cells):
                nx, ny = self.collision_map.grid_to_world(*cells[index + 1])
                yaw = math.atan2(ny - wy, nx - wx)
            elif poses:
                yaw = poses[-1][2]
            else:
                yaw = self.pose_yaw
            poses.append((wx, wy, yaw))
        return self._compress_path(poses)

    def _compress_path(self, path: List[Tuple[float, float, float]]) -> List[Tuple[float, float, float]]:
        if len(path) < 3:
            return path

        compressed = [path[0]]
        for index in range(1, len(path) - 1):
            x0, y0, _ = compressed[-1]
            x1, y1, _ = path[index]
            x2, y2, _ = path[index + 1]
            dx1 = round(x1 - x0, 4)
            dy1 = round(y1 - y0, 4)
            dx2 = round(x2 - x1, 4)
            dy2 = round(y2 - y1, 4)
            if abs(dx1 * dy2 - dy1 * dx2) > 1e-4:
                compressed.append(path[index])
        compressed.append(path[-1])
        return compressed

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        self._step_robot(max(0.0, min(dt, 0.1)))
        self._publish_runtime_topics(force=False)

    def _step_robot(self, dt: float) -> None:
        if dt <= 0.0:
            return

        vx = self.cmd_vx
        vy = self.cmd_vy
        vw = self.cmd_w

        if self.mode == "auto" and self.goal_pose is not None and self.global_plan:
            vx, vy, vw = self._tracking_command()

        vx = clamp(vx, -self.max_linear_speed, self.max_linear_speed)
        vy = clamp(vy, -self.max_linear_speed, self.max_linear_speed)
        vw = clamp(vw, -self.max_angular_speed, self.max_angular_speed)
        if self.mode != "auto":
            vx, vy = self._apply_obstacle_speed_constraint(vx, vy)

        world_dx = math.cos(self.pose_yaw) * vx - math.sin(self.pose_yaw) * vy
        world_dy = math.sin(self.pose_yaw) * vx + math.cos(self.pose_yaw) * vy
        next_x = self.pose_x + world_dx * dt
        next_y = self.pose_y + world_dy * dt
        next_yaw = normalize_angle(self.pose_yaw + vw * dt)

        if self._motion_is_safe(next_x, next_y):
            self.pose_x = next_x
            self.pose_y = next_y
            self.applied_vx = vx
            self.applied_vy = vy
        else:
            self.applied_vx = 0.0
            self.applied_vy = 0.0
            if self.mode == "auto":
                self._replan_if_needed()
            elif self.mode == "manual":
                self.get_logger().debug("Manual command blocked by inflated obstacle.")

        self.pose_yaw = next_yaw
        self.applied_w = vw
        self.trace_points.append((self.pose_x, self.pose_y, self.pose_yaw))
        if len(self.trace_points) > 2000:
            self.trace_points = self.trace_points[-2000:]

    def _apply_obstacle_speed_constraint(self, vx: float, vy: float) -> Tuple[float, float]:
        speed = math.hypot(vx, vy)
        if speed <= 1e-4:
            return vx, vy

        move_heading = self.pose_yaw + math.atan2(vy, vx)
        clearance = self._ray_cast_on_map(
            self.grid_map,
            self.pose_x,
            self.pose_y,
            move_heading,
            self.robot_radius + self.obstacle_slow_distance,
        )
        stop_clearance = self.robot_radius + self.obstacle_stop_distance
        slow_clearance = self.robot_radius + self.obstacle_slow_distance
        if clearance <= stop_clearance:
            return 0.0, 0.0
        if clearance < slow_clearance:
            scale = (clearance - stop_clearance) / (slow_clearance - stop_clearance)
            scale = clamp(scale, 0.0, 1.0)
            return vx * scale, vy * scale
        return vx, vy

    def _motion_is_safe(self, next_x: float, next_y: float) -> bool:
        return self._segment_is_safe(self.pose_x, self.pose_y, next_x, next_y)

    def _segment_is_safe(self, start_x: float, start_y: float, end_x: float, end_y: float) -> bool:
        start_cell = self.collision_map.world_to_grid(start_x, start_y)
        if start_cell is None or not self.collision_map.is_free(*start_cell):
            return False

        distance = math.hypot(end_x - start_x, end_y - start_y)
        step = max(self.collision_map.resolution * 0.5, 0.02)
        samples = max(1, int(math.ceil(distance / step)))
        for i in range(1, samples + 1):
            ratio = i / samples
            x = start_x + (end_x - start_x) * ratio
            y = start_y + (end_y - start_y) * ratio
            cell = self.collision_map.world_to_grid(x, y)
            if cell is None or not self.collision_map.is_free(*cell):
                return False
        return True

    def _tracking_command(self) -> Tuple[float, float, float]:
        goal_x, goal_y, goal_yaw = self.goal_pose
        dist_to_goal = math.hypot(goal_x - self.pose_x, goal_y - self.pose_y)

        if dist_to_goal < 0.12:
            heading_error = normalize_angle(goal_yaw - self.pose_yaw)
            if abs(heading_error) < 0.12:
                self.mode = "idle"
                self.goal_pose = None
                self.global_plan = []
                return 0.0, 0.0, 0.0
            return 0.0, 0.0, clamp(2.0 * heading_error, -self.max_angular_speed, self.max_angular_speed)

        nearest_idx = 0
        nearest_dist = float("inf")
        for index, pose in enumerate(self.global_plan):
            dist = math.hypot(pose[0] - self.pose_x, pose[1] - self.pose_y)
            if dist < nearest_dist:
                nearest_dist = dist
                nearest_idx = index

        lookahead_idx = self._select_tracking_target(nearest_idx)
        target_x, target_y, _ = self.global_plan[lookahead_idx]
        target_heading = math.atan2(target_y - self.pose_y, target_x - self.pose_x)
        heading_error = normalize_angle(target_heading - self.pose_yaw)

        linear = clamp(0.7 * dist_to_goal, 0.05, self.max_linear_speed)
        if abs(heading_error) > 1.0:
            linear *= 0.2
        elif abs(heading_error) > 0.5:
            linear *= 0.5
        angular = clamp(2.4 * heading_error, -self.max_angular_speed, self.max_angular_speed)
        return linear, 0.0, angular

    def _select_tracking_target(self, nearest_idx: int) -> int:
        if len(self.global_plan) <= 1:
            return 0

        lookahead_distance = 0.55
        anchor_idx: Optional[int] = None
        for index in range(nearest_idx, -1, -1):
            target_x, target_y, _ = self.global_plan[index]
            if self._segment_is_safe(self.pose_x, self.pose_y, target_x, target_y):
                anchor_idx = index
                break

        if anchor_idx is None:
            for index in range(nearest_idx + 1, len(self.global_plan)):
                target_x, target_y, _ = self.global_plan[index]
                if self._segment_is_safe(self.pose_x, self.pose_y, target_x, target_y):
                    return index
            return nearest_idx

        best_idx = anchor_idx
        for index in range(anchor_idx + 1, len(self.global_plan)):
            target_x, target_y, _ = self.global_plan[index]
            target_dist = math.hypot(target_x - self.pose_x, target_y - self.pose_y)
            if target_dist > lookahead_distance and best_idx != anchor_idx:
                break
            if self._segment_is_safe(self.pose_x, self.pose_y, target_x, target_y):
                best_idx = index
                if target_dist >= lookahead_distance:
                    break

        return best_idx

    def _publish_runtime_topics(self, force: bool) -> None:
        now = self.get_clock().now()
        self._publish_tf_and_odom()
        self._publish_paths()
        self._publish_footprint()

        if force or (now - self.last_map_publish_time).nanoseconds / 1e9 >= self.map_publish_period:
            self.last_map_publish_time = now
            self._publish_static_topics(force=False)

        if force or (now - self.last_scan_publish_time).nanoseconds / 1e9 >= self.scan_publish_period:
            self.last_scan_publish_time = now
            self._publish_scan()

        if force or (now - self.last_camera_publish_time).nanoseconds / 1e9 >= self.camera_publish_period:
            self.last_camera_publish_time = now
            self._publish_camera()

        if force or (now - self.last_status_publish_time).nanoseconds / 1e9 >= 1.0:
            self.last_status_publish_time = now
            self._publish_battery()
            self._publish_diagnostics()

    def _publish_static_topics(self, force: bool) -> None:
        stamp = self.get_clock().now().to_msg()
        self.map_msg.header.stamp = stamp
        self.global_costmap_msg.header.stamp = stamp
        self.map_pub.publish(self.map_msg)
        self.global_costmap_pub.publish(self.global_costmap_msg)

        local_costmap = self.collision_map.local_window(self.pose_x, self.pose_y, 4.0).to_message(self.map_frame)
        local_costmap.header.stamp = stamp
        self.local_costmap_pub.publish(local_costmap)

        self.topology_pub.publish(self.topology_map)

    def _publish_tf_and_odom(self) -> None:
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.pose_x
        transform.transform.translation.y = self.pose_y
        transform.transform.rotation = yaw_to_quaternion(self.pose_yaw)
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation = yaw_to_quaternion(self.pose_yaw)
        odom.twist.twist.linear.x = self.applied_vx
        odom.twist.twist.linear.y = self.applied_vy
        odom.twist.twist.angular.z = self.applied_w
        self.odom_pub.publish(odom)
        self.odom_alias_pub.publish(odom)

    def _publish_paths(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(self._path_message(self.global_plan, stamp))

        if self.global_plan:
            nearest_idx = min(
                range(len(self.global_plan)),
                key=lambda idx: math.hypot(self.global_plan[idx][0] - self.pose_x, self.global_plan[idx][1] - self.pose_y),
            )
            local_segment = self.global_plan[nearest_idx : nearest_idx + 12]
        else:
            local_segment = []
        self.local_plan_pub.publish(self._path_message(local_segment, stamp))
        self.trace_pub.publish(self._path_message(self.trace_points, stamp))

    def _path_message(self, points: List[Tuple[float, float, float]], stamp) -> Path:
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = stamp
        for x, y, yaw in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path.poses.append(pose)
        return path

    def _publish_footprint(self) -> None:
        polygon = PolygonStamped()
        polygon.header.stamp = self.get_clock().now().to_msg()
        polygon.header.frame_id = self.base_frame
        polygon.polygon.points = [
            Point32(x=0.18, y=0.14, z=0.0),
            Point32(x=0.18, y=-0.14, z=0.0),
            Point32(x=-0.18, y=-0.14, z=0.0),
            Point32(x=-0.18, y=0.14, z=0.0),
        ]
        self.footprint_pub.publish(polygon)

    def _publish_scan(self) -> None:
        laser = LaserScan()
        laser.header.stamp = self.get_clock().now().to_msg()
        laser.header.frame_id = self.scan_frame
        laser.angle_min = -math.pi
        laser.angle_max = math.pi
        laser.angle_increment = math.radians(2.0)
        laser.range_min = 0.05
        laser.range_max = 8.0

        scan_x = self.pose_x + math.cos(self.pose_yaw) * (-0.064)
        scan_y = self.pose_y + math.sin(self.pose_yaw) * (-0.064)

        ranges: List[float] = []
        beam_count = int(round((laser.angle_max - laser.angle_min) / laser.angle_increment)) + 1
        for beam_index in range(beam_count):
            angle = self.pose_yaw + laser.angle_min + beam_index * laser.angle_increment
            ranges.append(self._ray_cast(scan_x, scan_y, angle, laser.range_max))

        laser.ranges = ranges
        self.scan_pub.publish(laser)

    def _publish_camera(self) -> None:
        stamp = self.get_clock().now().to_msg()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = self.camera_frame
        image.height = self.camera_height
        image.width = self.camera_width
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = self.camera_width * 3
        image.data = self._render_camera_frame()
        self.camera_pub.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height = self.camera_height
        info.width = self.camera_width
        fx = float(self.camera_width) * 0.72
        fy = float(self.camera_height) * 0.92
        cx = float(self.camera_width) * 0.5
        cy = float(self.camera_height) * 0.5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.d = []
        info.distortion_model = "plumb_bob"
        self.camera_info_pub.publish(info)

    def _render_camera_frame(self) -> bytes:
        width = self.camera_width
        height = self.camera_height
        horizon = int(height * 0.46)
        buffer = bytearray(width * height * 3)

        for y in range(height):
            row = y * width * 3
            if y < horizon:
                shade = int(38 + 30 * y / max(1, horizon))
                color = (shade, shade + 2, shade + 6)
            else:
                mix = (y - horizon) / max(1, height - horizon)
                shade = int(72 + 36 * mix)
                color = (shade - 8, shade, shade - 10)
            buffer[row : row + width * 3] = bytes(color) * width

        self._draw_camera_walls(buffer, width, height, horizon)
        self._draw_camera_floor_grid(buffer, width, height, horizon)
        self._draw_robot_overlay(buffer, width, height)
        return bytes(buffer)

    def _draw_camera_walls(self, buffer: bytearray, width: int, height: int, horizon: int) -> None:
        fov = math.radians(70.0)
        max_range = 6.0
        for x in range(width):
            rel = (x / max(1, width - 1)) - 0.5
            ray_angle = self.pose_yaw + rel * fov
            distance = self._ray_cast(self.pose_x, self.pose_y, ray_angle, max_range)
            corrected = max(0.08, distance * math.cos(rel * fov))
            wall_height = int(min(height * 0.92, height * 0.72 / corrected))
            y0 = max(0, horizon - wall_height // 2)
            y1 = min(height, horizon + wall_height // 2)

            brightness = int(clamp(210 - corrected * 30.0, 52, 210))
            side_tint = int(18 * abs(rel))
            color = (
                max(0, brightness - 12 - side_tint),
                max(0, brightness - 4 - side_tint),
                brightness,
            )
            for y in range(y0, y1):
                idx = (y * width + x) * 3
                buffer[idx] = color[0]
                buffer[idx + 1] = color[1]
                buffer[idx + 2] = color[2]

    def _draw_camera_floor_grid(self, buffer: bytearray, width: int, height: int, horizon: int) -> None:
        vanishing_x = width // 2
        line_color = (128, 145, 130)
        for offset in range(-4, 5):
            bottom_x = width // 2 + offset * width // 7
            self._draw_line(buffer, width, height, vanishing_x, horizon, bottom_x, height - 1, line_color)

        y = horizon + 14
        while y < height:
            color = (118, 135, 122) if ((y - horizon) // 18) % 2 == 0 else (92, 112, 98)
            self._draw_line(buffer, width, height, 0, y, width - 1, y, color)
            y += max(12, int((y - horizon) * 0.26))

    def _draw_robot_overlay(self, buffer: bytearray, width: int, height: int) -> None:
        margin = 14
        panel_w = min(210, width - 2 * margin)
        panel_h = 54
        self._draw_rect(buffer, width, height, margin, margin, panel_w, panel_h, (24, 32, 38))
        bar_w = int((panel_w - 24) * 0.78)
        self._draw_rect(buffer, width, height, margin + 12, margin + 34, bar_w, 8, (70, 190, 120))

        yaw_marker = int((normalize_angle(self.pose_yaw) + math.pi) / (2.0 * math.pi) * (panel_w - 24))
        self._draw_rect(buffer, width, height, margin + 12 + yaw_marker - 2, margin + 16, 4, 14, (250, 200, 80))

    def _draw_rect(
        self,
        buffer: bytearray,
        width: int,
        height: int,
        x: int,
        y: int,
        rect_w: int,
        rect_h: int,
        color: Tuple[int, int, int],
    ) -> None:
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(width, x + rect_w)
        y1 = min(height, y + rect_h)
        for py in range(y0, y1):
            row = py * width * 3
            for px in range(x0, x1):
                idx = row + px * 3
                buffer[idx] = color[0]
                buffer[idx + 1] = color[1]
                buffer[idx + 2] = color[2]

    def _draw_line(
        self,
        buffer: bytearray,
        width: int,
        height: int,
        x0: int,
        y0: int,
        x1: int,
        y1: int,
        color: Tuple[int, int, int],
    ) -> None:
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            if 0 <= x0 < width and 0 <= y0 < height:
                idx = (y0 * width + x0) * 3
                buffer[idx] = color[0]
                buffer[idx + 1] = color[1]
                buffer[idx + 2] = color[2]
            if x0 == x1 and y0 == y1:
                break
            twice_err = 2 * err
            if twice_err >= dy:
                err += dy
                x0 += sx
            if twice_err <= dx:
                err += dx
                y0 += sy

    def _ray_cast(self, origin_x: float, origin_y: float, angle: float, max_range: float) -> float:
        return self._ray_cast_on_map(self.grid_map, origin_x, origin_y, angle, max_range)

    def _ray_cast_on_map(
        self,
        grid_map: GridMap,
        origin_x: float,
        origin_y: float,
        angle: float,
        max_range: float,
    ) -> float:
        step = max(grid_map.resolution * 0.5, 0.03)
        distance = laser_min = 0.05
        while distance <= max_range:
            x = origin_x + math.cos(angle) * distance
            y = origin_y + math.sin(angle) * distance
            cell = grid_map.world_to_grid(x, y)
            if cell is None:
                return distance
            if not grid_map.is_free(*cell):
                return max(laser_min, distance)
            distance += step
        return max_range

    def _publish_battery(self) -> None:
        battery = BatteryState()
        battery.header.stamp = self.get_clock().now().to_msg()
        battery.percentage = 0.78
        battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_pub.publish(battery)

    def _publish_diagnostics(self) -> None:
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()

        status = DiagnosticStatus()
        status.name = "astar_demo"
        status.hardware_id = "ros2_jazzy_demo"
        status.level = DiagnosticStatus.OK
        status.message = "running"
        status.values = [
            KeyValue(key="mode", value=self.mode),
            KeyValue(key="goal_active", value=str(self.goal_pose is not None).lower()),
            KeyValue(key="plan_points", value=str(len(self.global_plan))),
        ]
        diagnostics.status = [status]
        self.diagnostic_pub.publish(diagnostics)

    def _quat_to_yaw(self, z: float, w: float) -> float:
        return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
