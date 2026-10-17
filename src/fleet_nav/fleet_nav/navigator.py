"""Namespaced A* follower. One process per robot, topics under ``/robot_N``."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from fleet_nav.fleet_logic import BLOCKED, IDLE, NAVIGATING, SUCCEEDED
from fleet_nav.map_grid import load_occupancy
from fleet_nav.navigation_geometry import (
    AvoidanceState,
    command_on_path,
    path_heading,
    path_lateral,
    path_merge_target,
    sector_distances_from_scan,
    step_avoidance,
)


def _yaw(orientation) -> float:
    sin_y = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_y = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(sin_y, cos_y)


class FleetNavigator(Node):
    def __init__(self):
        super().__init__("fleet_navigator")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("goal_tolerance", 0.30)
        self.declare_parameter("waypoint_tolerance", 0.25)
        self.declare_parameter("max_linear", 0.30)
        self.declare_parameter("max_angular", 1.0)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("blocked_after", 4.0)
        self.declare_parameter("lookahead", 0.60)

        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        map_yaml = str(self.get_parameter("map_yaml").value)
        self.goal_tolerance = float(self.get_parameter("goal_tolerance").value)
        self.waypoint_tolerance = float(self.get_parameter("waypoint_tolerance").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.blocked_after = float(self.get_parameter("blocked_after").value)
        self.lookahead = float(self.get_parameter("lookahead").value)

        self.occupancy = load_occupancy(map_yaml)
        self.x = self.spawn_x
        self.y = self.spawn_y
        self.yaw = 0.0
        self.front = 999.0
        self.left = 999.0
        self.right = 999.0
        self.front_angle = 0.0
        self.left_angle = 1.27
        self.right_angle = -1.27
        self.avoidance = AvoidanceState()
        self.goal: tuple[float, float] | None = None
        self.path: list[tuple[float, float]] = []
        self.path_index = 0
        self.status = IDLE
        self.cruising = False
        self.blocked_since: float | None = None

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "nav_status", map_qos)
        self.path_pub = self.create_publisher(Path, "plan", 10)
        # Sensor QoS matches the Gazebo bridge (best effort). A reliable
        # subscription would stay silent.
        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "goal_pose", self._on_goal, map_qos)
        self.create_timer(0.1, self._control)
        self._publish_status()
        self.get_logger().info(
            f"Navigator ready. Map {self.occupancy.grid.shape[1]}x{self.occupancy.grid.shape[0]}, "
            f"spawn ({self.spawn_x:.2f}, {self.spawn_y:.2f})."
        )

    def _on_odom(self, msg: Odometry) -> None:
        # Ground-truth odometry is already expressed in the map frame.
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = _yaw(msg.pose.pose.orientation)

    def _on_scan(self, msg: LaserScan) -> None:
        sectors = sector_distances_from_scan(
            msg.ranges,
            msg.angle_min,
            msg.angle_increment,
            range_min=msg.range_min,
        )
        self.front = sectors.front
        self.left = sectors.left
        self.right = sectors.right
        self.front_angle = sectors.front_angle
        self.left_angle = sectors.left_angle
        self.right_angle = sectors.right_angle

    def _on_goal(self, msg: PoseStamped) -> None:
        if msg.header.frame_id == "cancel":
            self.goal = None
            self.path = []
            self.path_index = 0
            self.cruising = False
            self.blocked_since = None
            self.avoidance = AvoidanceState()
            self.status = IDLE
            self._publish_status()
            self.get_logger().info("Goal cancelled.")
            return
        goal = (msg.pose.position.x, msg.pose.position.y)
        self.status = NAVIGATING
        self._publish_status()
        path = self.occupancy.plan((self.x, self.y), goal)
        if not path:
            self.get_logger().warning(
                f"No path from ({self.x:.2f}, {self.y:.2f}) to ({goal[0]:.2f}, {goal[1]:.2f})."
            )
            self.status = BLOCKED
            self.goal = None
            self.path = []
            self.blocked_since = None
            self._publish_status()
            self.status = IDLE
            return
        self.goal = goal
        self.path = path
        self.path_index = 0
        self.cruising = False
        self.status = NAVIGATING
        self.blocked_since = None
        self.avoidance = AvoidanceState()
        self._publish_path()
        self._publish_status()
        self.get_logger().info(f"Goal ({goal[0]:.2f}, {goal[1]:.2f}), {len(path)} waypoints.")

    def _control(self) -> None:
        command = Twist()
        if self.status == NAVIGATING and self.goal is not None:
            distance_goal = math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)
            if distance_goal < self.goal_tolerance:
                self.status = SUCCEEDED
                self.goal = None
                self.path = []
                self._publish_status()
                self.status = IDLE
            else:
                command = self._pursue()
        self.cmd_pub.publish(command)
        self._publish_status()

    def _pursue(self) -> Twist:
        now = self.get_clock().now().nanoseconds * 1e-9
        command = Twist()
        while self.path_index < len(self.path) - 1:
            waypoint = self.path[self.path_index]
            if math.hypot(waypoint[0] - self.x, waypoint[1] - self.y) < self.waypoint_tolerance:
                self.path_index += 1
                continue
            # A point already behind the robot is not a reason to keep the
            # previous heading. Missing it by more than the tolerance used
            # to send the robot along that heading past the shelf.
            heading = path_heading(
                self.path, self.path_index, waypoint[0], waypoint[1], self.x, self.y,
            )
            along = (
                (self.x - waypoint[0]) * math.cos(heading)
                + (self.y - waypoint[1]) * math.sin(heading)
            )
            if along > 0.0:
                self.path_index += 1
                continue
            break
        target = self.occupancy.visible_lookahead(
            self.path, self.path_index, self.x, self.y, self.lookahead
        )
        # A blocked short chord is still the planned line. In the shelf-wall
        # gap the clearer lidar flank is the wall, and following it pushes the
        # nose in.
        on_path = target is not None or bool(self.path)
        if target is None and self.path:
            index = max(0, min(self.path_index, len(self.path) - 1))
            target = self.path[index]
        elif target is None:
            target = self.goal
        if target is None:
            return command
        if on_path:
            aim = path_merge_target(
                self.x, self.y, self.path, self.path_index, target[0], target[1],
            )
            lateral = path_lateral(self.x, self.y, self.path, self.path_index)
            heading = path_heading(
                self.path, self.path_index, target[0], target[1], self.x, self.y,
            )
        else:
            aim = target
            lateral = 999.0
            heading = math.atan2(aim[1] - self.y, aim[0] - self.x)
        aim_error = math.atan2(
            math.sin(math.atan2(aim[1] - self.y, aim[0] - self.x) - self.yaw),
            math.cos(math.atan2(aim[1] - self.y, aim[0] - self.x) - self.yaw),
        )
        # The lidar sits 0.15 m ahead of the base. The ray has to start there
        # or a wall reads closer than the map and looks like another robot.
        lidar_x = self.x + 0.15 * math.cos(self.yaw)
        lidar_y = self.y + 0.15 * math.sin(self.yaw)
        # One ray along the closest return. The nearest cell anywhere in the
        # sector is a shelf corner, and it used to explain a robot standing
        # in the middle of the same sector.
        mapped = self.occupancy.range_to_mapped(
            lidar_x, lidar_y, self.yaw + self.front_angle, samples=1,
        )
        mapped_left = self.occupancy.range_to_mapped(
            lidar_x, lidar_y, self.yaw + self.left_angle, samples=1,
        )
        mapped_right = self.occupancy.range_to_mapped(
            lidar_x, lidar_y, self.yaw + self.right_angle, samples=1,
        )
        if self.path:
            index = max(0, min(self.path_index, len(self.path) - 1))
            path_x, path_y = self.path[index]
        else:
            path_x, path_y = aim
        was_passing = self.avoidance.active
        was_side = self.avoidance.side
        passing, self.avoidance = step_avoidance(
            self.avoidance,
            now,
            self.x,
            self.y,
            self.yaw,
            math.cos(heading),
            math.sin(heading),
            aim[0] - self.x,
            aim[1] - self.y,
            self.front,
            self.left,
            self.right,
            aim_error,
            map_range=mapped,
            map_left=mapped_left,
            map_right=mapped_right,
            path_x=path_x,
            path_y=path_y,
            bearing=self.front_angle,
        )
        if passing is not None:
            side = self.avoidance.side
            side_changed = (side > 0.0) != (was_side > 0.0) or (side == 0.0) != (was_side == 0.0)
            if not was_passing or side_changed:
                if side > 0.0:
                    message = "Passing the robot on the right."
                elif side < 0.0:
                    message = "Passing the robot on the left."
                else:
                    message = "Stopping to let the other robot pass."
                self.get_logger().info(message)
            command.linear.x = passing[0]
            command.angular.z = passing[1]
            self.blocked_since = None
            return command
        linear, angular, stalling = command_on_path(
            self.x,
            self.y,
            self.yaw,
            aim[0],
            aim[1],
            self.front,
            self.max_linear,
            self.max_angular,
            self.stop_distance,
            left=self.left,
            right=self.right,
            lateral=lateral,
            align=not self.cruising,
        )
        # Spin in place only to face the path at the start. A later bend is
        # already on a feasible line, so the robot turns while moving.
        if abs(aim_error) <= 0.30:
            self.cruising = True
        command.linear.x = linear
        command.angular.z = angular
        if not stalling:
            self.blocked_since = None
            return command
        if self.blocked_since is None:
            self.blocked_since = now
        elif now - self.blocked_since >= self.blocked_after:
            self.status = BLOCKED
            self.goal = None
            self.path = []
            self.cruising = False
            self._publish_status()
            self.status = IDLE
            self.get_logger().warning("Stopped by an obstacle. Mission released.")
            return Twist()
        return command

    def _publish_status(self) -> None:
        message = String()
        message.data = self.status
        self.status_pub.publish(message)

    def _publish_path(self) -> None:
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.path:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)


def main() -> None:
    rclpy.init()
    node = FleetNavigator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
