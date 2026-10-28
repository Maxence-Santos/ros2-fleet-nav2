"""Namespaced A* follower. One process per robot, topics under ``/robot_N``."""

from __future__ import annotations

import math
from dataclasses import dataclass

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
from fleet_nav.map_grid import (
    FLEET_PEER_RADIUS,
    hold_crossing_path,
    load_occupancy,
    peer_is_planning_obstacle,
    peer_motion_speed,
)
from fleet_nav.navigation_geometry import (
    FLEET_LIDAR_FORWARD,
    AvoidanceState,
    aim_clear_of_peers,
    command_on_path,
    path_heading,
    path_lateral,
    path_merge_target,
    peers_the_path_clears,
    plan_reached,
    ranges_behind_cleared_peers,
    ranges_with_fleet,
    sector_body_returns,
    sector_distances_from_scan,
    step_avoidance,
)


@dataclass
class _Peer:
    x: float
    y: float
    yaw: float
    stamp: float
    speed: float
    win_x: float
    win_y: float
    win_t: float


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
        self.declare_parameter("peer_names", [""])

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
        self._ranges: tuple[float, ...] = ()
        self._angle_min = -math.pi
        self._angle_increment = 0.0
        self._range_min = 0.15
        self._peers: dict[str, _Peer] = {}
        self._around_key: tuple | None = None
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
        for peer in self.get_parameter("peer_names").value:
            if not peer:
                continue
            self.create_subscription(
                Odometry,
                f"/{peer}/odom",
                lambda msg, robot=peer: self._on_peer(robot, msg),
                qos_profile_sensor_data,
            )
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

    def _on_peer(self, name: str, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        now = self.get_clock().now().nanoseconds * 1e-9
        reported = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        previous = self._peers.get(name)
        if previous is None:
            self._peers[name] = _Peer(x, y, _yaw(msg.pose.pose.orientation), now, reported, x, y, now)
            return
        elapsed = now - previous.win_t
        if elapsed >= 0.15:
            speed = peer_motion_speed(
                reported, math.hypot(x - previous.win_x, y - previous.win_y), elapsed,
            )
            win_x, win_y, win_t = x, y, now
        else:
            # Keep the last pose window, but a twist that is already moving
            # counts at once. A zero twist does not cancel that window.
            speed = max(previous.speed, reported)
            win_x, win_y, win_t = previous.win_x, previous.win_y, previous.win_t
        self._peers[name] = _Peer(
            x, y, _yaw(msg.pose.pose.orientation), now, speed, win_x, win_y, win_t,
        )

    def _parked_peers(self) -> list[tuple[float, float, float]]:
        """Bases that are still, fresh enough that the pose is the robot."""
        now = self.get_clock().now().nanoseconds * 1e-9
        parked = []
        for peer in self._peers.values():
            if now - peer.stamp < 0.5 and peer_is_planning_obstacle(peer.speed):
                parked.append((peer.x, peer.y, peer.yaw))
        return parked

    def _plan_to(self, goal: tuple[float, float]) -> list:
        start = (self.x, self.y)
        parked = self._parked_peers()
        if parked:
            path = self.occupancy.plan_around(start, goal, parked)
            if len(path) >= 2:
                return path
        return self.occupancy.plan(start, goal)

    def _on_scan(self, msg: LaserScan) -> None:
        ranges = msg.ranges
        if self._peers:
            now = self.get_clock().now().nanoseconds * 1e-9
            mates = [
                (peer.x, peer.y, peer.yaw)
                for peer in self._peers.values()
                if now - peer.stamp < 0.5
            ]
            if mates:
                ranges = ranges_with_fleet(
                    ranges,
                    msg.angle_min,
                    msg.angle_increment,
                    self.x + FLEET_LIDAR_FORWARD * math.cos(self.yaw),
                    self.y + FLEET_LIDAR_FORWARD * math.sin(self.yaw),
                    self.yaw,
                    mates,
                    range_min=msg.range_min,
                    range_max=msg.range_max if msg.range_max > 0.0 else 3.0,
                )
        sectors = sector_distances_from_scan(
            ranges,
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
        self._ranges = tuple(ranges)
        self._angle_min = msg.angle_min
        self._angle_increment = msg.angle_increment
        self._range_min = msg.range_min

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
        path = self._plan_to(goal)
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
        self._around_key = None
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
            if plan_reached(
                self.x, self.y, self.goal, self.path, self.goal_tolerance,
            ):
                self.status = SUCCEEDED
                self.goal = None
                self.path = []
                self._publish_status()
                self.status = IDLE
            else:
                command = self._pursue()
        self.cmd_pub.publish(command)
        self._publish_status()

    def _pull_path_off_parked_robots(self) -> None:
        """Replan once if this path still runs through a robot that is still.

        The goal can arrive before the other poses. One attempt per set of
        positions: a corridor that cannot fit the margin keeps the path it has.
        """
        if self.goal is None or len(self.path) < 2:
            return
        # Waiting at a crossing. The other robot is on this path on purpose.
        if hold_crossing_path(
            self.avoidance.active, self.avoidance.side, self.avoidance.cross_hold,
        ):
            return
        parked = self._parked_peers()
        if not parked:
            return
        key = tuple(sorted((round(x, 1), round(y, 1)) for x, y, _yaw in parked))
        if key == self._around_key:
            return
        too_close = False
        for mate_x, mate_y, _yaw in parked:
            for point_x, point_y in self.path:
                if math.hypot(point_x - mate_x, point_y - mate_y) < FLEET_PEER_RADIUS - 0.02:
                    too_close = True
                    break
            if too_close:
                break
        if not too_close:
            self._around_key = key
            return
        revised = self.occupancy.plan_around((self.x, self.y), self.goal, parked)
        self._around_key = key
        if len(revised) < 2:
            return
        for mate_x, mate_y, _yaw in parked:
            nearest = min(math.hypot(point_x - mate_x, point_y - mate_y) for point_x, point_y in revised)
            if nearest < FLEET_PEER_RADIUS - 0.04:
                return
        self.path = revised
        self.path_index = 0
        self._publish_path()

    def _pursue(self) -> Twist:
        now = self.get_clock().now().nanoseconds * 1e-9
        command = Twist()
        self._pull_path_off_parked_robots()
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
        cleared = peers_the_path_clears(
            self.path,
            [(pose_x, pose_y) for pose_x, pose_y, _yaw in self._parked_peers()],
            FLEET_PEER_RADIUS - 0.06,
        )
        if on_path:
            aim = path_merge_target(
                self.x, self.y, self.path, self.path_index, target[0], target[1],
            )
            safe_aim = aim_clear_of_peers(
                self.x, self.y, self.path, self.path_index, cleared, FLEET_PEER_RADIUS,
            )
            if safe_aim is not None:
                aim = safe_aim
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
        # The saved map is compared from 0.15 m ahead of the base. Starting
        # at the base made every wall look closer than the ray and the robot
        # passed a shelf. The scan itself is taken 0.10 m behind that point.
        scan_x = self.x + FLEET_LIDAR_FORWARD * math.cos(self.yaw)
        scan_y = self.y + FLEET_LIDAR_FORWARD * math.sin(self.yaw)
        lidar_x = self.x + 0.15 * math.cos(self.yaw)
        lidar_y = self.y + 0.15 * math.sin(self.yaw)
        # One ray along the closest return. The nearest cell anywhere in the
        # sector is a shelf corner, and it used to explain a robot standing
        # in the middle of the same sector.
        # A shelf corner can be the closest hit in a sector while a walker
        # stands on the next ray. The pass has to see that walker. Clearance
        # still uses the raw minimum, so the offset does not steer into the shelf.
        open_front = open_left = open_right = None
        open_bearing = open_left_bearing = open_right_bearing = None
        see_front_angle = self.front_angle
        see_left_angle = self.left_angle
        see_right_angle = self.right_angle
        if self._ranges and self._angle_increment > 0.0:
            def map_at(angle: float) -> float:
                return self.occupancy.range_to_mapped(
                    lidar_x, lidar_y, self.yaw + angle, samples=1,
                    lidar_x=scan_x, lidar_y=scan_y,
                )

            body = sector_body_returns(
                self._ranges, self._angle_min, self._angle_increment, map_at,
                range_min=self._range_min,
            )
            if body.front < 900.0:
                open_front = body.front
                open_bearing = body.front_angle
                see_front_angle = body.front_angle
            if body.left < 900.0:
                open_left = body.left
                open_left_bearing = body.left_angle
                see_left_angle = body.left_angle
            if body.right < 900.0:
                open_right = body.right
                open_right_bearing = body.right_angle
                see_right_angle = body.right_angle
        mapped = self.occupancy.range_to_mapped(
            lidar_x, lidar_y, self.yaw + see_front_angle, samples=1,
            lidar_x=scan_x, lidar_y=scan_y,
        )
        mapped_left = self.occupancy.range_to_mapped(
            lidar_x, lidar_y, self.yaw + see_left_angle, samples=1,
            lidar_x=scan_x, lidar_y=scan_y,
        )
        mapped_right = self.occupancy.range_to_mapped(
            lidar_x, lidar_y, self.yaw + see_right_angle, samples=1,
            lidar_x=scan_x, lidar_y=scan_y,
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
            left_bearing=self.left_angle,
            right_bearing=self.right_angle,
            dest_x=self.goal[0] - self.x,
            dest_y=self.goal[1] - self.y,
            open_front=open_front,
            open_bearing=open_bearing,
            open_left=open_left,
            open_right=open_right,
            open_left_bearing=open_left_bearing,
            open_right_bearing=open_right_bearing,
            cleared_peers=cleared,
        )
        if passing is not None:
            side = self.avoidance.side
            side_changed = (side > 0.0) != (was_side > 0.0) or (side == 0.0) != (was_side == 0.0)
            if not was_passing or side_changed:
                if side > 0.0:
                    message = "Passing the robot on the right."
                elif side < 0.0:
                    message = "Passing the robot on the left."
                elif passing[0] > 0.05:
                    message = "Crossing ahead of the other robot."
                elif passing[0] < -0.02 or abs(passing[1]) > 0.05:
                    message = "Turning clear to let the other robot pass."
                else:
                    message = "Stopping to let the other robot pass."
                self.get_logger().info(message)
            command.linear.x = passing[0]
            command.angular.z = passing[1]
            self.blocked_since = None
            return command
        front_cmd, left_cmd, right_cmd = ranges_behind_cleared_peers(
            self.x,
            self.y,
            self.yaw,
            self.front,
            self.left,
            self.right,
            self.front_angle,
            self.left_angle,
            self.right_angle,
            cleared,
        )
        linear, angular, stalling = command_on_path(
            self.x,
            self.y,
            self.yaw,
            aim[0],
            aim[1],
            front_cmd,
            self.max_linear,
            self.max_angular,
            self.stop_distance,
            left=left_cmd,
            right=right_cmd,
            lateral=lateral,
            align=not self.cruising,
        )
        # Spin in place to face the path at the start. A later mild bend stays
        # a moving turn. A demi-tour still spins: command_on_path stops
        # translation once the heading error is past about 1.8 rad.
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
