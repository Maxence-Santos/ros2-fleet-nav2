"""Give each mission to the nearest idle robot and requeue a stall."""

from __future__ import annotations

import yaml
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

from fleet_nav.fleet_logic import BLOCKED, NAVIGATING, SUCCEEDED, FleetBook
from fleet_nav.map_grid import load_occupancy


class FleetManager(Node):
    def __init__(self):
        super().__init__("fleet_manager")
        self.declare_parameter("fleet_yaml", "")
        self.declare_parameter("map_yaml", "")
        self.declare_parameter("demo", True)

        with open(str(self.get_parameter("fleet_yaml").value), encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
        self.poses = {name: (float(item["x"]), float(item["y"])) for name, item in config["robots"].items()}
        self.demo_goals = [(float(item[0]), float(item[1])) for item in config.get("demo_goals", [])]
        self.book = FleetBook(
            list(self.poses),
            timeout_s=float(config.get("timeout_s", 20.0)),
            progress_m=float(config.get("progress_m", 0.3)),
            cooldown_s=float(config.get("cooldown_s", 8.0)),
        )
        for name, (x, y) in self.poses.items():
            self.book.set_pose(name, x, y, 0.0)
            self.book.robots[name].pose_known = False

        self.demo = bool(self.get_parameter("demo").value)
        self.demo_sent = False
        goal_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.goal_pubs = {
            name: self.create_publisher(PoseStamped, f"/{name}/goal_pose", goal_qos) for name in self.poses
        }
        self.assignment_pub = self.create_publisher(String, "/fleet/assignments", 10)
        for name in self.poses:
            self.create_subscription(
                Odometry, f"/{name}/odom", lambda msg, robot=name: self._on_odom(robot, msg), qos_profile_sensor_data
            )
            self.create_subscription(
                String,
                f"/{name}/nav_status",
                lambda msg, robot=name: self._on_status(robot, msg),
                goal_qos,
            )
        self.create_subscription(PoseStamped, "/fleet/mission", self._on_mission, 10)
        self.create_subscription(PoseStamped, "/fleet/direct", self._on_direct, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.occupancy = load_occupancy(str(self.get_parameter("map_yaml").value))
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_tf()
        self._publish_map()
        self.create_timer(0.5, self._tick)
        self.get_logger().info(f"Fleet manager watching {', '.join(self.poses)}.")

    def _publish_static_tf(self) -> None:
        stamp = self.get_clock().now().to_msg()
        transforms = []
        for name in self.poses:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "map"
            transform.child_frame_id = f"{name}/odom"
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self.static_tf.sendTransform(transforms)

    def _publish_map(self) -> None:
        message = OccupancyGrid()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.info.resolution = self.occupancy.resolution
        message.info.width = int(self.occupancy.display.shape[1])
        message.info.height = int(self.occupancy.display.shape[0])
        message.info.origin.position.x = self.occupancy.origin_x
        message.info.origin.position.y = self.occupancy.origin_y
        message.info.origin.orientation.w = 1.0
        message.data = self.occupancy.display.flatten().tolist()
        self.map_pub.publish(message)

    def _on_odom(self, name: str, msg: Odometry) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self.book.set_pose(name, msg.pose.pose.position.x, msg.pose.pose.position.y, now)

    def _on_status(self, name: str, msg: String) -> None:
        if msg.data not in (SUCCEEDED, BLOCKED, NAVIGATING):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        self.book.note_status(name, msg.data, now)

    def _on_mission(self, msg: PoseStamped) -> None:
        mission_id = self.book.submit(msg.pose.position.x, msg.pose.position.y, requeue=False)
        self.get_logger().info(f"Mission {mission_id} queued at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}).")

    def _on_direct(self, msg: PoseStamped) -> None:
        name = msg.header.frame_id
        if name not in self.book.robots:
            self.get_logger().warning(f"Direct mission ignored. Unknown robot '{name}'.")
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        assignment = self.book.command(name, msg.pose.position.x, msg.pose.position.y, now)
        self._publish_cancels()
        self._publish_assignment(assignment)

    def _publish_cancels(self) -> None:
        for name in self.book.take_cancels():
            cancel = PoseStamped()
            cancel.header.frame_id = "cancel"
            cancel.header.stamp = self.get_clock().now().to_msg()
            self.goal_pubs[name].publish(cancel)
            self.get_logger().info(f"Cancelled {name}.")

    def _publish_assignment(self, assignment) -> None:
        message = PoseStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = assignment.x
        message.pose.position.y = assignment.y
        message.pose.orientation.w = 1.0
        self.goal_pubs[assignment.robot].publish(message)
        note = String()
        note.data = f"{assignment.robot} mission={assignment.mission_id} x={assignment.x:.2f} y={assignment.y:.2f}"
        self.assignment_pub.publish(note)
        self.get_logger().info(f"Assigned {note.data}.")

    def _tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.demo and not self.demo_sent and all(robot.pose_known for robot in self.book.robots.values()):
            for x, y in self.demo_goals:
                mission_id = self.book.submit(x, y, requeue=False)
                self.get_logger().info(f"Demo mission {mission_id} at ({x:.2f}, {y:.2f}).")
            self.demo_sent = True
        assigned = self.book.tick(now)
        self._publish_cancels()
        for assignment in assigned:
            self._publish_assignment(assignment)


def main() -> None:
    rclpy.init()
    node = FleetManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
