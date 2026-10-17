"""Turn RViz Publish Point clicks into a selected robot's goal.

Click a robot (within ``select_radius``) to select it. The next click that
is not on a robot is sent to ``/fleet/direct`` for that robot.
"""

from __future__ import annotations

import yaml
import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from fleet_nav.fleet_logic import click_action


class ClickGoal(Node):
    def __init__(self):
        super().__init__("click_goal")
        self.declare_parameter("fleet_yaml", "")
        self.declare_parameter("select_radius", 0.7)
        with open(str(self.get_parameter("fleet_yaml").value), encoding="utf-8") as handle:
            config = yaml.safe_load(handle)
        self.names = list(config["robots"])
        self.radius = float(self.get_parameter("select_radius").value)
        self.poses: dict[str, tuple[float, float]] = {}
        self.selected: str | None = None
        for name in self.names:
            self.create_subscription(
                Odometry,
                f"/{name}/odom",
                lambda msg, robot=name: self._on_odom(robot, msg),
                qos_profile_sensor_data,
            )
        self.create_subscription(PointStamped, "/clicked_point", self._on_click, 10)
        self.direct_pub = self.create_publisher(PoseStamped, "/fleet/direct", 10)
        self.get_logger().info(
            "Click a robot to select it, then click the goal. "
            f"Selection radius {self.radius:.2f} m."
        )

    def _on_odom(self, name: str, msg: Odometry) -> None:
        self.poses[name] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_click(self, msg: PointStamped) -> None:
        if len(self.poses) < len(self.names):
            self.get_logger().warning("Waiting for every robot pose before accepting a click.")
            return
        selected, goal = click_action(
            self.poses,
            msg.point.x,
            msg.point.y,
            self.selected,
            self.radius,
        )
        self.selected = selected
        if goal is None:
            if selected is None:
                self.get_logger().info("Click ignored. Select a robot first.")
            else:
                self.get_logger().info(f"Selected {selected}. Click the goal.")
            return
        command = PoseStamped()
        command.header.frame_id = selected
        command.header.stamp = self.get_clock().now().to_msg()
        command.pose.position.x = goal[0]
        command.pose.position.y = goal[1]
        command.pose.orientation.w = 1.0
        self.direct_pub.publish(command)
        self.get_logger().info(f"Goal for {selected}: ({goal[0]:.2f}, {goal[1]:.2f}).")


def main() -> None:
    rclpy.init()
    node = ClickGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
