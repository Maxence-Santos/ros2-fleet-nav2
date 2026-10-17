#!/usr/bin/env python3
"""Save the map built by slam_toolbox into this project's maps directory.

Same two-step export as ros2-slam-nav2: the slam_toolbox service when it is
up, otherwise the latched /map topic. Run it from a sourced terminal while
start_slam_demo.sh is still running.
"""

import os
from pathlib import Path
import subprocess
import sys
import time

# The conda interpreter is first on PATH and has neither numpy nor rclpy.
if os.path.realpath(sys.executable) != os.path.realpath("/usr/bin/python3"):
    os.execv("/usr/bin/python3", ["/usr/bin/python3", *sys.argv])

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy


def default_prefix() -> str:
    root = Path(__file__).resolve().parent
    return str(root / "src" / "fleet_nav" / "maps" / "warehouse_map")


def save_from_topic(prefix: str) -> bool:
    rclpy.init()
    node = Node("map_saver_client")
    received = [None]

    def on_map(msg: OccupancyGrid) -> None:
        received[0] = msg

    qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )
    node.create_subscription(OccupancyGrid, "/map", on_map, qos)
    print("Waiting for /map...")
    deadline = time.monotonic() + 12.0
    while rclpy.ok() and received[0] is None:
        if time.monotonic() > deadline:
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    if received[0] is None:
        print("No /map message.")
        node.destroy_node()
        rclpy.shutdown()
        return False

    msg = received[0]
    width = msg.info.width
    height = msg.info.height
    grid = np.array(msg.data, dtype=np.int8).reshape((height, width))
    image = np.full((height, width), 205, dtype=np.uint8)
    image[grid == 0] = 254
    image[grid == 100] = 0
    image = np.flipud(image)

    pgm_path = f"{prefix}.pgm"
    with open(pgm_path, "wb") as handle:
        header = f"P5\n# fleet_nav map saver\n{width} {height}\n255\n".encode("ascii")
        handle.write(header)
        handle.write(image.tobytes())
    yaml_path = f"{prefix}.yaml"
    origin = msg.info.origin.position
    Path(yaml_path).write_text(
        "image: "
        + os.path.basename(pgm_path)
        + "\nmode: trinary\n"
        + f"resolution: {msg.info.resolution}\n"
        + f"origin: [{origin.x}, {origin.y}, 0.0]\n"
        + "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    print(f"Map saved from /map\n  {yaml_path}\n  {pgm_path}")
    node.destroy_node()
    rclpy.shutdown()
    return True


def main() -> None:
    prefix = sys.argv[1] if len(sys.argv) > 1 else default_prefix()
    output_dir = os.path.dirname(prefix)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    print(f"Saving map to {prefix}.yaml/.pgm")
    command = [
        "ros2", "service", "call",
        "/slam_toolbox/save_map",
        "slam_toolbox/srv/SaveMap",
        f"{{name: {{data: '{prefix}'}}}}",
    ]
    try:
        result = subprocess.run(command, capture_output=True, text=True, timeout=8)
    except Exception as exc:
        print(f"slam_toolbox service skipped ({exc}).")
        result = None
    if result is not None and result.returncode == 0 and (
        "result=0" in result.stdout.replace(" ", "") or "result:0" in result.stdout.replace(" ", "")
    ):
        print(f"Map saved by slam_toolbox\n  {prefix}.yaml/.pgm")
        return
    if result is not None and result.stdout:
        print(result.stdout.strip())
    print("Falling back to /map.")
    if not save_from_topic(prefix):
        sys.exit(1)


if __name__ == "__main__":
    main()
