"""Map the warehouse with robot_1 only. Gazebo and RViz stay open."""

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    share = get_package_share_directory("fleet_nav")
    fleet_yaml = os.path.join(share, "config", "fleet.yaml")
    world = os.path.join(share, "worlds", "warehouse.sdf")
    xacro_file = os.path.join(share, "urdf", "robot.urdf.xacro")
    with open(fleet_yaml, encoding="utf-8") as handle:
        fleet = yaml.safe_load(handle)
    spec = fleet["robots"]["robot_1"]
    description = xacro.process_file(
        xacro_file,
        mappings={"robot_name": "robot_1", "chassis_rgba": str(spec["rgba"])},
    ).toxml()

    gazebo = ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen")
    bridge = RosGzBridge(
        bridge_name="mapping_bridge",
        config_file=os.path.join(share, "config", "bridge.yaml"),
        use_composition=False,
        bridge_params=[{"use_sim_time": True}],
    )
    robot_state = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="robot_1",
        output="screen",
        remappings=[("tf", "/tf"), ("tf_static", "/tf_static")],
        parameters=[{
            "use_sim_time": True,
            "robot_description": description,
            "frame_prefix": "robot_1/",
        }],
    )
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_robot_1",
            output="screen",
            parameters=[{"use_sim_time": True}],
            arguments=[
                "-world", "warehouse",
                "-topic", "/robot_1/robot_description",
                "-name", "robot_1",
                "-x", str(spec["x"]),
                "-y", str(spec["y"]),
                "-z", "0.1",
            ],
        )],
    )
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(share, "config", "slam_toolbox.yaml"),
            {"use_sim_time": True},
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(share, "rviz", "mapping.rviz")],
        parameters=[{"use_sim_time": True}],
    )
    return LaunchDescription([gazebo, bridge, robot_state, spawn, slam, rviz])
