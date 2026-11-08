"""Bring up the Jupiter 2 head pan joint (AX-12A, ID 6) under ROS 2.

This replaces the ROS 1 `head.launch` that shipped with the description
package. That file used <node pkg= type=/> and
<rosparam> under <node>, none of which ROS 2 launch understands.

    ros2 launch jupiterobot2_head_bringup head.launch.py
    ros2 launch jupiterobot2_head_bringup head.launch.py port:=/dev/ttyUSB1 auto_enable:=true

Torque is OFF until you enable it (safety):
    ros2 service call /head/enable std_srvs/srv/SetBool "{data: true}"
Then move it:
    ros2 topic pub --once /head/joint_goal sensor_msgs/msg/JointState \
      "{name: [head_joint], position: [0.5]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from jupiterobot2_head_bringup.head_node import _default_head_port


def _resolve_port():
    """Resolve at launch time, not import time, so a replugged device is seen."""
    return _default_head_port()


def generate_launch_description():
    port = LaunchConfiguration("port")
    dxl_id = LaunchConfiguration("dxl_id")
    auto_enable = LaunchConfiguration("auto_enable")
    publish_urdf = LaunchConfiguration("publish_urdf")

    head_desc = get_package_share_directory("jupiterobot2_head_description")
    urdf = os.path.join(head_desc, "urdf", "jupiterobot2_head.urdf.xacro")

    head_node = Node(
        package="jupiterobot2_head_bringup",
        executable="head_node",
        name="head_node",
        output="screen",
        parameters=[{
            "port": port,
            "dxl_id": dxl_id,
            "auto_enable": auto_enable,
            "joint_name": "head_joint",
            "min_rad": -1.5,
            "max_rad": 1.5,
        }],
    )

    # robot_state_publisher needs a robot_description to broadcast head_joint.
    # Kept optional because the full robot bringup already publishes one, and
    # two publishers of the same TF tree is a classic source of duplicate-TF
    # confusion.
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="head_state_publisher",
        output="screen",
        condition=IfCondition(publish_urdf),
        parameters=[{
            "robot_description": __import__("subprocess").check_output(
                ["xacro", urdf]).decode(),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "port", default_value=_resolve_port(),
            description="FTDI FT232H holding the AX-12A bus. Defaults to "
                        "/dev/jupiter/head when the udev rule is installed, "
                        "else the FTDI found by VID, else /dev/ttyUSB1."),
        DeclareLaunchArgument("dxl_id", default_value="6"),
        DeclareLaunchArgument(
            "auto_enable", default_value="false",
            description="energise the servo immediately. Leave false: the node "
                        "reads position without holding torque."),
        DeclareLaunchArgument(
            "publish_urdf", default_value="false",
            description="also start robot_state_publisher for head_joint. Set "
                        "true only when running this package on its own."),
        head_node,
        rsp,
    ])
