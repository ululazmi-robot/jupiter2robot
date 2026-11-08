"""Bring up the REAL robot: SO-100 arm on the Feetech bus + head joint.

    ros2 launch jupiterobot2_gazebo hardware.launch.py

    # arm bus dead, just exercise the controllers:
    ros2 launch jupiterobot2_gazebo hardware.launch.py fake:=true

WHAT THIS STARTS
    robot_state_publisher   geometry + TF, from jupiter2_pro.gazebo.xacro
    ros2_control_node       the arm's hardware interface
    spawners                joint_state_broadcaster, arm/gripper/head controllers

WHAT THIS DOES NOT START
    base, lidar, camera, head servo node. Those are separate launches on
    purpose (jupiterobot2_bringup/jupiterobot2_core.launch.py,
    jupiterobot2_lidar.launch.py, astra_camera, jupiterobot2_head_bringup).
    Starting everything here would make one failure take down the lot.

    This also does NOT start Gazebo. It is a hardware launch.

SAFETY
    Controllers are activated but the arm will not move until you send it a
    trajectory. It holds position on activation. Check the pose in RViz first:

        ros2 launch jupiterobot2_description preview.launch.py

    and if the arm is somewhere unexpected, unplug it rather than commanding it.

DOMAIN
    Defaults to ROS_DOMAIN_ID 77, same as preview.launch.py, so probing hardware
    cannot collide with the live robot session on 42. Pass domain:= to change.

PORT
    /dev/jupiter/arm, the udev symlink. Not /dev/ttyACM1: ttyACM numbers are
    assigned by USB enumeration order and swap across reboots.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Loaded in this order. Broadcaster first so /joint_states exists before any
# trajectory controller starts publishing commands.
CONTROLLERS = ("joint_state_broadcaster", "head_group_controller",
               "arm_group_controller", "gripper_group_controller")


def _setup(context, *args, **kwargs):
    gz_share = get_package_share_directory("jupiterobot2_gazebo")

    model = os.path.join(gz_share, "urdf", "jupiter2_pro.gazebo.xacro")
    if not os.path.isfile(model):
        raise RuntimeError(f"hardware.launch.py: model not found at {model}")

    fake = LaunchConfiguration("fake").perform(context).strip().lower() in ("1", "true", "yes")
    port = LaunchConfiguration("port").perform(context)
    domain = LaunchConfiguration("domain").perform(context).strip()

    actions = []
    if domain:
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", domain))
        if domain == "42":
            actions.append(LogInfo(msg=(
                "hardware.launch.py: ROS_DOMAIN_ID=42 is the LIVE ROBOT domain. "
                "Another bringup may already own /dev/jupiter/arm.")))

    # xacro args, assembled as one string. Order does not matter to xacro.
    xacro_args = ["use_fake_hardware:=" + ("true" if fake else "false"),
                  "arm_port:=" + port]
    for key, arg in (("baudrate", "arm_baudrate"), ("servo_speed", "arm_servo_speed"),
                     ("servo_accel", "arm_servo_accel")):
        val = LaunchConfiguration(key).perform(context)
        if val:
            xacro_args.append(f"{arg}:={val}")

    # Forward the plugin-class override that hardware.xacro accepts. Without
    # this the xacro arg exists but can never be set from the command line, so
    # the documented escape hatch silently did nothing.
    plugin_class = LaunchConfiguration("arm_plugin_class").perform(context).strip()
    if plugin_class:
        xacro_args.append("arm_plugin_class:=" + plugin_class)

    # Build the command as a Command() substitution, not a plain string.
    # ParameterValue(plain_str) is NOT evaluated by launch: the node receives an
    # empty string and every consumer dies with "Error document empty" /
    # "invalid URDF passed in to robot parser". Command() is what actually runs
    # the process at launch time.
    xacro_cmd = Command(["xacro ", model, " " + " ".join(xacro_args)])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    actions += [
        LogInfo(msg=f"hardware.launch.py: fake={fake} port={port} "
                    f"domain={domain or '<ambient>'}"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": False}],
        ),
    ]

    # The controller manager. In Humble this is the ros2_control_node
    # executable, and it reads robot_description from the parameter file we hand
    # it, so the hardware plugin resolves the same URDF.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(gz_share, "config", "controllers.yaml"),
        ],
    )

    if fake:
        actions.append(control_node)
    else:
        # ORDER IS CRITICAL. The arm bus is exclusive-access: only one process
        # can hold /dev/jupiter/arm. ros2_control_node opens it on activate, so
        # the preflight must run and EXIT before the manager starts. Starting
        # both at once makes preflight fail with
        #     device reports readiness to read but returned no data
        # which looks like broken hardware but is just two readers.
        #
        # RegisterEventHandler chains on the preflight exiting, and only starts
        # the controller manager if it exited cleanly. A guard that does not
        # gate is worse than no guard.
        preflight = Node(
            package="jupiterobot2_gazebo",
            executable="preflight_check.py",
            name="preflight_check",
            output="screen",
            parameters=[{"port": port, "tolerance_ticks": 25}],
        )

        def _on_preflight_exit(event, _context):
            if event.returncode == 0:
                return [LogInfo(msg="preflight passed, starting ros2_control"),
                        control_node]
            return [LogInfo(msg="PREFLIGHT FAILED - not starting ros2_control. "
                                "Arm is not at rest pose; park it and relaunch."),
                    Shutdown(reason="preflight failure")]

        actions.append(preflight)
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=preflight,
                          on_exit=_on_preflight_exit)))

    # Spawn controllers after the manager is up. Chaining on each spawner's
    # exit is the reliable way in Humble; a plain TimerAction races on slow
    # machines and produces "Could not contact service /controller_manager".
    #
    # head_group_controller is SKIPPED when fake:=false. The real hardware
    # system has no head_joint (it is an AX-12 on the FTDI bus, driven by
    # head_node), so spawning it fails with "Not acceptable command interfaces
    # combination ... head_joint/position Not existing" and kills a spawner.
    controllers = [c for c in CONTROLLERS
                   if not (c == "head_group_controller" and not fake)]

    spawners = []
    for ctrl in controllers:
        spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ctrl, "--controller-manager", "/controller_manager"],
            output="screen",
        ))
    actions += spawners
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "fake", default_value="false",
            description="true = mock_components/GenericSystem, no servo bus. "
                        "Use this to test controllers with the arm unplugged."),
        DeclareLaunchArgument(
            "port", default_value="/dev/jupiter/arm",
            description="Feetech ST3215 bus. udev symlink, not /dev/ttyACMn."),
        DeclareLaunchArgument("baudrate", default_value="1000000"),
        DeclareLaunchArgument("servo_speed", default_value="2400"),
        DeclareLaunchArgument("servo_accel", default_value="50"),
        DeclareLaunchArgument(
            "domain", default_value="77",
            description="ROS_DOMAIN_ID. Defaults to 77 so it cannot collide "
                        "with the live robot session on 42."),
        DeclareLaunchArgument(
            "arm_plugin_class",
            default_value="rbc_so100_controller/RBCSOARM100Interface",
            description="ros2_control hardware plugin class. Passed through to "
                        "hardware.xacro, which also accepts the (wrong) "
                        "upstream name rbc_so100_controller/SOARM100Interface."),
        OpaqueFunction(function=_setup),
    ])
