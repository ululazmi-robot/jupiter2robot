"""Single-robot preview in RViz, driven by joint_state_publisher_gui sliders.

    ros2 launch jupiterobot2_description preview.launch.py

WHAT YOU GET
  robot_state_publisher  holds the URDF, broadcasts the fixed TF tree
  joint_state_publisher_gui  one slider per non-fixed joint, so you can pose
                             the arm/gripper/head by dragging instead of
                             publishing trajectories
  rviz2                  loads rviz/preview.rviz

The six movable joints are the SO-100 ones (shoulder_pan, shoulder_lift,
elbow_flex, wrist_flex, wrist_roll, gripper) plus head_joint and the two
wheels. Slider ranges come straight from the URDF <limit> tags, so they are
always correct for whatever the model currently says.

WHY THIS EXISTS SEPARATELY FROM display.launch.py
display.launch.py does the same thing but hardcodes the base xacro and does not
pass the rviz config, so RViz opens with an empty layout. This one is the
"look at the model and adjust it" entry point:

  * --sim / -s   load the Gazebo entry point instead of pure geometry, so you
                 see what Gazebo actually sees (all sensors, materials, and any
                 plugin that alters geometry). Slower to load but a truer
                 picture. Default is the geometry-only file, which is what you
                 want 95% of the time because it is instant.
  * --frame      set the RViz fixed frame. Defaults to base_footprint.
  * --rviz-config pass your own .rviz file.

BOTH entry points put the same 65 links / 64 joints in the tree, so what you
adjust here matches what Gazebo spawns. If they ever disagree, the refactor
broke and you should compare the expansions (see README, "Verification method").
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _resolve_model(context):
    """Pick the xacro entry point at launch time, not import time."""
    use_sim = LaunchConfiguration("sim").perform(context).lower() in ("1", "true", "yes")
    desc = get_package_share_directory("jupiterobot2_description")

    if use_sim:
        gz = get_package_share_directory("jupiterobot2_gazebo")
        path = os.path.join(gz, "urdf", "jupiter2_pro.gazebo.xacro")
    else:
        path = os.path.join(desc, "urdf", "jupiter2_pro.urdf.xacro")

    if not os.path.isfile(path):
        raise RuntimeError(
            "preview.launch.py: model not found at %s\n"
            "If sim:=true, jupiterobot2_gazebo must be built and sourced." % path
        )
    return path


def _launch_setup(context, *args, **kwargs):
    model_path = _resolve_model(context)

    desc = get_package_share_directory("jupiterobot2_description")
    rviz_default = os.path.join(desc, "rviz", "preview.rviz")

    # DOMAIN ISOLATION.
    # The real robot runs on ROS_DOMAIN_ID 42 (nav2 + turtlebot3_node + head_node
    # + a second robot_state_publisher). Starting this preview in that domain
    # puts a SECOND /robot_description and a SECOND TF tree on the wire, and
    # robot_state_publisher would fight the live one over odom->base_footprint.
    #
    # Default isolates to 77. Pass domain:=42 deliberately if you actually want
    # to overlay the model onto the live robot's TF -- and understand that you
    # are then publishing duplicate TF.
    domain = LaunchConfiguration("domain").perform(context).strip()
    actions = []
    if domain:
        actions.append(SetEnvironmentVariable("ROS_DOMAIN_ID", domain))
        if domain == "42":
            actions.append(LogInfo(msg=(
                "preview.launch.py: ROS_DOMAIN_ID=42 is the LIVE ROBOT domain. "
                "This preview will publish a second /robot_description and a "
                "second TF tree alongside the running hardware.")))

    # value_type=str is mandatory. A bare Command() gets YAML-parsed by
    # launch_ros, and the URDF contains comments with "key: value" (for
    # example "Joint 1: base -> shoulder" in so100.urdf). Without this the
    # launch dies with "Unable to parse the value of parameter
    # robot_description as yaml".
    robot_description = ParameterValue(
        Command(["xacro ", model_path]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": False}],
    )

    # Sliders for every movable joint. The GUI reads <limit> from the URDF, so
    # the ranges are never stale. rate + delta keep the TF stream light; raise
    # rate if the model looks laggy while dragging.
    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "rate": 20,
                     "delta": 0.01}],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Silent publisher, only when the GUI is switched off (headless / CI).
    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description,
                     "rate": 20}],
        condition=IfCondition(LaunchConfiguration("no_gui")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": False}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return actions + [
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        rsp,
        jsp_gui,
        jsp,
        rviz,
    ]


def generate_launch_description():
    desc = get_package_share_directory("jupiterobot2_description")

    sim_arg = DeclareLaunchArgument(
        "sim", default_value="false",
        description="true = preview the Gazebo entry point (all sensors and "
                    "materials) instead of the geometry-only model.")
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="start joint_state_publisher_gui sliders. Set false and "
                    "no_gui:=true for a static, headless preview.")
    no_gui_arg = DeclareLaunchArgument(
        "no_gui", default_value="false",
        description="start the silent joint_state_publisher instead of the GUI.")
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true",
        description="start RViz. Set false to just publish TF/robot_description.")
    cfg_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(desc, "rviz", "preview.rviz"),
        description="RViz config to load.")
    domain_arg = DeclareLaunchArgument(
        "domain", default_value="77",
        description="ROS_DOMAIN_ID for the preview. Defaults to 77 so it can "
                    "never collide with the live robot, which runs on 42. Set "
                    "to empty string to inherit the ambient domain.")
    frame_arg = DeclareLaunchArgument(
        "frame", default_value="base_footprint",
        description="Unused by the nodes; documented for parity with other "
                    "launches. Change the fixed frame in RViz instead.")

    return LaunchDescription([
        sim_arg, gui_arg, no_gui_arg, rviz_arg, cfg_arg, domain_arg, frame_arg,
        OpaqueFunction(function=_launch_setup),
    ])
