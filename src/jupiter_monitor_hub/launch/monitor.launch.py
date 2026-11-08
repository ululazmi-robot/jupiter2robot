"""Start the Jupiter 2 monitor: the health hub.

The TUI is interactive, so it is NOT launched from here -- a launch file that
takes over your terminal is a launch file you will only use once. Run it
separately:

    ros2 launch jupiter_monitor_hub monitor.launch.py     # hub only
    monitor                                               # dashboard, other terminal

The hub publishes /robot/health and /robot/state and probes every driver in
config/drivers.yaml. It never opens a port for writing and never starts or stops
a driver, so it is safe to leave running alongside anything.

Why this exists at all rather than `ros2 run`: the TUI works without the hub
(it probes locally), but you only get measured topic rates and a single
/robot/state snapshot when the hub is up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rate = LaunchConfiguration("rate_hz")
    measure = LaunchConfiguration("measure_rates")
    odom = LaunchConfiguration("odom_topic")
    joints = LaunchConfiguration("joint_states_topic")
    caminfo = LaunchConfiguration("camera_info_topic")

    hub = Node(
        package="jupiter_monitor_hub",
        executable="hub",
        name="jupiter_monitor_hub",
        output="screen",
        parameters=[{
            "rate_hz": rate,
            "measure_rates": measure,
            "odom_topic": odom,
            "joint_states_topic": joints,
            "camera_info_topic": caminfo,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rate_hz", default_value="2.0",
                              description="how often /robot/health is published"),
        DeclareLaunchArgument("measure_rates", default_value="true",
                              description="subscribe to primary topics to measure Hz"),
        # Native topic names. Override these if you put the robot in a namespace.
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        DeclareLaunchArgument("joint_states_topic", default_value="/joint_states"),
        DeclareLaunchArgument("camera_info_topic",
                              default_value="/camera/depth/camera_info"),
        hub,
    ])
