from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_path = get_package_share_directory('jupiterobot2_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'jupiter2_pro.urdf.xacro')

    # ParameterValue(..., value_type=str) IS REQUIRED.
    # A bare Command() is treated as a YAML expression by launch_ros, and an
    # expanded URDF is only valid YAML when no comment contains "key: value".
    # so100.urdf has comments like "Joint 1: base -> shoulder", which made this
    # launch die with:
    #   Unable to parse the value of parameter robot_description as yaml
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([

        # Joint State Publisher (GUI sliders)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
