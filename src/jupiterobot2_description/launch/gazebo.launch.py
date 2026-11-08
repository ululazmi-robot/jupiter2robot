import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('jupiterobot2_description')
    pkg_gazebo = get_package_share_directory('gazebo_ros')
    gz_pkg = get_package_share_directory('jupiterobot2_gazebo')

    # Sim geometry + Gazebo XML lives in jupiterobot2_gazebo. This launch is a
    # thin convenience wrapper; the real one is jupiterobot2_gazebo/launch/gazebo.launch.py
    # which also starts the controllers. Kept so `ros2 launch
    # jupiterobot2_description gazebo.launch.py` still works.
    xacro_file = os.path.join(gz_pkg, 'urdf', 'jupiter2_pro.gazebo.xacro')

    # value_type=str is required: launch_ros YAML-parses a bare Command(), and
    # the expanded URDF contains comments with "key: value" (see so100.urdf).
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str,
    )

    return LaunchDescription([

        # Gazebo (replaces RViz2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')
            )
        ),

        # Robot State Publisher (kept the same)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True   # changed False -> True for Gazebo
            }]
        ),

        # Spawn Entity (replaces joint_state_publisher_gui)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'jupiter2',
                '-topic',  'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
            ],
            output='screen'
        ),

    ])