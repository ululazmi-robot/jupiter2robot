import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Define Launch Configurations (Arguments)
    multi_robot_name = LaunchConfiguration('multi_robot_name')
    set_lidar_frame_id = LaunchConfiguration('set_lidar_frame_id')
    model = LaunchConfiguration('model')

    # 2. Locate the required packages
    jupiter_bringup_dir = FindPackageShare('jupiterobot2_bringup')
    jupiter_desc_dir = FindPackageShare('jupiterobot2_description')

    # 3. Setup Robot State Publisher (Translates description.launch.xml)
    urdf_path = PathJoinSubstitution([jupiter_desc_dir, 'urdf', 'jupiter2_pro.urdf.xacro'])

    # value_type=str is required on the hardware bringup too: a bare Command()
    # gets YAML-parsed by launch_ros and the URDF's comments break it.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]), value_type=str)
        }]
    )

    # 4. Include the Core Base Launch
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([jupiter_bringup_dir, 'launch', 'jupiterobot2_core.launch.py'])
        )
    )

    # 5. Include the Lidar Launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([jupiter_bringup_dir, 'launch', 'jupiterobot2_lidar.launch.py'])
        )
    )

    # 6. Return the full description (Fixed!)
    return LaunchDescription([
        DeclareLaunchArgument('multi_robot_name', default_value=''),
        DeclareLaunchArgument('set_lidar_frame_id', default_value='base_scan'),
        DeclareLaunchArgument('model', default_value='pro'),
        
        rsp_node,
        core_launch,
        lidar_launch
    ])