import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths
    pkg_gazebo = get_package_share_directory('jupiterobot2_gazebo')
    
    # Check if you have a custom rviz config, otherwise use default
    rviz_config_path = os.path.join(pkg_gazebo, 'config', 'slam.rviz')

    # 2. SLAM Toolbox Node (Async mode is best for performance)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_laser_range': 10.0,
            'minimum_time_interval': 0.1,
            'mode': 'mapping'
        }]
    )

    # 3. Robot Localization (EKF)
    # This fuses IMU and Wheel Odom for a stable 'odom' frame
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Ensure this matches your package's ekf config path
            'yaml_cfg': os.path.join(pkg_gazebo, 'config', 'ekf.yaml')
        }]
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        ekf_node,
        slam_node,
        rviz_node
    ])