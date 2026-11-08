import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Paths and Directories
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_gazebo = get_package_share_directory('jupiterobot2_gazebo')

    # Path to your map (swap with map_no_odom.yaml when testing your other map)
    map_file = os.path.join(pkg_gazebo, 'maps', 'trial_1.yaml')
    
    # Nav2 Parameters
    nav2_params = os.path.join(pkg_nav2, 'params', 'nav2_params.yaml')

    # RViz Config: Try to find your custom one, otherwise use the Nav2 default
    rviz_config_path = os.path.join(pkg_gazebo, 'config', 'slam.rviz')
    if not os.path.exists(rviz_config_path):
        rviz_config_path = os.path.join(pkg_nav2, 'rviz', 'nav2_default_view.rviz')

    # 2. Define the Nav2 Bringup (Includes Planner, Controller, and Recovery)
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'True',
            'params_file': nav2_params,
            'autostart': 'True'
        }.items()
    )

    # 3. Define the RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. Return everything in one LaunchDescription
    return LaunchDescription([
        nav2_bringup,
        rviz_node
    ])