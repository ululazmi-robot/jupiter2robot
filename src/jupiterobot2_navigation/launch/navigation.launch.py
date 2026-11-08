import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_gazebo = get_package_share_directory('jupiterobot2_gazebo')
    
    # Path to your map
    map_yaml_file = os.path.join(pkg_gazebo, 'maps', 'robocup_maps.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': 'true',
                'autostart': 'true',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
            }.items()
        )
    ])