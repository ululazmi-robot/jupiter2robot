import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('jupiterobot2_gazebo')
    
    # Ensure this points to a valid .rviz file or leave arguments empty []
    rviz_config_path = os.path.join(pkg_gazebo, 'config', 'slam.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])