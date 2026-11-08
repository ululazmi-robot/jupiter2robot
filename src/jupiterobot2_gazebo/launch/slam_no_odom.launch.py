import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Point to the NEW no-odom config file
    config_dir = os.path.join(get_package_share_directory('jupiterobot2_gazebo'), 'config')
    config_file = os.path.join(config_dir, 'mapper_params_no_odom.yaml')

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                config_file, 
                {'use_sim_time': True}
            ]
        )
    ])