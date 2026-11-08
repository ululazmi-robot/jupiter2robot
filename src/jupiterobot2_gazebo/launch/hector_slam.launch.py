import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hector_mapping',
            executable='hector_mapping',
            name='hector_mapping',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'map_frame': 'map',
                'base_frame': 'base_footprint', # Matches your robot
                'odom_frame': 'odom',
                'pub_map_odom_transform': True, # Hector creates the map->odom link
                'map_resolution': 0.05,
                'map_size': 1024,
                'scan_topic': 'scan',
            }]
        ),
        # RViz2 to see the result
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])