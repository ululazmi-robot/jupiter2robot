import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    # Use the absolute path you provided
    world_path = '/home/ubuntu/fyp1/src/jupiterobot2_gazebo/worlds/robocup_2025.world.xml'
    
    # Fix for the "Preparing World" hang by skipping online database
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "http://localhost")

    # Start Gazebo with your custom world
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, 
             '-s', 'libgazebo_ros_init.so', 
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    return LaunchDescription([
        env_var,
        gazebo
    ])