import os
from os import pathsep
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():

    # --------------------
    # Path Setup
    # --------------------
    desc_share  = get_package_share_directory("jupiterobot2_description")
    desc_prefix = get_package_prefix("jupiterobot2_description")
    
    # --------------------
    # 1. Declare the World Argument
    # Change 'empty.world' to your filename (e.g., 'my_lab.world')
    # --------------------
    world_argument = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('jupiterobot2_gazebo'), 'worlds', 'robocup_2024.world.xml'),
        description='/home/ubuntu/fyp1/src/jupiterobot2_gazebo/worlds/robocup_2024.world.xml'
    )

    # --------------------
    # Set GAZEBO_MODEL_PATH so all mesh packages are found
    # --------------------
    model_path  = os.path.join(desc_share, "models")
    model_path += pathsep + os.path.join(desc_prefix, "share")

    try:
        head_prefix = get_package_prefix("jupiterobot2_head_description")
        model_path += pathsep + os.path.join(head_prefix, "share")
    except Exception:
        pass  

    # so100_description, not jupiterobot2_arm_description: the AX-12 arm package
    # was deleted when the SO-100 was fitted. GAZEBO_MODEL_PATH needs the
    # package that actually holds the arm meshes now.
    try:
        arm_prefix = get_package_prefix("so100_description")
        model_path += pathsep + os.path.join(arm_prefix, "share")
    except Exception:
        pass

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)
    env_var_uri = SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")

    # --------------------
    # Robot description (URDF/Xacro)
    # --------------------
    # value_type=str: launch_ros YAML-parses a bare Command(), and the expanded
    # URDF contains comments with "key: value" (so100.urdf), which breaks that.
    robot_description = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([
                FindPackageShare('jupiterobot2_gazebo'),
                'urdf',
                'jupiter2_pro.gazebo.xacro'
            ]),
            ' use_gazebo:=true'
        ]),
        value_type=str,
    )

    # --------------------
    # Gazebo Process
    # --------------------
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            LaunchConfiguration('world'), # This uses the argument defined above
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen'
    )

    # --------------------
    # Robot state publisher
    # --------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # --------------------
    # Spawn entity
    # --------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'jupiter2',
            '-timeout', '60'
        ],
        output='screen'
    )

    return LaunchDescription([
        world_argument,   # Added the argument declaration
        env_var,          
        env_var_uri,      
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])