import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('jupiterobot2_gazebo')
    
    # 1. EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml'), {'use_sim_time': True}]
    )

    # 2. SLAM Toolbox Node
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml'), 
                    {'use_sim_time': True}]
    )

    return LaunchDescription([
        ekf_node,
        slam_node
    ])