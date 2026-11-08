from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{'device_id': 0}] # Change to 1 if js1
        ),
        Node(
            package='jupiterobot2_teleop',
            executable='joystick_teleop',
            name='jupiter2_teleop_joy',
            parameters=[{
                'scale_angular': 1.5,
                'scale_linear': 0.5,
                'axis_linear': 1,
                'axis_angular': 0,
                'axis_deadman': 5,
            }]
        )
    ])