from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jupiterobot2_teleop',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e', # Opens in a new terminal window to capture key presses
        )
    ])