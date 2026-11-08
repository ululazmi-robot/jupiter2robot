#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Jupiterobot2!
---------------------------
Moving around:
        w
   a    s    d
        x

w : move forward
x : move reverse
a : turn left
d : turn right
s : stop (zero velocity)

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

moveBindings = {
    'w': (1.0, 0.0),
    'x': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    's': (0.0, 0.0),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
}

def getKey(settings):
    # Set terminal to raw mode to read single characters
    tty.setraw(sys.stdin.fileno())
    # Wait up to 0.1s for a key press
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    # Save current terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init(args=args)
    node = rclpy.create_node('custom_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5  # Initial Linear speed (m/s)
    turn = 1.0   # Initial Angular speed (rad/s)
    x = 0.0      # Current linear direction
    th = 0.0     # Current angular direction

    try:
        print(msg)
        print(f"Currently: speed {speed} | turn {turn}")
        
        while rclpy.ok():
            key = getKey(settings)
            
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                if key == 's':
                    print("Stop command received.")
            
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print(f"Speed updated: speed {speed:.2f} | turn {turn:.2f}")
            
            elif key == '\x03':  # CTRL-C
                break
            
            # Create and publish the Twist message
            twist = Twist()
            twist.linear.x = float(x * speed)
            twist.angular.z = float(th * turn)
            pub.publish(twist)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        # Publish a final stop message before exiting
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)
        
        # Restore terminal settings so your terminal isn't broken
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()