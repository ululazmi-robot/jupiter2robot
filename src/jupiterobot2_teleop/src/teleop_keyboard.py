#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os

if os.name == 'nt':
    import msvcrt, time
else:
    import tty, termios

# Keeping your original settings
MAX_LIN_VEL = 0.26
MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Jupiterobot (ROS 2)!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit
"""

class JupiterTeleopKey(Node):
    def __init__(self):
        super().__init__('jupiter2_teleop_key')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        if os.name != 'nt':
            self.settings = termios.tcgetattr(sys.stdin)
        
        self.target_linear_vel = 0.0
        self.target_angular_vel = 0.0
        self.control_linear_vel = 0.0
        self.control_angular_vel = 0.0
        self.status = 0

    def getKey(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    return msvcrt.getch().decode()
                elif time.time() - startTime > timeout:
                    return ''
        
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def vels(self, target_linear_vel, target_angular_vel):
        return f"currently:\tlinear vel {target_linear_vel:.2f}\t angular vel {target_angular_vel:.2f}"

    def makeSimpleProfile(self, output, input_val, slop):
        if input_val > output:
            output = min(input_val, output + slop)
        elif input_val < output:
            output = max(input_val, output - slop)
        else:
            output = input_val
        return output

    def constrain(self, input_val, low, high):
        return max(min(input_val, high), low)

    def run_loop(self):
        try:
            print(msg)
            while rclpy.ok():
                key = self.getKey()
                if key == 'w':
                    self.target_linear_vel = self.constrain(self.target_linear_vel + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
                    self.status += 1
                    print(self.vels(self.target_linear_vel, self.target_angular_vel))
                elif key == 'x':
                    self.target_linear_vel = self.constrain(self.target_linear_vel - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL)
                    self.status += 1
                    print(self.vels(self.target_linear_vel, self.target_angular_vel))
                elif key == 'a':
                    self.target_angular_vel = self.constrain(self.target_angular_vel + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
                    self.status += 1
                    print(self.vels(self.target_linear_vel, self.target_angular_vel))
                elif key == 'd':
                    self.target_angular_vel = self.constrain(self.target_angular_vel - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL)
                    self.status += 1
                    print(self.vels(self.target_linear_vel, self.target_angular_vel))
                elif key == ' ' or key == 's':
                    self.target_linear_vel = 0.0
                    self.control_linear_vel = 0.0
                    self.target_angular_vel = 0.0
                    self.control_angular_vel = 0.0
                    print(self.vels(self.target_linear_vel, self.target_angular_vel))
                elif key == '\x03':
                    break

                if self.status == 20:
                    print(msg)
                    self.status = 0

                twist = Twist()
                self.control_linear_vel = self.makeSimpleProfile(self.control_linear_vel, self.target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = self.control_linear_vel
                self.control_angular_vel = self.makeSimpleProfile(self.control_angular_vel, self.target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.z = self.control_angular_vel
                
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            # Stop the robot on exit
            twist = Twist()
            self.publisher_.publish(twist)
            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = JupiterTeleopKey()
    node.run_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()