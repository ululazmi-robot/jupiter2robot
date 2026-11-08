#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import time


class OdomRobot(Node):

    def __init__(self):
        super().__init__('odom_robot')

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.get_logger().info("Odom Robot Controller Started")

    # ---------------- ODOM CALLBACK ----------------
    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # quaternion to yaw
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(siny, cosy)

    # ---------------- STOP ----------------
    def stop(self):
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)

    # ---------------- MOVE (ODOM CLOSED LOOP) ----------------
    def move(self, distance):

        start_x = self.x
        start_y = self.y

        target_dist = abs(distance)

        direction = 1.0 if distance > 0 else -1.0

        msg = Twist()
        msg.linear.x = 0.1 * direction

        self.get_logger().info(f"Moving {distance} meters")

        while rclpy.ok():

            dx = self.x - start_x
            dy = self.y - start_y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist >= target_dist:
                break

            self.cmd_pub.publish(msg)
            time.sleep(0.02)

        self.stop()

    # ---------------- TURN (ODOM CLOSED LOOP) ----------------
    def turn(self, angle_deg, direction):

        target_angle = math.radians(angle_deg)

        start_yaw = self.yaw

        if direction == "left":
            target_yaw = start_yaw + target_angle
            speed = 0.3
        elif direction == "right":
            target_yaw = start_yaw - target_angle
            speed = -0.3
        else:
            self.get_logger().error("Use left/right")
            return

        msg = Twist()
        msg.angular.z = speed

        self.get_logger().info(f"Turning {angle_deg} degrees {direction}")

        while rclpy.ok():

            error = target_yaw - self.yaw

            # normalize angle (-pi to pi)
            error = math.atan2(math.sin(error), math.cos(error))

            if abs(error) < 0.01:   # very accurate stop
                break

            self.cmd_pub.publish(msg)
            time.sleep(0.01)

        self.stop()


# ---------------- TERMINAL INTERFACE ----------------
def main(args=None):
    rclpy.init(args=args)

    node = OdomRobot()

    try:
        while rclpy.ok():

            rclpy.spin_once(node, timeout_sec=0.1)

            cmd = input("\n(f 1 / t 90 left / q): ")
            parts = cmd.split()

            if len(parts) == 0:
                continue

            if parts[0] == "q":
                break

            elif parts[0] == "f":
                node.move(float(parts[1]))

            elif parts[0] == "t":
                node.turn(float(parts[1]), parts[2])

            else:
                print("Invalid command")

    except KeyboardInterrupt:
        pass

    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()