#!/usr/bin/env python3
"""
minimal_motion_node.py (ROS 2 / rclpy version)

A minimal ROS 2 node that isolates open-loop `turn` and `move_forward`
primitives for the Jupiter2 robot, with an empirically calibrated
correction factor applied to the turn duration to compensate for
real-world drift (acceleration lag, wheel slip, momentum, etc.).
"""

import math
import time
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class MotionController(Node):

  def __init__(self):
    super().__init__('minimal_motion_node')

    # Publisher for velocity commands
    self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # Flag set once a turn completes
    self.check_pos = False

    # --- Calibration ---
    # correction_factor = commanded_angle / actual_measured_angle
    #
    # Empirically determined via spin_for_duration() + turn() tests:
    # the robot's real angular speed at commanded 0.5 rad/s was
    # significantly lower than expected. Re-measure and update this
    # value any time you change surface, battery level, or payload.
    self.turn_correction_factor = 3.35

    # Allow publisher to establish connection with subscribers
    time.sleep(1.0)
    self.get_logger().info('Jupiter2 Minimal Motion Node Initialized.')

  def spin_for_duration(self, duration_sec, direction='left', angular_speed=0.5):
    """Spin the robot at a fixed commanded angular_speed for a fixed

    duration_sec, with no angle calculation involved. Used purely to
    measure the robot's true angular speed empirically.

    After running this, physically measure the actual angle rotated,
    then compute:
        real_angular_speed = math.radians(actual_measured_angle) / duration_sec
    """
    turn_cmd = Twist()
    if direction == 'left':
      turn_cmd.angular.z = angular_speed
    elif direction == 'right':
      turn_cmd.angular.z = -angular_speed
    else:
      self.get_logger().error("Invalid direction. Use 'left' or 'right'.")
      return

    self.get_logger().info(
        f'[spin_for_duration] Direction={direction}, '
        f'Commanded_speed={angular_speed} rad/s, Duration={duration_sec}s'
    )

    start_time = self.get_clock().now().nanoseconds / 1e9
    while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration_sec:
      self.cmd_pub.publish(turn_cmd)
      time.sleep(0.1)

    turn_cmd.angular.z = 0.0
    self.cmd_pub.publish(turn_cmd)
    self.get_logger().info('[spin_for_duration] Complete, Jupiter2 stopped.')

  def turn(self, angle_degrees, direction):
    """Rotate the robot in place by `angle_degrees` using open-loop timing,

    corrected by `self.turn_correction_factor` to compensate for
    real-world deviation from the commanded angular speed.

    direction: 'left' (+z, counter-clockwise) or 'right' (-z, clockwise)
    """
    angular_speed = 0.5  # rad/s (commanded)
    angle_radians = math.radians(angle_degrees)

    # Apply correction factor to the commanded angle before computing duration
    corrected_angle_radians = angle_radians * self.turn_correction_factor
    duration = corrected_angle_radians / angular_speed

    turn_cmd = Twist()
    if direction == 'left':
      turn_cmd.angular.z = angular_speed
    elif direction == 'right':
      turn_cmd.angular.z = -angular_speed
    else:
      self.get_logger().error("Invalid direction. Use 'left' or 'right'.")
      return

    self.get_logger().info(
        f'[turn] Direction={direction}, Commanded={angle_degrees} deg,'
        f' Corrected={math.degrees(corrected_angle_radians):.1f} deg,'
        f' Factor={self.turn_correction_factor:.3f},'
        f' Duration={duration:.2f}s'
    )

    start_time = self.get_clock().now().nanoseconds / 1e9
    while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
      self.cmd_pub.publish(turn_cmd)
      time.sleep(0.1)

    # Zero out velocity to stop rotation
    turn_cmd.angular.z = 0.0
    self.check_pos = True
    self.cmd_pub.publish(turn_cmd)
    self.get_logger().info('[turn] Complete, Jupiter2 stopped.')

  def move_forward(self, distance):
    """Drive the robot forward (positive distance) or backward (negative distance)

    at fixed linear speed using open-loop timing.
    """
    speed = -0.05 if distance < 0 else 0.05  # m/s
    duration = abs(distance) / abs(speed)

    move_cmd = Twist()
    move_cmd.linear.x = speed

    self.get_logger().info(
        f'[move_forward] Distance={distance}m, Speed={speed}m/s,'
        f' Duration={duration:.2f}s'
    )

    start_time = self.get_clock().now().nanoseconds / 1e9
    while (self.get_clock().now().nanoseconds / 1e9) - start_time < duration:
      self.cmd_pub.publish(move_cmd)
      time.sleep(0.1)

    # Zero out velocity to stop translation
    move_cmd.linear.x = 0.0
    self.cmd_pub.publish(move_cmd)
    self.get_logger().info('[move_forward] Complete, Jupiter2 stopped.')

  def calibrate_turn(self, commanded_angle, actual_measured_angle):
    """Update the turn correction factor from a calibration measurement.

    Run turn(commanded_angle, direction), physically measure how far the
    robot actually rotated, then call this method to update the factor.

    Example:
        controller.turn(90, 'left')
        # ...physically measure the actual angle...
        controller.calibrate_turn(90, actual_measured_angle)
    """
    if actual_measured_angle == 0:
      self.get_logger().error('actual_measured_angle cannot be 0.')
      return

    self.turn_correction_factor = commanded_angle / actual_measured_angle
    self.get_logger().info(
        f'[calibrate_turn] Commanded={commanded_angle} deg,'
        f' Measured={actual_measured_angle} deg,'
        f' New correction_factor={self.turn_correction_factor:.4f}'
    )

  def run_demo(self):
    """Scripted motion sequence to test primitives on Jupiter2."""
    self.get_logger().info('Starting Jupiter2 motion test sequence...')

    self.move_forward(-0.1)  # Back up 0.1 m
    time.sleep(1.0)

    self.turn(145, 'left')  # Rotate 145 deg left
    time.sleep(1.0)

    self.turn(145, 'right')  # Rotate 145 deg right back
    time.sleep(1.0)

    self.move_forward(0.1)  # Move forward 0.1 m
    self.get_logger().info('Jupiter2 motion test sequence completed successfully.')


def main(args=None):
  rclpy.init(args=args)
  controller = MotionController()

  try:
    controller.run_demo()
  except KeyboardInterrupt:
    pass
  finally:
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()