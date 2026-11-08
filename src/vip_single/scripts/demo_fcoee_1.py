#!/usr/bin/env python3
"""
demo_fcoee_1.py (ROS 2 / rclpy version)

A combined ROS 2 node for the Jupiter2 robot that provides:
  - Open-loop `turn` and `move_forward` motion primitives, with an
    empirically calibrated correction factor for turn accuracy.
  - A `play_sound` method for playing audio files via mpg321,
    non-blocking so sound can play *while* the robot moves.
"""

import math
import os
import subprocess
import time
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class RobotController(Node):

  def __init__(self):
    super().__init__('vip_manager_node')

    # Publisher for velocity commands
    self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # Flag set once a turn completes
    self.check_pos = False

    # --- Motion calibration ---
    # correction_factor = commanded_angle / actual_measured_angle
    #
    # Empirically determined via spin_for_duration() + turn() tests:
    # the robot's real angular speed at commanded 0.5 rad/s was
    # significantly lower than expected. Re-measure and update this
    # value any time you change surface, battery level, or payload.
    self.turn_correction_factor = 3.35

    # --- Sound setup ---
    self.sounds_dir = '/home/ubuntu/fyp1/src/vip_single/sounds'

    # Keep track of any currently playing sound process
    self.sound_process = None

    # Allow publisher to establish connection with subscribers
    time.sleep(1.0)
    self.get_logger().info('Jupiter2 Robot Controller Node Initialized.')

  # ------------------------------------------------------------------
  # Motion primitives
  # ------------------------------------------------------------------

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

    move_cmd.linear.x = 0.0
    self.cmd_pub.publish(move_cmd)
    self.get_logger().info('[move_forward] Complete, Jupiter2 stopped.')

  def calibrate_turn(self, commanded_angle, actual_measured_angle):
    """Update the turn correction factor from a calibration measurement.

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

  # ------------------------------------------------------------------
  # Sound (non-blocking)
  # ------------------------------------------------------------------

  def play_sound(self, file_name):
    """Start playing an audio file in the background (non-blocking).

    Returns immediately so motion commands can run at the same time
    as the sound. file_name: name of the audio file (e.g. 'test.mp3').
    """
    sound_path = os.path.join(self.sounds_dir, file_name)

    if not os.path.isfile(sound_path):
      self.get_logger().error(f'[play_sound] File not found: {sound_path}')
      return

    self.get_logger().info(f'[play_sound] Playing (background): {sound_path}')
    try:
      self.sound_process = subprocess.Popen(
          ['mpg321', sound_path],
          stdout=subprocess.DEVNULL,
          stderr=subprocess.DEVNULL,
      )
    except FileNotFoundError:
      self.get_logger().error(
          '[play_sound] mpg321 not found. Install it with: sudo apt install mpg321'
      )

  def stop_sound(self):
    """Stop any currently playing sound started by play_sound()."""
    if self.sound_process is not None and self.sound_process.poll() is None:
      self.sound_process.terminate()
      self.get_logger().info('[stop_sound] Sound stopped.')
    self.sound_process = None

  # ------------------------------------------------------------------
  # Demo sequence
  # ------------------------------------------------------------------

  def run_demo(self):
    """Scripted motion + sound sequence to test primitives on Jupiter2.

    Sound starts playing in the background, then motion runs
    concurrently instead of waiting for the sound to finish.
    """
    self.get_logger().info('Starting Jupiter2 test sequence...')

    self.play_sound('test.mp3')  # starts playing, does not block

    self.move_forward(0.1)
    time.sleep(0.5)

    self.turn(25, 'left')
    time.sleep(0.5)

    self.turn(50, 'right')
    time.sleep(0.5)

    self.turn(25, 'left')
    time.sleep(0.5)

    self.move_forward(-0.1)
    self.turn(50, 'right')
    time.sleep(0.5)

    self.turn(100, 'left')
    time.sleep(0.5)

    self.turn(50, 'right')
    time.sleep(0.5)

    self.get_logger().info('Jupiter2 test sequence completed successfully.')


def main(args=None):
  rclpy.init(args=args)
  controller = RobotController()

  try:
    controller.run_demo()
  except KeyboardInterrupt:
    pass
  finally:
    controller.stop_sound()
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()