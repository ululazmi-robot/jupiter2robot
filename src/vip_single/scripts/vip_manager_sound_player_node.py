#!/usr/bin/env python3
"""
sound_player_node.py (ROS 2 / rclpy version)

A minimal, standalone ROS 2 node dedicated to playing audio files
using mpg321, pointing directly at the source sounds directory.
"""

import os
import rclpy
from rclpy.node import Node


class SoundPlayer(Node):

  def __init__(self):
    super().__init__('sound_player_node')
    self.sounds_dir = '/home/ubuntu/fyp1/src/vip_single/sounds'
    self.get_logger().info('Sound Player Node Initialized.')

  def play_sound(self, file_name):
    """Play an audio file located directly in the source sounds directory.

    file_name: name of the audio file (e.g. 'test.mp3').
    """
    sound_path = os.path.join(self.sounds_dir, file_name)

    if not os.path.isfile(sound_path):
      self.get_logger().error(f'[play_sound] File not found: {sound_path}')
      return

    self.get_logger().info(f'[play_sound] Playing: {sound_path}')
    result = os.system(f'mpg321 "{sound_path}"')

    if result != 0:
      self.get_logger().error(
          f'[play_sound] mpg321 exited with code {result}. '
          'Is mpg321 installed? Try: sudo apt install mpg321'
      )


def main(args=None):
  rclpy.init(args=args)
  player = SoundPlayer()

  try:
    player.play_sound('test.mp3')
  except KeyboardInterrupt:
    pass
  finally:
    player.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()