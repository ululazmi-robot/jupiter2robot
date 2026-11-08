#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gtts import gTTS
import os
import tempfile

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Subscriber waiting for text to speak out loud
        self.subscription = self.create_subscription(
            String,
            'speech_output',
            self.speak_callback,
            10
        )
        self.get_logger().info("TTS Node Started. Waiting for messages on '/speech_output'...")

    def speak_callback(self, msg):
        text = msg.data
        self.get_logger().info(f"Speaking: {text}")
        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()