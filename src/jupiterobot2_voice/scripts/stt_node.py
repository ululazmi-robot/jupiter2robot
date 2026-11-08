#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        
        # Publisher to send text commands to other nodes
        self.command_pub = self.create_publisher(String, 'voice_commands', 10)
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        self.get_logger().info("STT Node Started. Listening for speech...")
        self.create_timer(1.0, self.listen_loop)

    def listen_loop(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5) #change to self.recognizer.listen(source, timeout=10, phrase_time_limit=10) if want it litsen 10 second
                text = self.recognizer.recognize_google(audio).lower()
                
                self.get_logger().info(f"Recognized: {text}")

                # Publish to ROS topic
                msg = String()
                msg.data = text
                self.command_pub.publish(msg)

            except sr.UnknownValueError:
                pass # Silent/Noise
            except Exception as e:
                self.get_logger().error(f"STT Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()