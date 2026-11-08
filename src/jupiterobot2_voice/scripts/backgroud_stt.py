#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        
        self.command_pub = self.create_publisher(String, 'voice_commands', 10)
        
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # 1. Calibrate for background noise once on startup
        self.get_logger().info("Calibrating microphone for ambient noise... Please wait 2 seconds.")
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
            
            # You can manually set how loud a sound needs to be to trigger listening.
            # Higher number = less sensitive to background noise. (Default is around 300)
            # self.recognizer.energy_threshold = 400 

        self.get_logger().info("STT Node Ready! I will only process audio when I hear a voice.")

        # 2. Start listening in the background. 
        # This returns a function we can call later to stop the listening thread.
        self.stop_listening = self.recognizer.listen_in_background(
            self.microphone, 
            self.audio_callback  # The function to call when a phrase is detected
        )

    def audio_callback(self, recognizer, audio):
        """This function ONLY runs when the microphone detects a complete spoken phrase."""
        self.get_logger().info("Sound detected! Sending to Google...")
        try:
            text = recognizer.recognize_google(audio).lower()
            self.get_logger().info(f"Recognized: {text}")

            # Publish to ROS topic
            msg = String()
            msg.data = text
            self.command_pub.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().info("Heard noise, but couldn't understand any words.")
        except sr.RequestError as e:
            self.get_logger().error(f"Internet/Google API Error: {e}")
        except Exception as e:
            self.get_logger().error(f"STT Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STTNode()
    
    try:
        # rclpy.spin keeps the node alive while the background thread does the listening
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanly shut down the background listening thread when we kill the node
        node.stop_listening(wait_for_stop=False)
        rclpy.shutdown()

if __name__ == '__main__':
    main()