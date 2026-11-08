#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
from gtts import gTTS
import os
import tempfile

class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')
        
        # Publisher to send text commands to navigation
        self.command_pub = self.create_publisher(String, 'voice_commands', 10)
        
        # Audio tools
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        self.get_logger().info("Jupiter Voice Node Started. Ready to listen!")
        self.speak("Voice system initialized.")
        
        # Timer to run the listening loop
        self.create_timer(1.0, self.voice_loop)

    def speak(self, text):
        """Converts text to speech and plays it."""
        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

    def voice_loop(self):
        """Listens for speech and publishes the recognized text."""
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            try:
                print("Listening...")
                audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=5)
                text = self.recognizer.recognize_google(audio).lower()
                
                self.get_logger().info(f"Recognized: {text}")

                # Publish to ROS topic
                msg = String()
                msg.data = text
                self.command_pub.publish(msg)

                # Optional: Robot repeats what it heard
                self.speak(f"You said {text}")

            except sr.UnknownValueError:
                pass # Silent/Noise
            except Exception as e:
                self.get_logger().error(f"Voice Loop Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()