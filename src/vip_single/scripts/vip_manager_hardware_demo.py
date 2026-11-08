#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

import speech_recognition as sr
from gtts import gTTS
import os
import tempfile
import psutil
import csv
import time
import sys
import re
import threading
import math
from datetime import datetime

try:
    from pynvml import *
    nvmlInit()
    HAS_GPU = True
except Exception:
    HAS_GPU = False

class VipManualTimeMotionManager(Node):
    def __init__(self):
        super().__init__('vip_manual_time_motion_manager')
        
        # State Management
        self.is_busy = False
        self.stt_latency = 0.0
        
        # ROS 2 Velocity Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # TSV Logging Setup
        self.log_dir = "/home/ubuntu/fyp1/src/experiment_log"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        filename = f'robot_time_based_motion_{datetime.now().strftime("%Y%m%d_%H%M%S")}.tsv'
        self.tsv_filepath = os.path.join(self.log_dir, filename)
        self.init_tsv()
        
        # Background STT Setup
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
        
        self.get_logger().info(f"VIP Time-Based Motion Node Active. Logging to: {self.tsv_filepath}")
        self.speak("System arm. Ready for movement commands.")

        # Non-blocking background listener
        self.stop_listening = self.recognizer.listen_in_background(self.mic, self.voice_callback)

    def speak(self, text):
        try:
            self.get_logger().info(f"Robot speaking: {text}")
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

    def init_tsv(self):
        header = [
            'timestamp', 'event', 'cpu_%', 'ram_%', 'gpu_%', 
            'time_taken_sec', 'latency_sec', 'oneway_latency_sec', 'total_latency_sec'
        ]
        with open(self.tsv_filepath, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(header)

    def log_event(self, event_name, duration=0.0, lat=0.0, ow_lat=0.0, tot_lat=0.0):
        timestamp = datetime.now().strftime("%H:%M:%S")
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        gpu = 0.0
        if HAS_GPU:
            try:
                handle = nvmlDeviceGetHandleByIndex(0)
                gpu = nvmlDeviceGetUtilizationRates(handle).gpu
            except Exception: 
                pass
        
        with open(self.tsv_filepath, 'a', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow([timestamp, event_name, cpu, ram, gpu, 
                             round(duration, 3), round(lat, 3), round(ow_lat, 3), round(tot_lat, 3)])

    def voice_callback(self, recognizer, audio):
        if self.is_busy:
            return

        try:
            start_stt = time.time()
            text = recognizer.recognize_google(audio).lower()
            self.stt_latency = time.time() - start_stt
            
            self.get_logger().info(f"Recognized Voice Token: {text}")

            # Shutdown Keywords
            if "i am done" in text or "kill" in text:
                self.get_logger().warn("Shutdown command received.")
                self.log_event("SHUTDOWN_TRIGGERED", lat=self.stt_latency)
                self.speak("Experiment finished. Shutting down now.")
                
                self.stop_listening(wait_for_stop=False)
                if HAS_GPU:
                    nvmlShutdown()
                rclpy.shutdown()
                sys.exit(0)

            # --- TIME-BASED PARSING BLOCK ---
            action_type = None
            duration_sec = 0.0
            direction = None

            # Matches digits followed by optional phrase variants of seconds/sec
            time_match = re.search(r"([\d.]+)\s*(?:second|seconds|sec|s)?", text)

            # Added "straight" to the forward parsing loop for dynamic conversational command support
            if "forward" in text or "straight" in text:
                duration_sec = float(time_match.group(1)) if time_match else 1.0 # Default to 1 sec
                action_type = "FORWARD"
                
            elif "reverse" in text or "backward" in text:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "REVERSE"

            elif "turn right" in text or "rotate right" in text:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "TURN"
                direction = "right"

            elif "turn left" in text or "rotate left" in text:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "TURN"
                direction = "left"

            elif "turn backward" in text or "turn around" in text:
                duration_sec = float(time_match.group(1)) if time_match else 2.0  # Default 2 seconds for a full 180 flip
                action_type = "TURN"
                direction = "right"

            # If an action match is confirmed, dispatch the thread execution loop
            if action_type:
                self.is_busy = True
                motion_thread = threading.Thread(
                    target=self.execute_threaded_motion, 
                    args=(action_type, duration_sec, direction)
                )
                motion_thread.start()

        except sr.UnknownValueError:
            pass 
        except Exception as e:
            self.get_logger().error(f"Voice Callback Parsing Error: {e}")

    def execute_threaded_motion(self, action_type, duration_sec, direction=None):
        """ Runs inside a separate background thread so rclpy does not block """
        if action_type == "FORWARD":
            self.speak(f"Moving forward for {duration_sec} seconds.")
            self.log_event(f"START_FORWARD_{duration_sec}S", lat=self.stt_latency)
            
            start_move = time.time()
            self.move_forward(duration_sec, backward=False)
            elapsed = time.time() - start_move
            
            self.log_event(f"SUCCESS_FORWARD_{duration_sec}S", elapsed, self.stt_latency, self.stt_latency + 0.1, self.stt_latency + elapsed)

        elif action_type == "REVERSE":
            self.speak(f"Reversing for {duration_sec} seconds.")
            self.log_event(f"START_REVERSE_{duration_sec}S", lat=self.stt_latency)
            
            start_move = time.time()
            self.move_forward(duration_sec, backward=True)
            elapsed = time.time() - start_move
            
            self.log_event(f"SUCCESS_REVERSE_{duration_sec}S", elapsed, self.stt_latency, self.stt_latency + 0.1, self.stt_latency + elapsed)

        elif action_type == "TURN":
            self.speak(f"Turning {direction} for {duration_sec} seconds.")
            self.log_event(f"START_TURN_{direction.upper()}_{duration_sec}S", lat=self.stt_latency)
            
            start_move = time.time()
            self.turn(duration_sec, direction)
            elapsed = time.time() - start_move
            
            self.log_event(f"SUCCESS_TURN_{direction.upper()}_{duration_sec}S", elapsed, self.stt_latency, self.stt_latency + 0.1, self.stt_latency + elapsed)

        self.speak("Motion completed.")
        self.is_busy = False 

    # --- TIME-BASED ROS 2 MOVEMENT METHODS ---
    
    def turn(self, duration, direction):
        angular_speed = 0.5  # Fixed speed in rad/s

        turn_cmd = Twist()
        if direction == 'left':
            turn_cmd.angular.z = float(angular_speed)
        elif direction == 'right':
            turn_cmd.angular.z = float(-angular_speed)
        else:
            self.get_logger().error("Invalid direction.")
            return

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(turn_cmd)
            time.sleep(0.1)
            
        # Hard brake stop sequence
        turn_cmd.angular.z = 0.0
        self.cmd_pub.publish(turn_cmd)

    def move_forward(self, duration, backward=False):
        if backward:
            speed = -0.05  # m/s backward
        else:
            speed = 0.05   # m/s forward
        
        move_cmd = Twist()
        move_cmd.linear.x = float(speed)

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(move_cmd)
            time.sleep(0.1)
            
        # Hard brake stop sequence
        move_cmd.linear.x = 0.0
        self.cmd_pub.publish(move_cmd)

def main():
    rclpy.init()
    node = VipManualTimeMotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_listening(wait_for_stop=False)
            if HAS_GPU:
                nvmlShutdown()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 