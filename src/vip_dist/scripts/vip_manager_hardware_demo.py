#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from gtts import gTTS
import os
import tempfile
import psutil
import csv
import time
import sys
import re
import threading
from datetime import datetime

# NVML library configuration to dynamically query GPU utilization profiles if present
try:
    from pynvml import *
    nvmlInit()
    HAS_GPU = True
except Exception:
    HAS_GPU = False

class VipDistTimeMotionManager(Node):
    def __init__(self):
        super().__init__('vip_dist_time_motion_manager')
        
        # --- STATE MANAGEMENT & COMM VARIABLES ---
        self.is_busy = False
        self.recorded_latency = 0.0  # Holds the STT latency transmitted from the Jetson node
        
        # --- DDS COMMS CONSTANTS (From Experiment 1 & 2 Latency Benchmarks) ---
        self.oneway_latency = 0.083  # Stable 5 GHz Wi-Fi DDS transmission baseline (83ms)
        
        # --- ROS 2 VELOCITY PUBLISHER & VOICE TOPIC SUBSCRIBER ---
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.command_sub = self.create_subscription(
            String, 
            '/voice_commands', 
            self.voice_topic_callback, 
            10
        )

        # --- TSV LOGGING SETUP ---
        self.log_dir = "/home/ubuntu/fyp1/src/experiment_log"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        filename = f'dist_robot_time_motion_{datetime.now().strftime("%Y%m%d_%H%M%S")}.tsv'
        self.tsv_filepath = os.path.join(self.log_dir, filename)
        self.init_tsv()
        
        self.get_logger().info("==================================================================")
        self.get_logger().info("VIP DISTRIBUTED TIME-BASED MOTION NODE INITIALIZED")
        self.get_logger().info(f"Subscribed to topic: /voice_commands")
        self.get_logger().info(f"Logging experimental telemetry directly to: {self.tsv_filepath}")
        self.get_logger().info("==================================================================")
        
        self.speak("Distributed system ready. Listening to voice command topic channel.")

    def speak(self, text):
        """Generates clear, diagnostic text-to-speech feedback using gTTS played via mpg123"""
        try:
            self.get_logger().info(f"Robot speaking: {text}")
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception as e:
            self.get_logger().error(f"TTS Synthesis Error: {e}")

    def init_tsv(self):
        """Prepares the telemetry TSV sheet with exact variables for evaluation"""
        header = [
            'timestamp', 'event', 'cpu_%', 'ram_%', 'gpu_%', 
            'time_taken_sec', 'latency_sec', 'oneway_latency_sec', 'total_latency_sec'
        ]
        with open(self.tsv_filepath, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(header)

    def log_event(self, event_name, duration=0.0, lat=0.0, ow_lat=0.0, tot_lat=0.0):
        """Captures hardware stats directly from the Linux kernel using psutil"""
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
            writer.writerow([
                timestamp, event_name, cpu, ram, gpu, 
                round(duration, 3), round(lat, 3), round(ow_lat, 3), round(tot_lat, 3)
            ])

    def voice_topic_callback(self, msg):
        """Processes raw incoming commands offloaded from your Jetson orin nano"""
        if self.is_busy:
            self.get_logger().warn("Command received but ignored as system is currently executing motion.")
            return

        # Expecting string formatting: "command_text:stt_latency" (e.g., "turn left 3 seconds:0.718")
        try:
            data = msg.data.lower()
            if ":" in data:
                command_part, latency_part = data.split(":")
                self.recorded_latency = float(latency_part)
            else:
                command_part = data
                self.recorded_latency = 0.0  # Safe baseline fallback
            
            self.get_logger().info(f"Extracted Voice Token: '{command_part}' (Jetson STT Latency: {self.recorded_latency}s)")

            # Diagnostic Exit Command Handler
            if "i am done" in command_part or "kill the program" in command_part:
                self.get_logger().warn("Distributed Shutdown execution triggered.")
                self.log_event("SHUTDOWN_TRIGGERED", lat=self.recorded_latency)
                self.speak("Experiment finished. Shutting down now.")
                
                if HAS_GPU:
                    nvmlShutdown()
                rclpy.shutdown()
                sys.exit(0)

            # --- REGEX MOTION PARSING ---
            action_type = None
            duration_sec = 0.0
            direction = None

            # Look for number patterns representing movement duration bounds
            time_match = re.search(r"([\d.]+)\s*(?:second|seconds|sec|s)?", command_part)

            if "forward" in command_part:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "FORWARD"
                
            elif "reverse" in command_part or "backward" in command_part:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "REVERSE"

            elif "turn right" in command_part or "rotate right" in command_part:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "TURN"
                direction = "right"

            elif "turn left" in command_part or "rotate left" in command_part:
                duration_sec = float(time_match.group(1)) if time_match else 1.0
                action_type = "TURN"
                direction = "left"

            elif "turn backward" in command_part or "turn around" in command_part:
                duration_sec = float(time_match.group(1)) if time_match else 2.0  # 180-degree flip
                action_type = "TURN"
                direction = "right"

            # Run motion loops inside a dedicated, non-blocking background thread
            if action_type:
                self.is_busy = True
                motion_thread = threading.Thread(
                    target=self.execute_threaded_motion, 
                    args=(action_type, duration_sec, direction)
                )
                motion_thread.start()
            else:
                self.get_logger().warn(f"Unable to parse command parameters from phrase: '{command_part}'")

        except Exception as e:
            self.get_logger().error(f"Voice Callback Processing Error: {e}")

    def execute_threaded_motion(self, action_type, duration_sec, direction=None):
        """Asynchronously handles velocity loops without blocking the main ROS Executor"""
        
        if action_type == "FORWARD":
            self.speak(f"Moving forward for {duration_sec} seconds.")
            self.log_event(
                f"START_FORWARD_{duration_sec}S", 
                lat=self.recorded_latency,
                ow_lat=self.oneway_latency,
                tot_lat=self.recorded_latency + self.oneway_latency
            )
            
            start_move = time.time()
            self.move_forward(duration_sec, backward=False)
            elapsed = time.time() - start_move
            
            self.log_event(
                f"SUCCESS_FORWARD_{duration_sec}S", 
                duration=elapsed, 
                lat=self.recorded_latency, 
                ow_lat=self.oneway_latency, 
                tot_lat=self.recorded_latency + self.oneway_latency + elapsed
            )

        elif action_type == "REVERSE":
            self.speak(f"Reversing for {duration_sec} seconds.")
            self.log_event(
                f"START_REVERSE_{duration_sec}S", 
                lat=self.recorded_latency,
                ow_lat=self.oneway_latency,
                tot_lat=self.recorded_latency + self.oneway_latency
            )
            
            start_move = time.time()
            self.move_forward(duration_sec, backward=True)
            elapsed = time.time() - start_move
            
            self.log_event(
                f"SUCCESS_REVERSE_{duration_sec}S", 
                duration=elapsed, 
                lat=self.recorded_latency, 
                ow_lat=self.oneway_latency, 
                tot_lat=self.recorded_latency + self.oneway_latency + elapsed
            )

        elif action_type == "TURN":
            self.speak(f"Turning {direction} for {duration_sec} seconds.")
            self.log_event(
                f"START_TURN_{direction.upper()}_{duration_sec}S", 
                lat=self.recorded_latency,
                ow_lat=self.oneway_latency,
                tot_lat=self.recorded_latency + self.oneway_latency
            )
            
            start_move = time.time()
            self.turn(duration_sec, direction)
            elapsed = time.time() - start_move
            
            self.log_event(
                f"SUCCESS_TURN_{direction.upper()}_{duration_sec}S", 
                duration=elapsed, 
                lat=self.recorded_latency, 
                ow_lat=self.oneway_latency, 
                tot_lat=self.recorded_latency + self.oneway_latency + elapsed
            )

        self.speak("Motion completed.")
        self.is_busy = False 

    # --- TIME-BASED ROS 2 MOVEMENT METHODS ---
    
    def turn(self, duration, direction):
        angular_speed = 0.5  # rad/s turning velocity limit

        turn_cmd = Twist()
        if direction == 'left':
            turn_cmd.angular.z = float(angular_speed)
        elif direction == 'right':
            turn_cmd.angular.z = float(-angular_speed)
        else:
            self.get_logger().error("Invalid direction parameters.")
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
            speed = -0.05  # m/s backward speed limit
        else:
            speed = 0.05   # m/s forward speed limit
        
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
    node = VipDistTimeMotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            if HAS_GPU:
                nvmlShutdown()
            rclpy.shutdown()

if __name__ == '__main__':
    main()