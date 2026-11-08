#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from gtts import gTTS
import os
import tempfile
import psutil
import csv
import time
import sys
from datetime import datetime

try:
    from pynvml import *
    nvmlInit()
    HAS_GPU = True
except:
    HAS_GPU = False

class VipManager(Node):
    def __init__(self):
        super().__init__('vip_manager')
        
        # State Management
        self.is_busy = False
        self.target_location = None
        self.previous_location = None  
        self.current_location = None   
        
        self.stt_latency = 0.0 
        self.nav_start_time = 0.0
        
        self.nav = BasicNavigator()
        
        # --- UPDATED COORDINATES FROM SECOND SCRIPT ---
        self.locations = {
            "kitchen":     self.create_pose(-5.05, -1.06, 90.0),
            "living room": self.create_pose(-2.09, -0.358, 180.0),
            "home":        self.create_pose(0.0816, -0.039, 0.0)
        }

        # --- UPDATED LOGGING DIRECTORY ---
        self.log_dir = "/home/ubuntu/fyp1/src/experiment_log"
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        filename = f'robot_experiment_distributed_{datetime.now().strftime("%Y%m%d_%H%M%S")}.tsv'
        self.tsv_filepath = os.path.join(self.log_dir, filename)
        
        self.init_tsv()
        
        # Subscriber to listen to your laptop
        self.command_sub = self.create_subscription(
            String,
            '/voice_commands',
            self.voice_topic_callback,
            10)

        # Navigation monitor timer (Every 0.5 seconds)
        self.nav_monitor_timer = self.create_timer(0.5, self.monitor_navigation) 

        self.get_logger().info(f"VIP Manager (Distributed) Ready. Coordinates updated. Listening to /voice_commands")
        self.speak("Distributed system online. Waiting for laptop commands.")

    def speak(self, text):
        try:
            self.get_logger().info(f"Robot speaking: {text}")
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

    def create_pose(self, x, y, yaw_deg):
        import math
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        yaw_rad = math.radians(yaw_deg)
        pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
        return pose

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
            except: pass
        
        with open(self.tsv_filepath, 'a', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow([timestamp, event_name, cpu, ram, gpu, 
                             round(duration, 3), round(lat, 3), round(ow_lat, 3), round(tot_lat, 3)])

    def voice_topic_callback(self, msg):
        if self.is_busy:
            return

        text = msg.data.lower()
        self.get_logger().info(f"Received Topic Command: {text}")
        
        # Shutdown Keywords
        if "i am done" in text or "kill the program" in text:
            self.log_event("SHUTDOWN_TRIGGERED")
            self.speak("Experiment finished. Shutting down.")
            if HAS_GPU:
                nvmlShutdown()
            rclpy.shutdown()
            sys.exit(0)

        # Previous Location Logic
        if "previous location" in text:
            if self.previous_location:
                self.log_event("DETECTED_PREVIOUS")
                self.target_location = self.previous_location
                self.is_busy = True
                return
            else:
                self.speak("No memory of previous location.")
                return

        # Normal Location detection
        for place in self.locations.keys():
            if place in text:
                self.log_event(f"DETECTED_{place.upper()}")
                self.target_location = place
                self.is_busy = True 
                break

    def monitor_navigation(self):
        if not self.is_busy:
            return

        # Start Move Phase
        if self.target_location and self.nav_start_time == 0.0:
            self.speak(f"I am going to the {self.target_location}.")
            self.nav_start_time = time.time()
            self.nav.goToPose(self.locations[self.target_location])
            return

        # Monitoring Phase
        if self.nav_start_time > 0.0:
            if not self.nav.isTaskComplete():
                return 
            
            end_time = time.time()
            time_taken = end_time - self.nav_start_time
            
            # Simplified latency logging for distributed setup
            oneway_lat = 0.2 # Estimated network jitter
            total_lat = time_taken

            result = self.nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Arrived at {self.target_location}!")
                self.log_event(f"REACHED_{self.target_location.upper()}", 
                               time_taken, 0.0, oneway_lat, total_lat)
                self.speak(f"I have reached the {self.target_location}.")
                
                if self.current_location:
                    self.previous_location = self.current_location
                self.current_location = self.target_location
                
            else:
                self.log_event("NAV_FAILED")
                self.speak("Navigation failed.")

            self.is_busy = False
            self.target_location = None
            self.nav_start_time = 0.0

def main():
    rclpy.init()
    node = VipManager()
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