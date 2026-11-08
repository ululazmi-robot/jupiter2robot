#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

import speech_recognition as sr
from gtts import gTTS
import os
import tempfile
import psutil
import csv
import time
import sys
from datetime import datetime

class VipManager(Node):
    def __init__(self):
        super().__init__('vip_manager')
        
        # State Management
        self.is_busy = False
        self.state = "IDLE" 
        self.target_location = None
        
        # --- LOOP COUNTER SETUP ---
        self.trial_count = 0
        self.max_trials = 50 
        
        self.stt_latency = 0.0
        self.nav_start_time = 0.0
        self.wait_start_time = 0.0
        
        self.nav = BasicNavigator()
        
        self.locations = {
            "kitchen":     self.create_pose(-1.29, 1.23, 270.0),
            "living room": self.create_pose(-0.032, -0.00764, 270.0),
            "bedroom":     self.create_pose(-2.49, 0.0478, 270.0), 
            "study":   self.create_pose(-1.4, -1.38, 0.0), 
            "home":        self.create_pose(-0.2, -0.537, 0.0)
        }


        self.log_dir = "/home/ubuntu/fyp1/src/experiment_log"
        if not os.path.exists(self.log_dir): os.makedirs(self.log_dir)
            
        filename = f'robot_50_trial_test_{datetime.now().strftime("%Y%m%d_%H%M%S")}.tsv'
        self.tsv_filepath = os.path.join(self.log_dir, filename)
        self.init_tsv()
        
        self.nav_monitor_timer = self.create_timer(0.5, self.monitor_navigation) 
        self.recognizer = sr.Recognizer()
        self.mic = sr.Microphone()
        
        with self.mic as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=2.0)
        
        self.get_logger().info(f"VIP Manager: 50-Trial Loop Mode Active.")
        self.speak("Ready. Speak a location to start a 50 trial session.")
        self.stop_listening = self.recognizer.listen_in_background(self.mic, self.voice_callback)

    def speak(self, text):
        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception as e: self.get_logger().error(f"TTS Error: {e}")

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
        header = ['timestamp', 'trial_no', 'location', 'cpu_%', 'ram_%', 'time_taken_sec', 'stt_latency_sec']
        with open(self.tsv_filepath, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(header)

    def log_trial(self, trial_no, location, duration, lat):
        timestamp = datetime.now().strftime("%H:%M:%S")
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        with open(self.tsv_filepath, 'a', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow([timestamp, trial_no, location, cpu, ram, round(duration, 3), round(lat, 3)])

    def voice_callback(self, recognizer, audio):
        if self.is_busy: return
        try:
            start_stt = time.time()
            text = recognizer.recognize_google(audio).lower()
            self.stt_latency = time.time() - start_stt
            
            if "kill" in text or "done" in text:
                self.speak("Shutting down.")
                rclpy.shutdown(); sys.exit(0)

            for place in self.locations.keys():
                if place in text and place != "home":
                    self.get_logger().info(f"STARTING 50 TRIALS FOR: {place}")
                    self.target_location = place
                    self.trial_count = 1
                    self.is_busy = True 
                    self.state = "MOVING_TO_TARGET"
                    break
        except Exception: pass

    def monitor_navigation(self):
        if not self.is_busy: return

        # PHASE 1: GO TO TARGET
        if self.state == "MOVING_TO_TARGET" and self.nav_start_time == 0.0:
            self.get_logger().info(f"Trial {self.trial_count}/50: Heading to {self.target_location}")
            self.nav_start_time = time.time()
            self.nav.goToPose(self.locations[self.target_location])
            return

        # PHASE 2: ARRIVAL & LOGGING
        if self.state == "MOVING_TO_TARGET" and self.nav_start_time > 0.0:
            if not self.nav.isTaskComplete(): return 
            
            duration = time.time() - self.nav_start_time
            if self.nav.getResult() == TaskResult.SUCCEEDED:
                # LOG ONLY THIS TRIP
                self.log_trial(self.trial_count, self.target_location, duration, self.stt_latency)
                self.state = "WAITING"
                self.wait_start_time = time.time()
            else:
                self.get_logger().error("Nav Failed. Retrying trial.")
                self.nav_start_time = 0.0 # Retry this specific trial

        # PHASE 3: 10 SECOND WAIT
        if self.state == "WAITING":
            if (time.time() - self.wait_start_time) >= 3.0:
                self.state = "RETURNING_HOME"
                self.nav_start_time = 0.0
            return

        # PHASE 4: GO HOME (SILENT / NO LOG)
        if self.state == "RETURNING_HOME" and self.nav_start_time == 0.0:
            self.nav_start_time = time.time()
            self.nav.goToPose(self.locations["home"])
            return

        # PHASE 5: CHECK IF FINISHED OR LOOP AGAIN
        if self.state == "RETURNING_HOME" and self.nav_start_time > 0.0:
            if not self.nav.isTaskComplete(): return 

            if self.trial_count < self.max_trials:
                self.trial_count += 1
                self.state = "MOVING_TO_TARGET"
                self.nav_start_time = 0.0
                # STT Latency is only relevant for the 1st voice command
                self.stt_latency = 0.0 
            else:
                self.speak(f"Finished 50 trials for {self.target_location}")
                self.reset_session()

    def reset_session(self):
        self.is_busy = False
        self.state = "IDLE"
        self.trial_count = 0
        self.target_location = None
        self.nav_start_time = 0.0

def main():
    rclpy.init()
    node = VipManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()