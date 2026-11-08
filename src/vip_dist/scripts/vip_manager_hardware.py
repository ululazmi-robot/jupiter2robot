#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from gtts import gTTS
import os, tempfile, psutil, csv, time, sys
from datetime import datetime

class DistributedVipManager(Node):
    def __init__(self):
        super().__init__('vip_manager_distributed')
        
        # --- CONFIGURATION ---
        self.max_trials = 50 
        self.trial_count = 0
        self.state = "IDLE" 
        self.is_busy = False
        
        # This will store the latency sent from your Jetson
        self.recorded_latency = 0.0 
        
        self.nav_start_time = 0.0
        self.wait_start_time = 0.0
        self.target_location = None
        
        self.nav = BasicNavigator()
        
        # Coordinates mapped for Gazebo Simulation
        self.locations = {
            "kitchen":     self.create_pose(-1.29, 1.23, 270.0),
            "living room": self.create_pose(-0.032, -0.00764, 270.0),
            "bedroom":     self.create_pose(-2.49, 0.0478, 270.0), 
            "study":       self.create_pose(-1.4, -1.38, 0.0), 
            "home":        self.create_pose(-0.2, -0.537, 0.0)
        }

        # TSV Logging Setup
        self.log_dir = "/home/ubuntu/fyp1/src/experiment_log2"
        if not os.path.exists(self.log_dir): 
            os.makedirs(self.log_dir)
        filename = f'dist_50_trials_{datetime.now().strftime("%Y%m%d_%H%M%S")}.tsv'
        self.tsv_filepath = os.path.join(self.log_dir, filename)
        self.init_tsv()
        
        # Subscribers and Timers
        self.command_sub = self.create_subscription(String, '/voice_commands', self.voice_topic_callback, 10)
        self.nav_monitor_timer = self.create_timer(0.5, self.monitor_navigation) 

        self.get_logger().info("VIP Distributed System Manager: 50-Trial Loop Mode Active")
        self.speak("Distributed system ready Manager.Speak a location to start a 50 trial session")

    def init_tsv(self):
        header = ['timestamp', 'trial_no', 'location', 'cpu_%', 'ram_%', 'time_taken_sec', 'stt_latency_sec']
        with open(self.tsv_filepath, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(header)

    def log_trial(self, trial_no, location, duration):
        timestamp = datetime.now().strftime("%H:%M:%S")
        cpu = psutil.cpu_percent()
        ram = psutil.virtual_memory().percent
        with open(self.tsv_filepath, 'a', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            # Uses the latency parsed from the initial incoming command
            writer.writerow([timestamp, trial_no, location, cpu, ram, 
                             round(duration, 3), round(self.recorded_latency, 3)])

    def voice_topic_callback(self, msg):
        if self.is_busy: 
            return
        
        # Expecting message format "location:latency" (e.g., "kitchen:0.855")
        try:
            data = msg.data.lower()
            if ":" in data:
                location_part, latency_part = data.split(":")
                self.recorded_latency = float(latency_part)
            else:
                location_part = data
                self.recorded_latency = 0.0 # Fallback if no latency payload exists
            
            for place in self.locations.keys():
                if place in location_part and place != "home":
                    self.target_location = place
                    self.trial_count = 1
                    self.is_busy = True 
                    self.state = "MOVING_TO_TARGET"
                    self.get_logger().info(f"Starting 50 automated trials for {self.target_location.upper()}. Network STT Latency: {self.recorded_latency}s")
                    break
        except Exception as e:
            self.get_logger().error(f"Callback processing error: {e}")

    def monitor_navigation(self):
        if not self.is_busy: 
            return

        # PHASE 1: GO TO TARGET
        if self.state == "MOVING_TO_TARGET" and self.nav_start_time == 0.0:
            self.get_logger().info(f"Trial {self.trial_count}/{self.max_trials}: Heading to {self.target_location}")
            self.nav_start_time = time.time()
            self.nav.goToPose(self.locations[self.target_location])
            return

        # PHASE 2: ARRIVAL & LOGGING
        if self.state == "MOVING_TO_TARGET" and self.nav_start_time > 0.0:
            if not self.nav.isTaskComplete(): 
                return 
            
            duration = time.time() - self.nav_start_time
            if self.nav.getResult() == TaskResult.SUCCEEDED:
                # Log stats only for the active trip
                self.log_trial(self.trial_count, self.target_location, duration)
                self.state = "WAITING"
                self.wait_start_time = time.time()
            else:
                self.get_logger().error("Navigation Failed. Retrying this trial iteration.")
                self.nav_start_time = 0.0 

        # PHASE 3: 3-SECOND DELAY AT TARGET
        if self.state == "WAITING":
            if (time.time() - self.wait_start_time) >= 3.0:
                self.state = "RETURNING_HOME"
                self.nav_start_time = 0.0
            return

        # PHASE 4: GO HOME (SILENT / NO LOGGING)
        if self.state == "RETURNING_HOME" and self.nav_start_time == 0.0:
            self.nav_start_time = time.time()
            self.nav.goToPose(self.locations["home"])
            return

        # PHASE 5: CHECK IF FINISHED OR LOOP AGAIN
        if self.state == "RETURNING_HOME" and self.nav_start_time > 0.0:
            if not self.nav.isTaskComplete(): 
                return 

            if self.trial_count < self.max_trials:
                self.trial_count += 1
                self.state = "MOVING_TO_TARGET"
                self.nav_start_time = 0.0
                # STT Latency is only relevant for the 1st voice trigger; reset for automated loops
                self.recorded_latency = 0.0 
            else:
                self.speak(f"Finished fifty trials for {self.target_location}")
                self.is_busy = False
                self.state = "IDLE"

    def create_pose(self, x, y, yaw_deg):
        import math
        pose = PoseStamped()
        pose.header.frame_id, pose.header.stamp = 'map', self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        yaw_rad = math.radians(yaw_deg)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)
        return pose

    def speak(self, text):
        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name}")
        except Exception: 
            pass

def main():
    rclpy.init()
    node = DistributedVipManager()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt: 
        pass
    finally: 
        rclpy.shutdown()

if __name__ == '__main__': 
    main()