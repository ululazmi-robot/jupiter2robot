#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String 
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from gtts import gTTS
import os, tempfile, psutil, csv, sys
from datetime import datetime

class SimVipManager(Node):
    def __init__(self):
        super().__init__('vip_manager_sim')
        
        # --- SIMULATION TIME CONFIG ---
        # Forces the node to sync with Gazebo's clock
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.max_trials = 50 
        self.trial_count = 0
        self.state = "IDLE" 
        self.is_busy = False
        self.recorded_latency = 0.0 
        
        self.nav_start_time = None
        self.wait_start_time = None
        self.target_location = None
        
        self.nav = BasicNavigator()
        
        # --- GAZEBO WORLD COORDINATES ---
        # These are standard for 'turtlebot3_house.world'. 
        # Adjust these via RViz 'Publish Point' if using a custom world.
        self.locations = {
            "kitchen":     self.create_pose(3.0, -2.99, 270.0),
            "living room": self.create_pose(1.24, -2.99, 270.0),
            "bedroom":     self.create_pose(2.81, -1.07, 270.0), 
            "study":   self.create_pose(1.09, -1.22, 0.0), 
            "home":        self.create_pose(0.879, -2.53, 90.0)
        }

        # TSV Logging Setup
        self.log_dir = os.path.expanduser("/home/ubuntu/fyp1/src/experiment_log2/exp2")
        if not os.path.exists(self.log_dir): os.makedirs(self.log_dir)
        filename = f'sim_50_trials_{datetime.now().strftime("%Y%m%d_%H%M%S")}.tsv'
        self.tsv_filepath = os.path.join(self.log_dir, filename)
        self.init_tsv()
        
        self.command_sub = self.create_subscription(String, '/voice_commands', self.voice_topic_callback, 10)
        self.nav_monitor_timer = self.create_timer(0.5, self.monitor_navigation) 

        self.get_logger().info("SIMULATION VIP Manager Active. Synchronized with Gazebo Clock.")
        self.speak("Simulation ready. Send a voice command to start trials.")

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
            writer.writerow([timestamp, trial_no, location, cpu, ram, 
                             round(duration, 3), round(self.recorded_latency, 3)])

    def voice_topic_callback(self, msg):
        if self.is_busy: return
        try:
            data = msg.data.lower()
            location_part, latency_part = data.split(":") if ":" in data else (data, "0.0")
            self.recorded_latency = float(latency_part)
            
            for place in self.locations.keys():
                if place in location_part and place != "home":
                    self.target_location = place
                    self.trial_count = 1
                    self.is_busy = True 
                    self.state = "MOVING_TO_TARGET"
                    self.get_logger().info(f"SIM START: {self.target_location.upper()}")
                    break
        except Exception as e:
            self.get_logger().error(f"Sim Callback Error: {e}")

    def monitor_navigation(self):
        if not self.is_busy: return
        current_ros_time = self.get_clock().now()

        # PHASE 1: GO TO TARGET
        if self.state == "MOVING_TO_TARGET" and self.nav_start_time is None:
            self.get_logger().info(f"Trial {self.trial_count}/50: Heading to {self.target_location}")
            self.nav_start_time = current_ros_time
            self.nav.goToPose(self.locations[self.target_location])
            return

        # PHASE 2: ARRIVAL
        if self.state == "MOVING_TO_TARGET" and self.nav_start_time is not None:
            if not self.nav.isTaskComplete(): return 
            
            # Duration calculated using ROS time (accurate even if simulation speed changes)
            duration = (current_ros_time - self.nav_start_time).nanoseconds / 1e9
            
            if self.nav.getResult() == TaskResult.SUCCEEDED:
                self.log_trial(self.trial_count, self.target_location, duration)
                self.state = "WAITING"
                self.wait_start_time = current_ros_time
            else:
                self.get_logger().warn("Nav Failed in Sim. Retrying...")
                self.nav_start_time = None 

        # PHASE 3: WAIT
        if self.state == "WAITING":
            wait_duration = (current_ros_time - self.wait_start_time).nanoseconds / 1e9
            if wait_duration >= 3.0:
                self.state = "RETURNING_HOME"
                self.nav_start_time = None
            return

        # PHASE 4: GO HOME
        if self.state == "RETURNING_HOME" and self.nav_start_time is None:
            self.nav_start_time = current_ros_time
            self.nav.goToPose(self.locations["home"])
            return

        # PHASE 5: LOOP
        if self.state == "RETURNING_HOME" and self.nav_start_time is not None:
            if not self.nav.isTaskComplete(): return 

            if self.trial_count < self.max_trials:
                self.trial_count += 1
                self.state = "MOVING_TO_TARGET"
                self.nav_start_time = None
                self.recorded_latency = 0.0 
            else:
                self.speak(f"Simulation trials finished.")
                self.is_busy = False
                self.state = "IDLE"

    def create_pose(self, x, y, yaw_deg):
        import math
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        yaw_rad = math.radians(yaw_deg)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0)
        return pose

    def speak(self, text):
        try:
            tts = gTTS(text=text, lang='en')
            with tempfile.NamedTemporaryFile(delete=True, suffix='.mp3') as fp:
                tts.save(fp.name)
                os.system(f"mpg123 -q {fp.name} > /dev/null 2>&1") # Silenced errors
        except: pass

def main():
    rclpy.init()
    node = SimVipManager()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()