#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import csv
import psutil
import time
import os  
from datetime import datetime
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. Initialize Tools
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt') 
        
        # 2. Performance Logging Setup with Custom Directory Destination
        self.log_dir = "/home/ubuntu/fyp1/src/experiment_log"
        if not os.path.exists(self.log_dir): 
            os.makedirs(self.log_dir)
            self.get_logger().info(f"Created log directory at: {self.log_dir}")
            
        self.tsv_filepath = os.path.join(self.log_dir, '90cm_performance_data.tsv')
        
        self.trial_count = 0
        self.max_trials = 50
        self.init_tsv()
        
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("Using NVIDIA GPU (CUDA) for YOLOv8")
        
        # 3. Subscriber & Publisher
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Image, '/yolo_result', 10)
        
        cv2.namedWindow("YOLOv8 Real-Time Monitor", cv2.WINDOW_NORMAL)
        self.get_logger().info("YOLOv8 Detection Node Ready (Filtered for Bottles + Recognition %).")

    def init_tsv(self):
        # Updated header to include the new object_recognition_% field
        header = ['timestamp', 'trial_no', 'location', 'cpu_%', 'ram_%', 'time_taken_sec', 'stt_latency_sec', 'object_recognition_%']
        with open(self.tsv_filepath, 'w', newline='') as f:
            writer = csv.writer(f, delimiter='\t')
            writer.writerow(header)

    def log_data(self, location, time_taken, stt_latency, recognition_percentage):
        if self.trial_count < self.max_trials:
            self.trial_count += 1
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cpu_usage = psutil.cpu_percent(interval=None)
            ram_usage = psutil.virtual_memory().percent
            
            with open(self.tsv_filepath, 'a', newline='') as f:
                writer = csv.writer(f, delimiter='\t')
                writer.writerow([
                    timestamp, self.trial_count, location, cpu_usage, ram_usage, 
                    f"{time_taken:.4f}", f"{stt_latency:.4f}", f"{recognition_percentage:.2f}"
                ])
            
            self.get_logger().info(f"Trial {self.trial_count}/50 logged. Conf: {recognition_percentage:.2f}%")
            if self.trial_count == self.max_trials:
                self.get_logger().info(f"Data collection complete. TSV file saved at: {self.tsv_filepath}")

    def image_callback(self, msg):
        try:
            start_time = time.time()
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # RUN INFERENCE
            results = self.model(cv_image, conf=0.5, classes=[39], verbose=False)
            inference_end_time = time.time()
            
            total_time = inference_end_time - start_time
            
            # Extract target confidence score
            recognition_percentage = 0.0
            if len(results[0].boxes) > 0:
                # Get the highest confidence score from detected targets and scale to a percentage
                highest_conf = max(results[0].boxes.conf).item()
                recognition_percentage = highest_conf * 100.0
            
            # Log data if target count is not reached
            if self.trial_count < self.max_trials:
                self.log_data(location="Lab_A", time_taken=total_time, stt_latency=total_time, recognition_percentage=recognition_percentage)
            
            # Visualization
            annotated_frame = results[0].plot()
            hd_frame = cv2.resize(annotated_frame, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            
            cv2.imshow("YOLOv8 Real-Time Monitor", hd_frame)
            cv2.waitKey(1)
            
            result_msg = self.bridge.cv2_to_imgmsg(hd_frame, encoding='bgr8')
            self.publisher.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()