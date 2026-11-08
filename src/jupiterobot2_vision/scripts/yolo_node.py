#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. Initialize Tools
        self.bridge = CvBridge()
        
        # 2. Load YOLOv8 Model
        self.model = YOLO('yolov8n.pt') 
        
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("Using NVIDIA GPU (CUDA) for YOLOv8")
        else:
            self.get_logger().info("Using CPU for YOLOv8")
        
        # 3. Subscriber & Publisher
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  
            self.image_callback,
            10)
            
        self.publisher = self.create_publisher(Image, '/yolo_result', 10)
        
        # Setup OpenCV window and enforce standard handling layout
        cv2.namedWindow("YOLOv8 Real-Time Monitor", cv2.WINDOW_NORMAL)
        
        self.get_logger().info("YOLOv8 Detection Node is Ready with 1080p Upscaling Engine.")

    def image_callback(self, msg):
        try:
            # A. Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # B. UPSCALE CAPTURE INTERFACE TO 1920x1080 BEFORE PROCESSING DRAWINGS
            # This ensures that all bounding boxes, font scale ratios, and lines scale cleanly.
            hd_frame = cv2.resize(cv_image, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            
            # C. Run Inference (YOLO handles internal downsizing natively to match network resolution)
            results = self.model(hd_frame, conf=0.5, verbose=False)
            boxes = results[0].boxes
            
            # D. DRAW TOP DATA BOARD (Scaled cleanly for 1080p display layout)
            total_count = len(boxes)
            header_text = f"Objects Tracked: {total_count}"
            
            # Upscaled dark background banner box matching the template look
            cv2.rectangle(hd_frame, (20, 20), (550, 90), (0, 0, 0), -1)
            cv2.putText(hd_frame, header_text, (40, 68), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.3, (255, 255, 255), 3, cv2.LINE_AA)
            
            # E. PROCESS INDIVIDUAL BOUNDING BOX DESIGN (Directly on the 1080p Grid)
            if total_count == 0:
                self.get_logger().info("Monitoring... No objects detected.")
            else:
                detected_items = []
                for box in boxes:
                    # Capture exact coordinates on the 1080p frame
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    class_id = int(box.cls[0])
                    label = self.model.names[class_id]
                    confidence = float(box.conf[0])
                    detected_items.append(f"{label} ({confidence*100:.1f}%)")
                    
                    # 1. Draw neon-green bounding box (Thicker weight for higher resolution display)
                    cv2.rectangle(hd_frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    
                    # 2. Draw white anchor dot at center of mass
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.circle(hd_frame, (cx, cy), 6, (255, 255, 255), -1)
                    
                    # 3. Compile targeting tags (Font adjusted for 1080p visibility)
                    tag_label = f"{label.upper()} [{confidence*100:.0f}%]"
                    tag_coords = f"X:{cx} Y:{cy}"
                    
                    cv2.putText(hd_frame, tag_label, (x1 + 8, y1 + 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(hd_frame, tag_coords, (x1 + 8, y1 + 55), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                
                self.get_logger().info(f"Detected on Monitor: {', '.join(detected_items)}")
            
            # F. POP UP FULLSCREEN SHOW WINDOW
            cv2.imshow("YOLOv8 Real-Time Monitor", hd_frame)
            cv2.waitKey(1)
            
            # Publish out upscaled annotated frame buffer back to ROS architecture
            result_msg = self.bridge.cv2_to_imgmsg(hd_frame, encoding='bgr8')
            self.publisher.publisher.publish(result_msg) if hasattr(self.publisher, 'publisher') else self.publisher.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f"Detection Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down mix platform...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()