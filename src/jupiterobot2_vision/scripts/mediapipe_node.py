#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp

class MediaPipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_node')
        
        # 1. Initialize MediaPipe Tools
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        
        # Initialize the Holistic model
        self.holistic = self.mp_holistic.Holistic(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        self.bridge = CvBridge()
        
        # 2. Subscriber & Publisher
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
            
        self.publisher = self.create_publisher(Image, '/mediapipe_result', 10)
        
        self.get_logger().info("MediaPipe Holistic Node is Ready.")

    def image_callback(self, msg):
        try:
            # Convert to OpenCV and Flip for "Mirror View"
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.flip(cv_image, 1)
            
            # MediaPipe requires RGB
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = self.holistic.process(image_rgb)
            
            # Draw the landmarks back onto the BGR image
            # 1. Face Mesh
            self.mp_drawing.draw_landmarks(cv_image, results.face_landmarks, self.mp_holistic.FACEMESH_CONTOURS)
            # 2. Right Hand
            self.mp_drawing.draw_landmarks(cv_image, results.right_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            # 3. Left Hand
            self.mp_drawing.draw_landmarks(cv_image, results.left_hand_landmarks, self.mp_holistic.HAND_CONNECTIONS)
            # 4. Pose Skeleton
            self.mp_drawing.draw_landmarks(cv_image, results.pose_landmarks, self.mp_holistic.POSE_CONNECTIONS)
            
            # Publish Result
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.publisher.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f"MediaPipe Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()