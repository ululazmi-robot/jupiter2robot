import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time

class LatencyTalker(Node):
    def __init__(self):
        super().__init__('latency_talker')
        self.publisher = self.create_publisher(Float64, 'latency_test', 10)
        self.timer = self.create_timer(0.5, self.publish_msg)

    def publish_msg(self):
        msg = Float64()
        msg.data = time.time()  # send timestamp
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = LatencyTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

