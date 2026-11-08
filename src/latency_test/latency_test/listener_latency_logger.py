#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
import time
import psutil
import os


class LatencyLogger(Node):

    def __init__(self):
        super().__init__('latency_logger')

        # Explicit QoS (important for Fast DDS)
        qos = QoSProfile(depth=10)

        # Subscriber
        self.subscription = self.create_subscription(
            Float64,
            'latency_test',
            self.callback,
            qos
        )

        self.counter = 0
        self.start_time = time.time()

        # Output file (TSV)
        self.file_path = os.path.join(os.getcwd(), 'latency_log.tsv')
        self.file = open(self.file_path, 'w')

        # TSV header
        self.file.write(
            "no\ttimestamp\tlatency_sec\toneway_latency_sec\t"
            "total_latency_sec\tcpu_usage_percent\n"
        )
        self.file.flush()

        self.get_logger().info(
            f'Latency Listener started, logging to {self.file_path}'
        )


    def callback(self, msg):
        receive_time = time.time()
        sent_time = msg.data

        latency = receive_time - sent_time
        total_latency = receive_time - self.start_time

        # CPU usage on listener machine
        cpu_usage = psutil.cpu_percent(interval=None)

        self.counter += 1

        # Write TSV row
        self.file.write(
            f"{self.counter}\t"
            f"{receive_time:.6f}\t"
            f"{latency:.6f}\t"
            f"{latency:.6f}\t"
            f"{total_latency:.6f}\t"
            f"{cpu_usage:.2f}\n"
        )
        self.file.flush()

        # Optional debug output
        self.get_logger().info(
            f"Msg {self.counter} | Latency: {latency:.6f}s | CPU: {cpu_usage:.2f}%"
        )


    def destroy_node(self):
        # Ensure file is closed cleanly
        if hasattr(self, 'file') and not self.file.closed:
            self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = LatencyLogger()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

