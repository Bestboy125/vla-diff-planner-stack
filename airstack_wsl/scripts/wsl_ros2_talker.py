"""Publish a bounded AirStack-side ROS 2 transport test stream."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    rclpy.init()
    node = Node("airstack_wsl_transport_probe")
    publisher = node.create_publisher(String, "/airstack/reverse_transport_probe", 10)
    time.sleep(1.5)
    for sequence in range(12):
        message = String()
        message.data = f"wsl-airstack-probe-{sequence}"
        publisher.publish(message)
        print(message.data, flush=True)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.25)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
