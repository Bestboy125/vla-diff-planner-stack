"""Publish a bounded ROS 2 test stream with Isaac Sim's bundled Humble rclpy."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    rclpy.init()
    node = Node("isaac_windows_transport_probe")
    publisher = node.create_publisher(String, "/airstack/transport_probe", 10)

    # Give Fast DDS time to register both clients with the discovery server.
    deadline = time.monotonic() + 12.0
    sequence = 0
    while time.monotonic() < deadline:
        message = String()
        message.data = f"windows-isaac-probe-{sequence}"
        publisher.publish(message)
        print(message.data, flush=True)
        sequence += 1
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.4)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
