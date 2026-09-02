"""Receive a bounded ROS 2 test stream with Isaac Sim's bundled Humble rclpy."""

import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> int:
    received: list[str] = []
    rclpy.init()
    node = Node("isaac_windows_transport_listener")

    def on_message(message: String) -> None:
        received.append(message.data)
        print(message.data, flush=True)

    node.create_subscription(String, "/airstack/reverse_transport_probe", on_message, 10)
    # WSL startup and first-time Fast DDS endpoint matching can take several seconds.
    deadline = time.monotonic() + 45.0
    while time.monotonic() < deadline and len(received) < 5:
        rclpy.spin_once(node, timeout_sec=0.25)

    node.destroy_node()
    rclpy.shutdown()
    if len(received) < 5:
        print(f"ERROR: expected 5 messages, received {len(received)}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
