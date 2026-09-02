#!/usr/bin/env python3
"""Relay Isaac standard ROS messages from DDS domain 42 to local domain 43.

Two independent processes are used deliberately: the source process uses the
Fast DDS Discovery Server needed across the Windows/WSL NAT boundary, while the
sink process uses ROS 2 Simple Discovery for a stable full AirStack graph.
Serialized CDR payloads are framed over a localhost TCP connection.
"""

import argparse
import socket
import struct
import threading
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.serialization import deserialize_message, serialize_message
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu


HEADER = struct.Struct("!BI")
HOST = "127.0.0.1"
PORT = 19042
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
STEREO_BASELINE_M = 0.12
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=2,
)
RELIABLE_QOS = QoSProfile(depth=10)
# Tuple fields are: topic name, message type, source QoS, sink QoS.  Isaac's
# ROS camera and Pegasus state publishers are best-effort.  Pose and twist are
# upgraded to reliable inside AirStack's local domain for deterministic state
# conversion and control.
TOPICS = {
    1: ("/robot_1/sensors/front_camera/image/rgb", Image, SENSOR_QOS, SENSOR_QOS),
    2: ("/robot_1/sensors/front_camera/image/camera_info", CameraInfo, RELIABLE_QOS, RELIABLE_QOS),
    3: ("/pegasus0/state/pose", PoseStamped, SENSOR_QOS, RELIABLE_QOS),
    4: ("/pegasus0/state/twist", TwistStamped, SENSOR_QOS, RELIABLE_QOS),
    5: ("/pegasus0/sensors/imu", Imu, SENSOR_QOS, SENSOR_QOS),
    6: ("/robot_1/sensors/front_camera/image/depth", Image, SENSOR_QOS, RELIABLE_QOS),
}


def read_exact(sock: socket.socket, length: int) -> bytes:
    chunks = []
    remaining = length
    while remaining:
        chunk = sock.recv(remaining)
        if not chunk:
            raise ConnectionError("bridge peer disconnected")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def normalize_camera_info(message: CameraInfo) -> CameraInfo:
    """Match Isaac 5.1 CameraInfoHelper output to the actual render product."""
    if not message.width or not message.height:
        return message
    if message.width != IMAGE_WIDTH or message.height != IMAGE_HEIGHT:
        scale_x = IMAGE_WIDTH / float(message.width)
        scale_y = IMAGE_HEIGHT / float(message.height)
        message.width = IMAGE_WIDTH
        message.height = IMAGE_HEIGHT
        message.k[0] *= scale_x
        message.k[2] *= scale_x
        message.k[4] *= scale_y
        message.k[5] *= scale_y
        message.p[0] *= scale_x
        message.p[2] *= scale_x
        message.p[3] *= scale_x
        message.p[5] *= scale_y
        message.p[6] *= scale_y
        message.p[7] *= scale_y
        message.roi.x_offset = int(round(message.roi.x_offset * scale_x))
        message.roi.y_offset = int(round(message.roi.y_offset * scale_y))
        message.roi.width = int(round(message.roi.width * scale_x))
        message.roi.height = int(round(message.roi.height * scale_y))

    # Isaac Sim 5.1's ROS CameraInfoHelper derives fy from the horizontal
    # aperture and the image aspect ratio even when the USD camera has an
    # explicit vertical aperture.  The renderer/depth annotator uses square
    # pixels, so publish the corresponding pinhole model to DROAN.  Without
    # this correction a 320x240, 90-degree camera is reported as
    # fx=160/fy=213.33 and vertically warps the reconstructed obstacle cloud.
    if message.k[0] > 0.0:
        message.k[4] = message.k[0]
    if message.p[0] > 0.0:
        message.p[5] = message.p[0]

    # Isaac publishes a monocular CameraInfo matrix. DROAN consumes stereo
    # disparity and derives the baseline from P[3], so expose the calibrated
    # virtual stereo baseline used by depth_to_disparity.py.
    if message.p[0] and abs(message.p[3]) < 1e-9:
        message.p[3] = -message.p[0] * STEREO_BASELINE_M
    return message


def run_sink() -> None:
    rclpy.init()
    node = rclpy.create_node("isaac_sensor_bridge_sink")
    publishers = {
        topic_id: node.create_publisher(message_type, name, sink_qos)
        for topic_id, (name, message_type, _, sink_qos) in TOPICS.items()
    }
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)
        node.get_logger().info(f"BRIDGE_SINK_READY tcp://{HOST}:{PORT}")
        while rclpy.ok():
            connection, _ = server.accept()
            node.get_logger().info("BRIDGE_SOURCE_CONNECTED")
            with connection:
                try:
                    while rclpy.ok():
                        topic_id, payload_size = HEADER.unpack(read_exact(connection, HEADER.size))
                        payload = read_exact(connection, payload_size)
                        _, message_type, _, _ = TOPICS[topic_id]
                        message = deserialize_message(payload, message_type)
                        # Isaac camera helpers stamp messages in simulation
                        # time while AirStack TF uses wall time. Retimestamp at
                        # the domain boundary so DROAN can resolve camera->map.
                        if topic_id in (1, 2, 6):
                            # Leave a small margin for the independently
                            # bridged pose/TF sample to arrive first.
                            message.header.stamp = (
                                node.get_clock().now() - Duration(seconds=0.1)
                            ).to_msg()
                        if topic_id == 2:
                            message = normalize_camera_info(message)
                        publishers[topic_id].publish(message)
                except (ConnectionError, OSError, KeyError) as exc:
                    node.get_logger().warning(f"bridge source reconnecting: {exc}")

    node.destroy_node()
    rclpy.shutdown()


def run_source() -> None:
    rclpy.init()
    node = rclpy.create_node("isaac_sensor_bridge_source")
    latest_frames = {}
    frames_ready = threading.Condition()

    def enqueue(topic_id: int, message) -> None:
        # Keep one latest sample per topic.  This prevents high-rate IMU/state
        # traffic from evicting every camera frame, while also bounding memory.
        payload = serialize_message(message)
        with frames_ready:
            latest_frames[topic_id] = payload
            frames_ready.notify()

    subscriptions = []
    for topic_id, (name, message_type, source_qos, _) in TOPICS.items():
        subscriptions.append(
            node.create_subscription(
                message_type,
                name,
                lambda message, selected=topic_id: enqueue(selected, message),
                source_qos,
            )
        )

    def sender() -> None:
        sock = None
        announced = False
        next_topic_id = 1
        while rclpy.ok():
            if sock is None:
                try:
                    sock = socket.create_connection((HOST, PORT), timeout=2.0)
                    sock.settimeout(None)
                    if not announced:
                        node.get_logger().info("BRIDGE_SOURCE_READY")
                        announced = True
                except OSError:
                    time.sleep(0.5)
                    continue
            with frames_ready:
                frames_ready.wait_for(lambda: bool(latest_frames), timeout=0.5)
                if not latest_frames:
                    continue
                topic_ids = sorted(latest_frames)
                eligible = [item for item in topic_ids if item >= next_topic_id]
                topic_id = eligible[0] if eligible else topic_ids[0]
                payload = latest_frames.pop(topic_id)
                next_topic_id = topic_id % len(TOPICS) + 1
            try:
                sock.sendall(HEADER.pack(topic_id, len(payload)) + payload)
            except OSError:
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None

    threading.Thread(target=sender, daemon=True).start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("source", "sink"))
    args = parser.parse_args()
    run_source() if args.mode == "source" else run_sink()


if __name__ == "__main__":
    main()
