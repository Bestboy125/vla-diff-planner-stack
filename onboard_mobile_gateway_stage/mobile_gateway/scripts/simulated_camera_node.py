#!/usr/bin/env python3
"""Publishes a repository image as raw and compressed ROS camera topics for integration tests."""

import cv2
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Header


def main():
    rospy.init_node("simulated_camera")
    image_path = rospy.get_param("~image")
    frame_id = rospy.get_param("~frame_id", "simulated_camera_optical_frame")
    fps = float(rospy.get_param("~fps", 10.0))
    quality = int(rospy.get_param("~jpeg_quality", 80))
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    raw_topic = rospy.get_param("~raw_topic", "/camera/color/image_raw")
    compressed_topic = rospy.get_param(
        "~compressed_topic", "/camera/color/image_raw/compressed")

    image = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("cannot read test image: " + image_path)
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if image.shape[1] != width or image.shape[0] != height:
        image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
    ok, encoded = cv2.imencode(
        ".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if not ok:
        raise RuntimeError("cannot encode test image")

    raw_publisher = rospy.Publisher(raw_topic, Image, queue_size=2)
    compressed_publisher = rospy.Publisher(compressed_topic, CompressedImage, queue_size=2)
    rate = rospy.Rate(max(0.2, fps))
    sequence = 0
    rospy.logwarn(
        "simulated camera publishing %s at %dx%d, %.1f Hz",
        image_path, width, height, fps)
    while not rospy.is_shutdown():
        header = Header(seq=sequence, stamp=rospy.Time.now(), frame_id=frame_id)
        raw = Image(
            header=header,
            height=image.shape[0],
            width=image.shape[1],
            encoding="bgr8",
            is_bigendian=0,
            step=image.shape[1] * 3,
            data=image.tobytes(),
        )
        compressed = CompressedImage(
            header=header, format="jpeg", data=encoded.tobytes())
        raw_publisher.publish(raw)
        compressed_publisher.publish(compressed)
        sequence += 1
        rate.sleep()


if __name__ == "__main__":
    main()
