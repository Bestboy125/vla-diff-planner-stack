#!/usr/bin/env python3
"""One YOLO-World model shared by RGB and infrared semantic pipelines."""
import json
import os
import re
import threading
import time

import cv2
import numpy as np
import rospy
from std_msgs.msg import String
from ultralytics import YOLOWorld

from shared_yolo_world_detector.srv import DetectTarget, DetectTargetResponse


def image_to_bgr(message):
    encoding = message.encoding.lower()
    if encoding in ("bgr8", "rgb8"):
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        image = rows[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image
    if encoding in ("mono8", "8uc1"):
        rows = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        return cv2.cvtColor(rows[:, :message.width].copy(), cv2.COLOR_GRAY2BGR)
    raise ValueError("unsupported image encoding %s" % message.encoding)


class SharedYoloWorldDetector(object):
    def __init__(self):
        gp = rospy.get_param
        weights = os.path.expanduser(str(gp("~weights", "~/models/yolo_world/yolov8s-worldv2.pt")))
        self.device = gp("~device", 0)
        self.service_name = str(gp("~service_name", "/shared_yolo_world_detector/detect"))
        self.lock = threading.RLock()
        self.model = YOLOWorld(weights)
        self.active_label = None
        self.request_count = 0
        self.status_pub = rospy.Publisher("~status", String, queue_size=10, latch=True)
        self.service = rospy.Service(self.service_name, DetectTarget, self.on_detect)
        self.publish_status("READY", "one YOLO-World model is ready for all semantic pipelines")
        rospy.loginfo("shared YOLO-World detector ready; service=%s", self.service_name)

    def publish_status(self, state, detail, extra=None):
        payload = {
            "state": state,
            "detail": detail,
            "active_label": self.active_label,
            "request_count": self.request_count,
            "time_unix_ms": int(time.time() * 1000),
        }
        if extra:
            payload.update(extra)
        self.status_pub.publish(String(data=json.dumps(payload, separators=(",", ":"))))

    def on_detect(self, request):
        label = request.target_label.strip().lower()
        threshold = float(request.confidence_threshold)
        if re.fullmatch(r"[a-z][a-z-]{0,31}", label) is None:
            return DetectTargetResponse(False, "target_label must be one English word", "", 0.0, [])
        if not 0.0 < threshold <= 1.0:
            return DetectTargetResponse(False, "confidence_threshold must be within (0, 1]", "", 0.0, [])
        try:
            image = image_to_bgr(request.image)
            with self.lock:
                self.request_count += 1
                if label != self.active_label:
                    self.model.set_classes([label])
                    self.active_label = label
                result = self.model.predict(
                    image, conf=threshold, device=self.device, verbose=False
                )[0]
                candidates = []
                for box, score, class_index in zip(
                    result.boxes.xyxy.cpu().tolist(),
                    result.boxes.conf.cpu().tolist(),
                    result.boxes.cls.cpu().tolist(),
                ):
                    if result.names[int(class_index)] == label:
                        candidates.append((float(score), [float(value) for value in box]))
            if not candidates:
                return DetectTargetResponse(False, "no %s detected" % label, label, 0.0, [])
            score, bbox = max(candidates, key=lambda item: item[0])
            self.publish_status(
                "DETECTED", "shared detector found target",
                {"label": label, "confidence": score, "bbox_xyxy": bbox},
            )
            return DetectTargetResponse(True, "target detected", label, score, bbox)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "shared YOLO-World request failed: %s", exc)
            self.publish_status("ERROR", "inference failed: %s" % exc)
            return DetectTargetResponse(False, "inference failed: %s" % exc, label, 0.0, [])


if __name__ == "__main__":
    rospy.init_node("shared_yolo_world_detector")
    SharedYoloWorldDetector()
    rospy.spin()
