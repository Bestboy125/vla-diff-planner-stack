# Shared YOLO-World detector

This package owns the only YOLO-World model used by the combined onboard
semantic stack. RGB monocular captures and D435 infrared frames call the same
serialized `/shared_yolo_world_detector/detect` service, preventing duplicate
model allocations and concurrent GPU inference.

The node has no flight-control publishers.
