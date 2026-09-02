# Semantic 3D localization bridge

The runtime chain is:

1. YOLO-World maps a text prompt to image-space `bbox_xyxy`.
2. A rectified stereo pair supplies the median disparity inside that box.
3. `Z=fx*baseline/disparity`, `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy` recover the
   point in the ROS optical camera frame.
4. The calibrated `T_body_camera` and time-aligned Fast-LIO
   `T_world_body` transform the point into the planning world frame.
5. A standoff pose is published to `/goal`; Diff-Planner plans around its
   point-cloud obstacles, while `/planning/yaw` turns the vehicle to the target.

For real sensors, synchronize left image, right image and Fast-LIO odometry
with `message_filters`, reject small/negative disparities, and propagate
stereo/calibration/pose covariance before accepting the target.

Validation:

```bash
python3 scripts/geometry_self_test.py
roslaunch semantic_3d_localizer semantic_diff_planner_sim.launch
```

YOLO-World inference (run in its CUDA virtual environment):

```bash
~/venvs/yolo_world/bin/python scripts/yolo_world_infer.py \
  --weights ~/models/yolo_world/yolov8s-worldv2.pt --image IMAGE.jpg \
  --classes person bus "traffic light" --output-dir artifacts/yolo_world
```
