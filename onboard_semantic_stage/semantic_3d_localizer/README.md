# Semantic 3D localization bridge

The runtime chain is:

1. YOLO-World maps a text prompt to image-space `bbox_xyxy`.
2. A rectified stereo pair supplies the median disparity inside that box.
3. The real node consumes D435 depth already aligned to color. It takes a
   MAD-filtered median in the central bounding-box region, then uses
   `X=(u-cx)Z/fx`, `Y=(v-cy)Z/fy` to recover the point in the ROS optical
   camera frame. `Z=fx*baseline/disparity` documents the underlying stereo
   geometry and is used by the dependency-free geometry test.
4. The calibrated `T_body_camera` and time-aligned Fast-LIO
   `T_world_body` transform the point into the planning world frame.
5. The target is published in both body and world frames. A horizontal
   standoff pose that keeps current vehicle altitude is published as a candidate.
   The bridge rejects observations with excessive timestamp skew or depth MAD,
   and requires four consistent 3-D observations before exposing a stable goal.
   It reaches `/goal` only when both execution gates are explicitly enabled and
   `~send_goal` is called; Diff-Planner then plans around its point-cloud
   obstacles. Automatic stable-goal publication is separately opt-in and is off
   by default. The yaw output is a candidate only and is not wired directly to
   `/planning/yaw`.

Safe D435 + Fast-LIO/EKF candidate-only launch:

```bash
roslaunch semantic_3d_localizer semantic_d435_fastlio.launch \
  target_class:=person execution_enabled:=false publish_planner_goal:=false
```

After inspecting `target_body`, `target_world`, `stable_goal_candidate`, and the
calibration on a stationary bench, a single planner goal can be committed with:

```bash
rosservice call /semantic_target_node/send_goal
```

The service refuses the request unless both launch execution gates were enabled
and a non-stale stable target exists.

For a no-motion planner integration test, set `planner_goal_topic` to an isolated
preview topic consumed by the preview Diff-Planner instance. The real default is
`/goal`.

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
