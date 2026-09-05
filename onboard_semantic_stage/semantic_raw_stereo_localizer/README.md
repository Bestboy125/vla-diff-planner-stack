# Semantic raw-stereo localizer

This ROS package is an independent alternative to `semantic_3d_localizer`.

- `semantic_3d_localizer` consumes RealSense `aligned_depth_to_color`.
- `semantic_raw_stereo_localizer` consumes only the rectified D435 infrared left
  and right images plus their `CameraInfo`; it never reads a RealSense depth topic.

The package follows the geometry of `camera_coordinate_core_v1.0.0`: establish
correspondences in two views, recover metric depth from a calibrated metric
baseline, triangulate, and reject degenerate geometry. The reference package's
SuperPoint/LightGlue source and weights are not copied because its
`THIRD_PARTY_NOTICES.md` does not grant a standalone redistribution license.
Because the current D435 supplies synchronized rectified views, the default
backend is an independent OpenCV SGBM disparity computation over those raw
views, followed by a centre-of-box ROI and depth-MAD filtering. It does not use
the D435 depth image. A learned `lightglue` backend follows the reference
pipeline while avoiding redistribution of its restricted SuperPoint files:
DISK features, LightGlue matching, target-box and fundamental-matrix filtering,
a box-centred coherent feature cluster, positive-disparity triangulation, and
near-depth foreground clustering plus depth-MAD filtering. The near-depth
cluster must independently satisfy `min_matches`, preventing a few spurious
close matches from winning. Optional LK forward/backward sub-pixel refinement
is available but disabled by default: on the tested infrared scene it followed
background texture through holes in the detected chair. An `orb` backend
retains the same geometric path with classical descriptors. Select either with
`depth_backend:=lightglue` or `depth_backend:=orb`.

The sparse learned backend is primarily an experimental/reference path. At
roughly 2.3 m the connected D435 has only about 8--9 pixels of disparity, so a
one-pixel sparse-keypoint error creates decimetre-scale depth error. Keep
`depth_backend:=sgbm` for the current flight configuration unless a wider
baseline or a validated sub-pixel matcher is introduced.

The learned backend requires external runtime packages and official model
weights; neither model source nor weights are committed here:

```bash
python3 -m pip install --user kornia==0.7.2 kornia_rs
```

This is true stereo inference from the two current D435 views. It does **not**
sample, align, or repackage `/camera/depth/*`; the driver-provided projection
matrices are used only as stereo calibration.

## Geometry

The baseline is read from the projection matrices published by the driver:

```text
C_x = -P[0,3] / P[0,0]
baseline = C_right_x - C_left_x
```

For the connected D435 at 640x480, `fx=388.2721` and the right projection term
is `-19.3843`, giving a baseline of about `0.04992 m`. Matched feature points are
triangulated with the two full 3x4 projection matrices. The robust median depth
is applied to the YOLO bounding-box center ray, so the reported coordinate is
the box center at the matched visible-surface depth, not an object's hidden
geometric center.

## Distinct runtime names

- package: `semantic_raw_stereo_localizer`
- node: `/semantic_raw_stereo_node`
- estimate: `/semantic_raw_stereo_node/estimate_json`
- left-camera target: `/semantic_raw_stereo_node/target_left_camera`
- body/world targets: `/semantic_raw_stereo_node/target_body`, `target_world`
- stable candidate: `/semantic_raw_stereo_node/stable_goal_candidate`
- one-shot commit service: `/semantic_raw_stereo_node/send_goal`

Safe candidate-only launch when the D435 driver and EKF are already running:

```bash
roslaunch semantic_raw_stereo_localizer semantic_raw_stereo_real.launch \
  target_class:=person execution_enabled:=false publish_planner_goal:=false
```

Standalone D435 raw-infra launch:

```bash
roslaunch semantic_raw_stereo_localizer semantic_d435_raw_stereo_fastlio.launch \
  target_class:=person execution_enabled:=false publish_planner_goal:=false
```

Both execution gates default to false. Even with both enabled, `/goal` is only
published after four stable estimates and an explicit `send_goal` service call,
unless automatic publishing is separately opted in.

## Limitations

- YOLO-World runs on a three-channel copy of the left infrared image. Classes
  trained primarily on RGB may have lower recall.
- ORB is less invariant than SuperPoint/LightGlue and can fail on textureless,
  repetitive, reflective, or partially occluded targets.
- Depth uncertainty still grows quickly as disparity approaches zero.
- The current body-camera extrinsic is marked as an initial estimate and must be
  verified before physical execution.
- A raw-stereo failure is published as no new target; callers must never reuse a
  previous estimate as if it were current.

Geometry self-test:

```bash
python3 scripts/raw_stereo_geometry_self_test.py
```
