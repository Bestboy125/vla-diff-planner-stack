# Two-position monocular semantic orbit

This is a new ROS package and does not replace the validated D435 raw-stereo
pipeline. It uses the D435 rectified left view at two vehicle positions, the
same calibrated `body_T_left_camera` transform as the stereo pipeline, the measured
Fast-LIO/EKF poses at both exposure times, and the bundled
`camera_coordinate_core_v1.0.0` SuperPoint/LightGlue estimator.

Runtime sequence:

1. detect the requested YOLO-World class and capture image A;
2. use the requested `baseline_direction` (`left` or `right`) and
   `baseline_distance_m` (0.5--1.0 m) to send the atomic `GOTO_WORLD` skill to
   the second viewpoint;
3. wait for low velocity and capture image B;
4. compute the measured camera baseline from both time-aligned poses and the
   calibrated body-camera transform, and require it to remain close to the
   requested second-viewpoint displacement;
5. require learned feature inliers, reprojection quality, bounded uncertainty
   and overlap with the second YOLO box;
6. accept target candidates up to 50 m only when the geometry and uncertainty
   gates pass, convert the B-camera estimate to `world`, approach in <=2 m stages, then
   invoke the existing atomic `ORBIT` skill.

The launch defaults to `execution_enabled=false`.  Starting it does not arm,
take off, change mode, or publish a planner goal.  A request is accepted only
when the vehicle is already connected, armed and within the configured altitude
bounds. The existing D435 stereo package remains unchanged and takes over for
near-range refinement in the hybrid mission.

The vendored estimator and weights are copied from the user-supplied reference
package.  Its original validation notes and third-party notices are retained.
