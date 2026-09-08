# Semantic scan, interrupt, stereo orbit and resume

This independent ROS package orchestrates existing verified components; it does
not replace the D435 semantic localizer, semantic-orbit executor, atomic skill
server or Diff-Planner.

The fixed preset spans world X = origin.x .. origin.x + 6 m and world Y =
origin.y .. origin.y + 10 m, at the task-start altitude. Only the task-start
position is captured; the heading never rotates this route. Five 6 m sweeps run
alternately along +X/-X at Y offsets 0, 2.5, 5, 7.5 and 10 m. For wire
compatibility, `scan_width_m=6` denotes the X extent and `scan_length_m=10`
denotes the Y extent. Straight scan samples are at most 1 m apart and each
row transition contains six sampled curved connector points. Every point is sent
as a safety-limited `GOTO_WORLD` action, so the executable trajectory is still
planned by Diff-Planner.

Scan and return actions opt into `yaw_mode=path_tangent`: the atomic executor
sends yaw along the horizontal segment from the current position to the next
goal together with the position command. The trajectory server applies its angular
rate/acceleration limits. This remains waypoint-by-waypoint stop-and-settle
execution, not a continuously tracked global spline. The near-target orbit
retains `face_center`; after orbiting the saved world route is resumed.
`route_json` includes the fixed axes and nominal incoming yaw per waypoint;
status reports the actual leg yaw, which can differ on a return from an orbit.

Runtime state sequence:

1. `WAITING_FOR_YOLO` - request the continuously running raw-stereo YOLO-World
   node to detect `chair`.
2. `NAVIGATING` / `RETURNING` - send one scan or return leg through the atomic
   executor. `WAYPOINT_REACHED` is emitted only after the action result reports
   `SUCCEEDED`.
3. `CHAIR_DETECTED` - a fresh raw-stereo 3-D chair observation immediately
   cancels the active navigation action and publishes the recoverable planner
   hover-stop.
4. `STOPPED_FOR_CHAIR` - fresh odometry must confirm low speed for the configured
   settle duration.
5. `ORBIT_REQUESTED` - invoke the existing D435 semantic orbit request with the
   fixed clockwise, 1.5 m, one-lap contract. That executor independently waits
   for a stable stereo target before moving.
6. `ORBIT_COMPLETED` - only a matching semantic-orbit `SUCCEEDED` status resumes
   the interrupted scan waypoint. Already processed targets are suppressed by
   world-position radius and a post-orbit cooldown.
7. `MISSION_COMPLETED` - emitted only after every generated scan waypoint has
   returned an action success result.

Launch defaults to `execution_enabled=false`. It never arms, takes off or changes
flight mode, and a request is rejected unless the aircraft is already connected,
armed, localized and inside the configured altitude envelope.
