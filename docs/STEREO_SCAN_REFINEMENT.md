# Stereo scan target refinement (2026-09-08)

The existing dual-camera startup command remains unchanged. Restart the stack
while safely disarmed to load these changes. Deployment and offline tests do not
start ROS or send any flight commands.

## Behavior

- Scan interruption now requires four stable stereo observations, not one raw
  position. Both near and coarse stable observations can interrupt scanning.
- The scan passes `target_world_hint` (world XYZ) to the stereo orbit executor.
  Each subsequent accepted observation must be within `target_match_radius_m`
  (default 1 m, Euclidean) of the last accepted position. An unseeded standalone
  task locks its first accepted stable observation.
- `TARGET_MISMATCH` means no motion was issued for that candidate. Detection
  timeout remains active; target loss does not authorize blind advancement.
- `max_depth` is now 30 m instead of 6 m. This is a configurable filtering ceiling,
  NOT a validated sensor operating range. Pixel support, minimum disparity,
  depth MAD, epipolar checks, timestamp and stability checks remain unchanged.
- `precise_depth_m=6` separates near and coarse estimates by optical-axis depth.
  Far observations publish only `stable_coarse_target_world`; they never publish
  a stable planner goal or the near `stable_target_world` topic. Crossing the
  boundary resets the four-observation history.
- Coarse estimates authorize approach only when the entry point is farther than
  the stage limit. Each completed stage requests fresh localization. Default
  maximum stage length remains 2 m; stage count increases from 4 to 20, including
  the one-click wrapper override. Near-entry coarse estimates wait instead of
  entering a circle. Only a fresh near stable estimate can authorize ORBIT.
- Orbit remains radius 1.5 m, one lap, requested direction, face-center yaw and
  current flight altitude. Successful completion reports the actual target XYZ.
- Scan records deduplication only after success, using the refined target;
  horizontal deduplication radius is reduced from 3 m to 1 m. Failure stops the
  mission; success resumes the interrupted waypoint with a 5-second cooldown.
  Scan orbit timeout increases to 1800 seconds for the bounded approach sequence.

## Limitations and validation

Spatial gating is not visual object identity tracking. The detector still returns
the highest-confidence class match. If another chair dominates outside the lock,
the task waits/fails rather than searching all boxes; chairs within 1 m can still
be confused. Corrections larger than 1 m also require a new observation/task,
rather than silently expanding the association gate. Do not blindly enlarge it.
Being within 6 m does not guarantee accurate depth or a collision-free orbit.

Run offline:

    python3 scripts/semantic_refinement_self_test.py
    python3 scripts/semantic_orbit_contract_self_test.py
    python3 scripts/raw_stereo_geometry_self_test.py

The refinement tests execute actual executor callbacks and the localizer's actual
stability block with mocked publishers. Scan callback tests cover hint forwarding,
successful refined-position deduplication, failed-orbit behavior and fixed axes.
Physical range, identity association and approach/orbit behavior still require
controlled validation with a human operator; passing offline tests is not flight
approval.
