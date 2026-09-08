# Hybrid far-to-near semantic orbit

The mission performs one safety-gated chain:

1. D435 rectified-left YOLO detection at position A.
2. Measured 0.5-1.0 m lateral baseline and a second D435-left capture B.
3. Left-view coarse target localization using the same calibrated
   `body_T_left_camera` as the stereo localizer (no coarse-stage orbit).
4. Bounded `GOTO_WORLD` approach legs, each planned by Diff-Planner.
5. At a 4 m coarse range, discard the coarse center and request a fresh D435
   raw-stereo stable target.
6. The verified stereo executor approaches the refined center and performs one
   clockwise 1.5 m orbit.

Status markers are published on `/hybrid_semantic_orbit_mission/status`:
`MONOCULAR_LOCALIZING`, `COARSE_TARGET_ESTIMATED`, `COARSE_APPROACHING`,
`D435_HANDOFF`, `D435_DETECTING`, `ORBITING`, and `SUCCEEDED`, with
`FAILED`, `REJECTED`, or `CANCELLED` as terminal failures.

The launch defaults to `execution_enabled=false` and never arms or takes off.
