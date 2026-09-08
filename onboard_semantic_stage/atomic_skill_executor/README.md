# Atomic skill staged extension

`scripts/atomic_skill_server.py` is deployed over the onboard
`src/integration/atomic_skill_executor/scripts/atomic_skill_server.py`.

It preserves MOVE, ROTATE, HOLD and ORBIT and adds `GOTO_WORLD`, using the
existing action `center` field as an absolute world waypoint. Each invocation
is limited to 2 m by `~max_world_goto_distance` and is subdivided according to
`~max_goal_step`; every waypoint is still published through Diff-Planner.

`GOTO_WORLD` accepts `yaw_mode=path_tangent` for scan/return legs: yaw is the
world XY bearing from the current position to the goal. Empty or `fixed` mode
retains the captured heading for existing callers. No action message changes
are needed. The executor advertises `~goto_yaw_modes` on startup; the scan
orchestrator rejects requests if this capability is unavailable.
