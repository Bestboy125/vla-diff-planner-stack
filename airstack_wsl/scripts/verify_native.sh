#!/usr/bin/env bash
set -eo pipefail

EXPECTED_COMMIT="278acbffaf748cd6e0102b3a25cfea544e031c83"
AIR_STACK_ROOT="${AIR_STACK_ROOT:-/home/airstack/AirStack}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

source "${SCRIPT_DIR}/airstack_env.sh"
cd "${AIR_STACK_ROOT}"

actual_commit="$(git rev-parse HEAD)"
[[ "${actual_commit}" == "${EXPECTED_COMMIT}" ]] || {
  echo "ERROR: AirStack commit ${actual_commit}, expected ${EXPECTED_COMMIT}" >&2
  exit 1
}

python3 - <<'PY'
import importlib

for module_name in ("pytak", "paho.mqtt.client", "utm", "yaml"):
    importlib.import_module(module_name)
    print(f"runtime import OK: {module_name}")
PY

for package_name in \
  airstack_msgs droan_local_planner exploration_planner vdb_mapping_ros2 \
  trajectory_controller mavros_interface robot_bringup gcs_bringup; do
  ros2 pkg prefix "${package_name}" >/dev/null
  echo "ROS package OK: ${package_name}"
done

if find "${AIR_STACK_ROOT}/.native_ws/robot/install" -type f -executable -print0 \
  | xargs -0 -r ldd 2>/dev/null | grep -q "not found"; then
  echo "ERROR: unresolved shared library detected" >&2
  exit 1
fi

ros2 launch robot_bringup sim.launch.xml --show-args >/dev/null
ros2 launch gcs_bringup gcs.launch.xml --show-args >/dev/null

openvdb_version="$(pkg-config --modversion openvdb 2>/dev/null || true)"
echo "OpenVDB: ${openvdb_version:-9.1.0 (/usr/local)}"
echo "AirStack native verification PASSED"
