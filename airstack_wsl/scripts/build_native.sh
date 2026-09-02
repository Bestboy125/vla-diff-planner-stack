#!/usr/bin/env bash
set -eo pipefail

AIR_STACK_ROOT="${AIR_STACK_ROOT:-/home/airstack/AirStack}"
NATIVE_ROOT="${AIR_STACK_ROOT}/.native_ws"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-6}"
MODE="${1:-all}"

source /opt/ros/humble/setup.bash
cd "${AIR_STACK_ROOT}"

build_common() {
  mkdir -p "${NATIVE_ROOT}/common"
  colcon --log-base "${NATIVE_ROOT}/common/log" build \
    --symlink-install \
    --base-paths common/ros_packages \
    --build-base "${NATIVE_ROOT}/common/build" \
    --install-base "${NATIVE_ROOT}/common/install" \
    --executor parallel \
    --parallel-workers "${PARALLEL_WORKERS}" \
    --event-handlers console_cohesion+
}

build_robot() {
  source "${NATIVE_ROOT}/common/install/setup.bash"
  export CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH:-}"
  mkdir -p "${NATIVE_ROOT}/robot"

  local clean_args=()
  if [[ "${MODE}" == "robot-clean" ]]; then
    clean_args+=(--cmake-clean-cache)
  fi

  colcon --log-base "${NATIVE_ROOT}/robot/log" build \
    --symlink-install \
    --base-paths robot/ros_ws/src \
    --build-base "${NATIVE_ROOT}/robot/build" \
    --install-base "${NATIVE_ROOT}/robot/install" \
    --packages-skip macvo_ros2 \
    --executor parallel \
    --parallel-workers "${PARALLEL_WORKERS}" \
    --event-handlers console_cohesion+ \
    "${clean_args[@]}"
}

build_gcs() {
  source "${NATIVE_ROOT}/common/install/setup.bash"
  if [[ -f "${NATIVE_ROOT}/robot/install/setup.bash" ]]; then
    source "${NATIVE_ROOT}/robot/install/setup.bash"
  fi
  mkdir -p "${NATIVE_ROOT}/gcs"
  colcon --log-base "${NATIVE_ROOT}/gcs/log" build \
    --symlink-install \
    --base-paths gcs/ros_ws/src \
    --build-base "${NATIVE_ROOT}/gcs/build" \
    --install-base "${NATIVE_ROOT}/gcs/install" \
    --executor parallel \
    --parallel-workers "${PARALLEL_WORKERS}" \
    --event-handlers console_cohesion+
}

case "${MODE}" in
  common)
    build_common
    ;;
  robot|robot-clean)
    build_robot
    ;;
  gcs)
    build_gcs
    ;;
  all)
    build_common
    build_robot
    build_gcs
    ;;
  *)
    echo "Usage: $0 [common|robot|robot-clean|gcs|all]" >&2
    exit 2
    ;;
esac
