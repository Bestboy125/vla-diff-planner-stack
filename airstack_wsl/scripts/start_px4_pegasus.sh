#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-/home/airstack/PX4-Autopilot}"
PX4_BUILD="${PX4_ROOT}/build/px4_sitl_default"
PX4_BIN="${PX4_BUILD}/bin/px4"
PX4_STATE="${PX4_STATE:-/home/airstack/.local/state/px4-pegasus/instance0}"
PX4_LOG="${PX4_STATE}/px4.log"
WINDOWS_HOST="${PX4_SIM_HOST_ADDR:-$(ip route | sed -n 's/^default via \([^ ]*\).*/\1/p' | head -n1)}"

if [[ ! -x "${PX4_BIN}" ]]; then
  echo "ERROR: PX4 SITL binary is missing: ${PX4_BIN}" >&2
  exit 10
fi
if [[ -z "${WINDOWS_HOST}" ]]; then
  echo "ERROR: could not determine the Windows host address from WSL" >&2
  exit 11
fi

mkdir -p "${PX4_STATE}"
if pgrep -f "^${PX4_BIN} .* -i 0 -d$" >/dev/null 2>&1; then
  if ss -tnp 2>/dev/null | grep -q "ESTAB.*${WINDOWS_HOST}:4560.*px4"; then
    echo "PX4 instance 0 is already connected to ${WINDOWS_HOST}:4560"
    exit 0
  fi
  # Pegasus' TCP client does not reconnect after Isaac restarts. Replace only
  # the stale instance-0 process so a fresh TCP session is established.
  mapfile -t stale_pids < <(pgrep -f "^${PX4_BIN} .* -i 0 -d$")
  kill -TERM "${stale_pids[@]}" 2>/dev/null || true
  sleep 2
  for stale_pid in "${stale_pids[@]}"; do
    kill -KILL "${stale_pid}" 2>/dev/null || true
  done
fi

export PX4_SIM_HOST_ADDR="${WINDOWS_HOST}"
export PX4_SIM_MODEL="gazebo-classic_iris"
export px4_instance=0

cd "${PX4_STATE}"
nohup "${PX4_BIN}" "${PX4_BUILD}/etc" -s etc/init.d-posix/rcS -i 0 -d >"${PX4_LOG}" 2>&1 &
PX4_PID=$!
echo "${PX4_PID}" >"${PX4_STATE}/px4.pid"

for _ in $(seq 1 40); do
  if ! kill -0 "${PX4_PID}" 2>/dev/null; then
    tail -n 80 "${PX4_LOG}" >&2
    exit 13
  fi
  if grep -q "INFO  \[simulator_mavlink\].*connected" "${PX4_LOG}" 2>/dev/null || \
     ss -tnp 2>/dev/null | grep -q "${WINDOWS_HOST}:4560"; then
    echo "PX4_READY pid=${PX4_PID} simulator=${WINDOWS_HOST}:4560 log=${PX4_LOG}"
    exit 0
  fi
  sleep 0.25
done

echo "ERROR: PX4 started but did not connect to Pegasus within 10 seconds" >&2
tail -n 80 "${PX4_LOG}" >&2
exit 14
