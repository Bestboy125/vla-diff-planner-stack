#!/usr/bin/env bash
set -euo pipefail

WSL_IP="$(hostname -I | awk '{print $1}')"
if [[ -z "${WSL_IP}" ]]; then
  echo "ERROR: cannot determine WSL IPv4 address" >&2
  exit 10
fi

export LD_LIBRARY_PATH="/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
exec /bin/bash /opt/ros/humble/bin/fastdds discovery \
  -i 0 \
  -l 127.0.0.1 -p 11811 \
  -l "${WSL_IP}" -p 11811
