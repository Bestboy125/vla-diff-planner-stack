#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$ROOT_DIR/linux/config.env"
IMAGE_NAME="vla-ground-station:local"

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Missing $CONFIG_FILE. Copy config.env.example and fill the private values." >&2
  exit 1
fi

if grep -Eq '(^|=)(REQUIRED|REPLACE_WITH_)' "$CONFIG_FILE"; then
  echo "config.env still contains an unconfigured value." >&2
  exit 1
fi

chmod 600 "$CONFIG_FILE"
chmod 700 "$ROOT_DIR/linux/start.sh" "$ROOT_DIR/linux/start-live.sh" \
  "$ROOT_DIR/linux/stop.sh" "$ROOT_DIR/linux/sync-time-with-onboard.sh"
docker build --pull=false -f "$ROOT_DIR/linux/Dockerfile" -t "$IMAGE_NAME" "$ROOT_DIR"

if docker container inspect vla-ground-station >/dev/null 2>&1; then
  role="$(docker inspect -f '{{index .Config.Labels "com.vla.role"}}' vla-ground-station)"
  if [[ "$role" != "ground-station" ]]; then
    echo "Refusing to replace an unrecognized container named vla-ground-station." >&2
    exit 1
  fi
  docker rm -f vla-ground-station >/dev/null
fi

"$ROOT_DIR/linux/start.sh"
