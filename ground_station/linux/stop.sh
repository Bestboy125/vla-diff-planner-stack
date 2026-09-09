#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER_NAME="vla-ground-station"
if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  docker stop "$CONTAINER_NAME" >/dev/null
  echo "VLA ground station stopped."
else
  echo "VLA ground station is not installed."
fi
