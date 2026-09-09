#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$ROOT_DIR/linux/config.env"
IMAGE_NAME="vla-ground-station:local"
CONTAINER_NAME="vla-ground-station"
URL="http://127.0.0.1:8080"

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "Missing $CONFIG_FILE. Run linux/deploy.sh first." >&2
  exit 1
fi

if ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "Missing image $IMAGE_NAME. Run linux/deploy.sh first." >&2
  exit 1
fi

if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  role="$(docker inspect -f '{{index .Config.Labels "com.vla.role"}}' "$CONTAINER_NAME")"
  [[ "$role" == ground-station ]] || {
    echo "Refusing to operate on an unrecognized container named $CONTAINER_NAME." >&2
    exit 1
  }
  control_enabled="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" |
    sed -n 's/^CONTROL_OUTPUT_ENABLED=//p' | tail -n 1)"
  if [[ "$control_enabled" == true && \
        "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" == true ]]; then
    echo 'A Live ground-station container is running. It was not interrupted.' >&2
    echo 'Land safely and run linux/stop.sh before returning to locked mode.' >&2
    exit 1
  fi
  if [[ "$control_enabled" == true ]]; then
    docker rm "$CONTAINER_NAME" >/dev/null
  elif [[ "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" != "true" ]]; then
    docker start "$CONTAINER_NAME" >/dev/null
  fi
fi

if ! docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  docker run -d \
    --name "$CONTAINER_NAME" \
    --label com.vla.role=ground-station \
    --restart unless-stopped \
    --network host \
    --env-file "$CONFIG_FILE" \
    "$IMAGE_NAME" >/dev/null
fi

for _ in $(seq 1 40); do
  if curl --fail --silent --show-error "$URL/api/health" |
      python3 -c 'import json,sys; data=json.load(sys.stdin); raise SystemExit(0 if data.get("control_output_enabled") is False and data.get("safety_lock") is True else 1)' \
      >/dev/null 2>&1; then
    echo "VLA ground station is ready: $URL"
    echo "LAN operator URL: http://192.168.5.4:8080"
    echo "Control output verified safety-locked."
    if command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
      xdg-open "$URL" >/dev/null 2>&1 || true
    fi
    exit 0
  fi
  sleep 0.5
done

echo "Ground station did not become ready in verified locked mode. Recent logs:" >&2
docker logs --tail 80 "$CONTAINER_NAME" >&2 || true
exit 1
