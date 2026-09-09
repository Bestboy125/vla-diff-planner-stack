#!/usr/bin/env bash
set -Eeuo pipefail
umask 077

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONFIG_FILE="$ROOT_DIR/linux/config.env"
CREDENTIAL_FILE="${VLA_CONTROL_CREDENTIAL_FILE:-${HOME}/Desktop/vla_control_credentials.txt}"
CLOCK_CHECK="$ROOT_DIR/backend/tools/check_onboard_clock.py"
IMAGE_NAME="vla-ground-station:local"
CONTAINER_NAME="vla-ground-station"
URL="http://127.0.0.1:8080"

fail() {
  echo "Live start blocked: $*" >&2
  exit 1
}

live_env_file=''
cleanup() {
  if [[ -n "$live_env_file" && -f "$live_env_file" ]]; then
    rm -f -- "$live_env_file"
  fi
}
trap cleanup EXIT

config_value() {
  local key="$1"
  local count value
  count="$(grep -c "^${key}=" "$CONFIG_FILE" || true)"
  [[ "$count" == 1 ]] || fail "config.env must contain exactly one ${key} entry"
  value="$(sed -n "s/^${key}=//p" "$CONFIG_FILE")"
  value="${value%$'\r'}"
  [[ -n "$value" ]] || fail "${key} is empty"
  printf '%s' "$value"
}

[[ -f "$CONFIG_FILE" ]] || fail "missing $CONFIG_FILE"
[[ -f "$CREDENTIAL_FILE" ]] || fail "missing $CREDENTIAL_FILE"
[[ -f "$CLOCK_CHECK" ]] || fail "missing $CLOCK_CHECK"
[[ "$(stat -c '%a' "$CREDENTIAL_FILE")" == 600 ]] || fail 'credential file mode must be 600'
[[ "$(stat -c '%U' "$CREDENTIAL_FILE")" == "$(id -un)" ]] || fail 'credential file owner is incorrect'

mapfile -t credentials < "$CREDENTIAL_FILE"
[[ "${#credentials[@]}" == 2 ]] || fail 'credential file must contain exactly two lines'
operator_token="${credentials[0]}"
confirmation="${credentials[1]}"
[[ "$operator_token" =~ ^[[:xdigit:]]{64}$ ]] || fail 'operator token must be 64 hexadecimal characters'
[[ "$confirmation" == I_ACCEPT_REAL_FLIGHT_CONTROL ]] || fail 'confirmation phrase does not match'
[[ "$(config_value OPERATOR_CONTROL_TOKEN)" == "$operator_token" ]] || fail 'desktop token and config.env token do not match'
[[ "$(config_value LIVE_CONTROL_CONFIRMATION)" == "$confirmation" ]] || fail 'desktop phrase and config.env phrase do not match'
[[ "$(config_value CONTROL_OUTPUT_ENABLED)" == false ]] || fail 'config.env must retain the safe false default'

onboard_host="$(config_value ONBOARD_BRIDGE_HOST)"
onboard_port="$(config_value ONBOARD_BRIDGE_PORT)"
clock_limit_ms="$(config_value ONBOARD_MAX_CLOCK_OFFSET_MS)"
[[ "$onboard_host" =~ ^[A-Za-z0-9.-]+$ ]] || fail 'invalid onboard bridge host'
[[ "$onboard_port" =~ ^[0-9]+$ && "$onboard_port" -ge 1 && "$onboard_port" -le 65535 ]] || fail 'invalid onboard bridge port'
[[ "$clock_limit_ms" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail 'invalid clock offset limit'

timeout 3 bash -c "</dev/tcp/${onboard_host}/${onboard_port}" 2>/dev/null ||
  fail "onboard bridge is unavailable at ${onboard_host}:${onboard_port}"
python3 "$CLOCK_CHECK" --host "$onboard_host" --max-offset-ms "$clock_limit_ms" --samples 3 --interval 10 ||
  fail 'onboard clock preflight failed'

docker image inspect "$IMAGE_NAME" >/dev/null 2>&1 || fail "missing image $IMAGE_NAME"
desired_image_id="$(docker image inspect -f '{{.Id}}' "$IMAGE_NAME")"
if docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  role="$(docker inspect -f '{{index .Config.Labels "com.vla.role"}}' "$CONTAINER_NAME")"
  [[ "$role" == ground-station ]] || fail "unrecognized container named $CONTAINER_NAME"
  control_values="$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$CONTAINER_NAME" |
    sed -n 's/^CONTROL_OUTPUT_ENABLED=//p')"
  current_control="$(tail -n 1 <<<"$control_values")"
  control_count="$(wc -l <<<"$control_values" | tr -d ' ')"
  current_image_id="$(docker inspect -f '{{.Image}}' "$CONTAINER_NAME")"
  if [[ "$current_control" == true && "$control_count" == 1 && \
        "$current_image_id" == "$desired_image_id" && \
        "$(docker inspect -f '{{.State.Running}}' "$CONTAINER_NAME")" == true ]]; then
    echo 'Live ground station is already running; no container was replaced.'
  else
    docker rm -f "$CONTAINER_NAME" >/dev/null
  fi
fi

if ! docker container inspect "$CONTAINER_NAME" >/dev/null 2>&1; then
  live_env_file="$(mktemp "$ROOT_DIR/linux/.live-env.XXXXXX")"
  while IFS= read -r line; do
    line="${line%$'\r'}"
    case "$line" in
      CONTROL_OUTPUT_ENABLED=*) printf 'CONTROL_OUTPUT_ENABLED=true\n' ;;
      *) printf '%s\n' "$line" ;;
    esac
  done < "$CONFIG_FILE" > "$live_env_file"
  chmod 600 "$live_env_file"
  docker run -d \
    --name "$CONTAINER_NAME" \
    --label com.vla.role=ground-station \
    --restart unless-stopped \
    --network host \
    --env-file "$live_env_file" \
    "$IMAGE_NAME" >/dev/null
  rm -f -- "$live_env_file"
  live_env_file=''
fi

for _ in $(seq 1 40); do
  if curl --fail --silent --show-error "$URL/api/health" |
      python3 -c 'import json,sys; data=json.load(sys.stdin); raise SystemExit(0 if data.get("control_output_enabled") is True and data.get("safety_lock") is False else 1)' \
      >/dev/null 2>&1; then
    echo "Live ground station is ready: $URL"
    echo 'Operator token and confirmation phrase are still required for every Live request.'
    echo 'No mission, takeoff, arming, or trajectory command was sent.'
    if command -v xdg-open >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
      xdg-open "$URL" >/dev/null 2>&1 || true
    fi
    exit 0
  fi
  sleep 0.5
done

echo 'Live ground station did not become ready. Recent logs:' >&2
docker logs --tail 80 "$CONTAINER_NAME" >&2 || true
exit 1
