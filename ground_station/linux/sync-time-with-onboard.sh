#!/usr/bin/env bash
set -Eeuo pipefail

ONBOARD_NTP_HOST="${1:-192.168.5.5}"
MAX_OFFSET_MS="${VLA_ONBOARD_MAX_CLOCK_OFFSET_MS:-300}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLOCK_CHECK="$ROOT_DIR/backend/tools/check_onboard_clock.py"
DROP_IN_DIR=/etc/systemd/timesyncd.conf.d
DROP_IN_FILE="$DROP_IN_DIR/vla-onboard.conf"

fail() {
  echo "Time synchronization failed: $*" >&2
  exit 1
}

[[ "$ONBOARD_NTP_HOST" =~ ^[A-Za-z0-9.-]+$ ]] || fail 'invalid onboard NTP host'
[[ "$MAX_OFFSET_MS" =~ ^[0-9]+([.][0-9]+)?$ ]] || fail 'invalid clock offset limit'
[[ -f "$CLOCK_CHECK" ]] || fail "missing $CLOCK_CHECK"
systemctl list-unit-files systemd-timesyncd.service >/dev/null 2>&1 ||
  fail 'systemd-timesyncd is unavailable'

echo "Checking onboard NTP server at ${ONBOARD_NTP_HOST}:123..."
# The first probe checks reachability. A wide limit allows this tool to repair a
# clock that is initially far from the onboard clock.
python3 "$CLOCK_CHECK" \
  --host "$ONBOARD_NTP_HOST" \
  --max-offset-ms 86400000 \
  --samples 1 \
  --interval 10 || fail 'onboard NTP server is unreachable or invalid; system configuration was not changed'

sudo -v
temporary="$(mktemp)"
trap 'rm -f -- "$temporary"' EXIT
cat > "$temporary" <<EOF
[Time]
NTP=${ONBOARD_NTP_HOST}
PollIntervalMinSec=16
PollIntervalMaxSec=64
EOF

sudo install -d -m 0755 -o root -g root "$DROP_IN_DIR"
sudo install -m 0644 -o root -g root "$temporary" "$DROP_IN_FILE"
sudo timedatectl set-ntp true
sudo systemctl restart systemd-timesyncd.service

echo 'Waiting for systemd-timesyncd to select the onboard server...'
selected=false
for _ in $(seq 1 30); do
  server_address="$(timedatectl show-timesync --property=ServerAddress --value 2>/dev/null || true)"
  synchronized="$(timedatectl show --property=NTPSynchronized --value 2>/dev/null || true)"
  if [[ "$server_address" == "$ONBOARD_NTP_HOST" && "$synchronized" == yes ]]; then
    selected=true
    break
  fi
  sleep 1
done
[[ "$selected" == true ]] || fail 'timesyncd did not select and synchronize with the onboard server within 30 seconds'

echo "Verifying three clock samples within +/-${MAX_OFFSET_MS} ms..."
python3 "$CLOCK_CHECK" \
  --host "$ONBOARD_NTP_HOST" \
  --max-offset-ms "$MAX_OFFSET_MS" \
  --samples 3 \
  --interval 10 || fail 'strict post-sync clock verification failed'

echo "PASS: laptop clock is synchronized with ${ONBOARD_NTP_HOST}."
