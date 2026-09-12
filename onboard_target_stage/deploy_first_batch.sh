#!/usr/bin/env bash
# Install source only. Never starts ROS, MAVROS, planners or a flight controller.
set -euo pipefail
repo=/home/nv/Diff-planner
stage="$(cd -- "$(dirname -- "$0")/.." && pwd)"
[[ -d "$repo/src/integration/vla_diff_bridge" ]] || exit 2
if pgrep -f '(^|/)(rosmaster|px4ctrl)( |$)|vla_diff_bridge_node.py' >/dev/null; then
  echo 'Refusing installation while the control stack is running.' >&2; exit 2
fi
check() {
  [[ "$(sha256sum "$repo/$1" | cut -d ' ' -f 1)" == "$2" ]] || {
    echo "Deployment baseline changed: $1; inspect before replacing." >&2; exit 3;
  }
}
check src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py ec158107861668f58506418a5241f1c4264d038daf552f5ea7899d63b9cf8e73
check src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py dc39b9cea5ac1f2013015f957553137cc2c1649ea88949bbeac5f0c32c738cbf
check src/perception/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py 74f8933d6e287c8510f0c1fc9166d7939c9f133ea97e9b37970c8d5519b431dd
check sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh 7a81582fdd0cee75fca471754a724787adb9c4d12091babbedea5eb5d10ca950
[[ ! -e "$repo/src/perception/target_interaction" ]] || { echo 'Target package already exists; inspect first.'; exit 3; }
backup="$(mktemp -d /home/nv/.local/state/target_first_batch_backup_XXXXXX)"
cd "$repo"
cp --parents src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py \
  src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py \
  src/perception/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py \
  sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh "$backup/"
install -m 755 "$stage/Diff-Planner/src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py" src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py
install -m 644 "$stage/Diff-Planner/src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py" src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py
install -m 755 "$stage/onboard_semantic_stage/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py" src/perception/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py
install -m 755 "$stage/Diff-Planner/sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh" sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
cp -a "$stage/onboard_target_stage/target_interaction" src/perception/target_interaction
find src/perception/target_interaction -type f \( -name '*.py' -o -name '*.xml' -o -name '*.launch' -o -name '*.txt' \) -exec sed -i 's/\r$//' {} +
sed -i 's/\r$//' src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py src/perception/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
chmod +x src/perception/target_interaction/scripts/target_interaction_node.py
python3 -m compileall -q src/perception/target_interaction/scripts src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py src/perception/semantic_raw_stereo_localizer/scripts/semantic_raw_stereo_node.py
bash -n sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
printf 'Source installed. Backup: %s\nNo flight processes started.\n' "$backup"
