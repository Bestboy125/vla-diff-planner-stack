#!/usr/bin/env bash
# Deploy only; no ROS or flight processes launched, no calibration gate enabled.
set -euo pipefail
repo=/home/nv/Diff-planner
stage="$(cd -- "$(dirname -- "$0")/.." && pwd)"
if pgrep -f '(^|/)(rosmaster|px4ctrl)( |$)|vla_diff_bridge_node.py' >/dev/null; then
  echo 'Refusing source changes while control stack is running.'; exit 2
fi
check() { [[ "$(sha256sum "$repo/$1" | cut -d ' ' -f 1)" == "$2" ]] || { echo "Baseline changed: $1"; exit 3; }; }
check src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py 6482ee6c1152b6c91130f549134cfd62b48e5cb274b85384045f541b728b9f8f
check src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py 298c45742dac71cdf968deda5ad6d55fe76bae1fe3e5d884e0f8782b6728913f
check sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh d6f4d1cdac682641f1b8c8fb9e3be77d59adf82f472e5258fa38386cfd039638
for file in package.xml CMakeLists.txt launch/target_interaction.launch scripts/target_interaction_node.py scripts/target_geometry.py; do
  baseline="$(tar -xOf /home/nv/target_first_batch_20260912.tar.gz "onboard_target_stage/target_interaction/$file" | sed 's/\r$//' | sha256sum | cut -d ' ' -f 1)"
  check "src/perception/target_interaction/$file" "$baseline"
done
backup="$(mktemp -d /home/nv/.local/state/target_second_batch_backup_XXXXXX)"
cd "$repo"
cp --parents src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh "$backup/"
cp -a src/perception/target_interaction "$backup/target_interaction"
install -m 755 "$stage/Diff-Planner/src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py" src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py
install -m 644 "$stage/Diff-Planner/src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py" src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py
install -m 755 "$stage/Diff-Planner/sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh" sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
cp -a "$stage/onboard_target_stage/target_interaction/." src/perception/target_interaction/
find src/perception/target_interaction -type f \( -name '*.py' -o -name '*.xml' -o -name '*.launch' -o -name '*.txt' \) -exec sed -i 's/\r$//' {} +
sed -i 's/\r$//' src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
chmod +x src/perception/target_interaction/scripts/target_interaction_node.py
python3 -m compileall -q src/perception/target_interaction/scripts src/integration/vla_diff_bridge/scripts/vla_diff_bridge_node.py src/integration/vla_diff_bridge/src/vla_diff_bridge/protocol.py
bash -n sh_files/run_diff_px4ctrl_multipoint_vla_preview.sh
printf 'Installed second batch. Backup: %s\nLanding calibration gate remains false by default. No flight processes started.\n' "$backup"
