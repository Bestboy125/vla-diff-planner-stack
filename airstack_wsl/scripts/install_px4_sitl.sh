#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-/home/airstack/PX4-Autopilot}"
PX4_VENV="${PX4_VENV:-/home/airstack/.venvs/px4-v1.14.3}"
PX4_TAG="v1.14.3"
PX4_COMMIT="1dacb4cdef2d7145754fc788fa8dc482eed74b40"

ensure_archive_git_repo() {
  local destination="$1"
  local commit="$2"

  if [[ -f "${destination}/.git" ]]; then
    unlink "${destination}/.git"
  fi
  if [[ ! -d "${destination}/.git" ]]; then
    git -c init.defaultBranch=archive init -q "${destination}"
    git -C "${destination}" add -A
    git -C "${destination}" \
      -c user.name='PX4 archive installer' \
      -c user.email='px4-archive@localhost' \
      commit -q -m "Pinned source archive ${commit}"
  fi
}

install_archive_submodule() {
  local repository="$1"
  local commit="$2"
  local relative_path="$3"
  local destination="${PX4_ROOT}/${relative_path}"
  local marker="${destination}/.px4_archive_commit"

  if [[ -f "${marker}" ]] && grep -qx "${commit}" "${marker}"; then
    ensure_archive_git_repo "${destination}" "${commit}"
    printf 'submodule ready: %s @ %s\n' "${relative_path}" "${commit}"
    return
  fi

  mkdir -p "${destination}"
  curl --fail --location --retry 3 --retry-delay 2 \
    "https://codeload.github.com/${repository}/tar.gz/${commit}" \
    | tar -xz --strip-components=1 -C "${destination}"
  printf '%s\n' "${commit}" > "${marker}"
  # Give the extracted snapshot valid local Git metadata so parent-repository
  # version checks can traverse submodules without trying the network.
  ensure_archive_git_repo "${destination}" "${commit}"
  printf 'submodule installed: %s @ %s\n' "${relative_path}" "${commit}"
}

if [[ ! -d "${PX4_ROOT}/.git" ]]; then
  mkdir -p "$(dirname "${PX4_ROOT}")"
  GIT_CONFIG_GLOBAL=/dev/null git -c http.version=HTTP/1.1 clone \
    --branch "${PX4_TAG}" --depth 1 \
    https://github.com/PX4/PX4-Autopilot.git "${PX4_ROOT}"
fi

cd "${PX4_ROOT}"
actual_commit="$(git rev-parse HEAD)"
if [[ "${actual_commit}" != "${PX4_COMMIT}" ]]; then
  printf 'PX4 commit mismatch: expected %s, got %s\n' "${PX4_COMMIT}" "${actual_commit}" >&2
  exit 1
fi

# AirStack communicates with PX4 through MAVROS/MAVLink. The uXRCE-DDS
# client is unrelated to this route and would pull a second network-fetched
# Micro-CDR build during compilation, so omit it from this dedicated profile.
sed -i '/^CONFIG_MODULES_UXRCE_DDS_CLIENT=y$/d' boards/px4/sitl/default.px4board

# These are the submodules enabled by boards/px4/sitl/default.px4board for
# the simulator-independent `none` target. Codeload avoids unreliable Git
# smart-HTTP connections while preserving the exact commits pinned by PX4.
install_archive_submodule PX4/PX4-GPSDrivers \
  b3ffec3f173a4dcb4d9e604222a30215023e9880 src/drivers/gps/devices
install_archive_submodule mavlink/libevents \
  a9a3fc07abb8bd8eb6fbca64c35b479cab91ff35 src/lib/events/libevents
install_archive_submodule mavlink/mavlink \
  18955a04c7c7467e00ea42b704addb4a9c12b53a src/modules/mavlink/mavlink
install_archive_submodule PX4/Micro-XRCE-DDS-Client \
  4248559f3b111155c783e524e461ccc83e768103 src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client
install_archive_submodule ArduPilot/pymavlink \
  2ca2c13b54b4c75dd71c79acafc7ec40d9cb4965 src/modules/mavlink/mavlink/pymavlink
install_archive_submodule nlohmann/json \
  bc889afb4c5bf1c0d8ee29ef35eaaf4c8bef8a5d src/lib/events/libevents/libs/cpp/parse/nlohmann_json

python3 -m venv "${PX4_VENV}"
requirements_marker="${PX4_VENV}/.px4-v1.14.3-requirements-ready"
if [[ ! -f "${requirements_marker}" ]]; then
  "${PX4_VENV}/bin/python" -m pip install --upgrade 'pip<25' wheel
  # PX4 v1.14.3 predates modern pip's rejection of the old
  # `matplotlib>=3.0.*` specifier. Normalize only that legacy line.
  requirements_tmp="$(mktemp)"
  trap 'rm -f "${requirements_tmp}"' EXIT
  sed 's/^matplotlib>=3\.0\.\*$/matplotlib>=3.0/' Tools/setup/requirements.txt > "${requirements_tmp}"
  "${PX4_VENV}/bin/python" -m pip install 'empy==3.3.4' 'numpy<2' -r "${requirements_tmp}"
  touch "${requirements_marker}"
fi

# Archive-backed submodules deliberately do not contain Git metadata.
# PX4 supports this environment switch for source snapshots.
export GIT_SUBMODULES_ARE_EVIL=1
export PATH="${PX4_VENV}/bin:${PATH}"

# PX4 v1.14.3's CMake function resolves relative .git dependencies inside
# the binary tree. Mirror the immutable markers there so Ninja can satisfy
# those dependencies without contacting the network.
for relative_path in \
  src/drivers/gps/devices \
  src/lib/events/libevents \
  src/modules/mavlink/mavlink \
  src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client; do
  binary_marker="${PX4_ROOT}/build/px4_sitl_default/${relative_path}/.git"
  mkdir -p "$(dirname "${binary_marker}")"
  printf 'archive-submodule\n' > "${binary_marker}"
done

# Build only. The additional Make target `none` starts an interactive PX4
# process and belongs in the runtime script, not in an idempotent installer.
make px4_sitl_default

test -x "${PX4_ROOT}/build/px4_sitl_default/bin/px4"
printf 'PX4 build ready: %s (%s)\n' "${PX4_TAG}" "${PX4_COMMIT}"
sha256sum "${PX4_ROOT}/build/px4_sitl_default/bin/px4"
