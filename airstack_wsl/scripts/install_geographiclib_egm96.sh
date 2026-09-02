#!/usr/bin/env bash
set -euo pipefail

# SourceForge is heavily throttled on a single connection from this host.  The
# archive supports byte ranges, so fetch fixed, verified chunks in parallel.
URL="${EGM96_URL:-https://netix.dl.sourceforge.net/project/geographiclib/geoids-distrib/egm96-5.tar.bz2}"
EXPECTED_SIZE=10225152
CHUNK_SIZE=524288
CACHE_ROOT="${EGM96_CACHE_ROOT:-/var/cache/airstack/geographiclib}"
PARTS_DIR="${CACHE_ROOT}/egm96-5.parts"
ARCHIVE="${CACHE_ROOT}/egm96-5.tar.bz2"
DEST="/usr/share/GeographicLib"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "ERROR: run this installer as root" >&2
  exit 10
fi
if [[ -r "${DEST}/geoids/egm96-5.pgm" ]]; then
  echo "EGM96_READY path=${DEST}/geoids/egm96-5.pgm"
  exit 0
fi

mkdir -p "${PARTS_DIR}" "${DEST}"
export URL EXPECTED_SIZE CHUNK_SIZE PARTS_DIR

fetch_part() {
  local index="$1" start end expected output actual
  start=$((index * CHUNK_SIZE))
  end=$((start + CHUNK_SIZE - 1))
  if (( end >= EXPECTED_SIZE )); then end=$((EXPECTED_SIZE - 1)); fi
  expected=$((end - start + 1))
  output="${PARTS_DIR}/part-$(printf '%03d' "${index}")"
  if [[ -f "${output}" ]] && [[ "$(stat -c %s "${output}")" -eq "${expected}" ]]; then
    return 0
  fi
  curl --fail --location --retry 5 --retry-delay 1 \
    --connect-timeout 10 --max-time 240 \
    --range "${start}-${end}" --output "${output}.tmp" "${URL}"
  actual="$(stat -c %s "${output}.tmp")"
  if [[ "${actual}" -ne "${expected}" ]]; then
    echo "ERROR: part ${index} expected ${expected} bytes, received ${actual}" >&2
    exit 20
  fi
  mv "${output}.tmp" "${output}"
}
export -f fetch_part

PART_COUNT=$(((EXPECTED_SIZE + CHUNK_SIZE - 1) / CHUNK_SIZE))
seq 0 $((PART_COUNT - 1)) | xargs -n1 -P16 bash -c 'fetch_part "$1"' _

rm -f "${ARCHIVE}.tmp"
for index in $(seq 0 $((PART_COUNT - 1))); do
  part="${PARTS_DIR}/part-$(printf '%03d' "${index}")"
  cat "${part}" >>"${ARCHIVE}.tmp"
done
[[ "$(stat -c %s "${ARCHIVE}.tmp")" -eq "${EXPECTED_SIZE}" ]]
bzip2 -t "${ARCHIVE}.tmp"
mv "${ARCHIVE}.tmp" "${ARCHIVE}"
tar -xjf "${ARCHIVE}" -C "${DEST}"
test -r "${DEST}/geoids/egm96-5.pgm"

echo "EGM96_READY path=${DEST}/geoids/egm96-5.pgm bytes=$(stat -c %s "${DEST}/geoids/egm96-5.pgm")"
