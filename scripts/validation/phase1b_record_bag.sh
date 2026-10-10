#!/bin/sh
set -eu
base=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
[ "$#" -eq 2 ] || { echo 'usage: phase1b_record_bag.sh PREPARED_ARTIFACT_DIR SECONDS' >&2; exit 2; }
artifact=$(realpath "$1")
seconds=$2
case "$seconds" in ''|*[!0-9]*) echo 'integer duration required' >&2; exit 2 ;; esac
[ "$seconds" -ge 5 ] && [ "$seconds" -le 3600 ]
set +e
timeout --signal=INT --kill-after=15s "${seconds}s" ros2 launch chili_crane_bringup phase1b_sensor_record.launch.py artifact_dir:="$artifact"
result=$?
set -e
[ "$result" -eq 0 ] || [ "$result" -eq 124 ] || exit "$result"
python3 "$base/tools/phase1b_manifest.py" finalize --output "$artifact"
