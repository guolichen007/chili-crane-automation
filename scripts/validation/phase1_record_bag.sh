#!/usr/bin/env sh
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
if [ "$#" -lt 3 ]; then
  printf '%s\n' 'usage: phase1_record_bag.sh <new-artifact-dir> <PHYSICAL|SIMULATED|REPLAY|SYNTHETIC> <scene> [duration-seconds]' >&2
  exit 2
fi
artifact=$1
source=$2
scene=$3
duration=${4:-30}
case "$duration" in ''|*[!0-9]*) exit 2 ;; esac
[ "$duration" -ge 5 ] && [ "$duration" -le 3600 ]
python3 "$repo/tools/phase1_manifest.py" create --output "$artifact" --source "$source" --scene "$scene"
result=0
timeout --signal=INT --kill-after=15 "$duration" ros2 launch chili_crane_bringup phase1_lidar_record.launch.py artifact_dir:="$artifact" || result=$?
if [ "$result" -ne 0 ] && [ "$result" -ne 124 ]; then
  printf '%s\n' 'recording failed; evidence retained' >&2
  exit "$result"
fi
python3 "$repo/tools/phase1_manifest.py" finalize --output "$artifact"
printf '%s\n' 'NO_DO_WRITE / capture complete; field and calibration acceptance unchanged'
