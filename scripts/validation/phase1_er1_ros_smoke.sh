#!/usr/bin/env sh
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
exec python3 "$repo/tools/phase1_ros_smoke.py" --mode raw "$@"
