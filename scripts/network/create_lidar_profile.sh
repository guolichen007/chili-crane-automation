#!/usr/bin/env sh
set -eu
if [ "$#" -eq 0 ] || [ "$1" != "--dry-run" ]; then
  printf '%s\n' 'Only --dry-run is supported. Deployment owner applies reviewed commands.' >&2
  exit 2
fi
shift
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
exec python3 "$ROOT/tools/sensor_network_contract.py" dry-run "$@"
