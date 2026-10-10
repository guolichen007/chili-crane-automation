#!/usr/bin/env sh
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
printf '%s\n' 'NO_DO_WRITE / read-only AF_PACKET; deployment owner supplies capture privilege'
exec python3 "$repo/scripts/sensors/er1_udp_probe.py" "$@"
