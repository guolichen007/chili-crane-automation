#!/bin/sh
set -eu
base=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
exec python3 "$base/tools/ptp_packet_probe.py" "$@"
