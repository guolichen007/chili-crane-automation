#!/bin/sh
set -eu
base=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
[ "$#" -eq 2 ] || { echo 'usage: ptp_dry_run.sh hardware|software UDPv4|L2' >&2; exit 2; }
case "$1" in hardware) config=ptp_hardware_gm.conf ;; software) config=ptp_software_gm.conf ;; *) exit 2 ;; esac
case "$2" in UDPv4) transport=-4 ;; L2) transport=-2 ;; *) exit 2 ;; esac
printf 'DRY_RUN_ONLY: owner must inspect NIC capabilities, packet delay mechanism and competing time services.\n'
printf 'Candidate (NOT EXECUTED): ptp4l -f "%s/config/time/%s" -i enp3s0 %s -A -m\n' "$base" "$config" "$transport"
printf 'No systemd enable, no clock changes, no phc2sys invocation. Service start does not imply VALID.\n'
