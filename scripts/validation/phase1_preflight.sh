#!/usr/bin/env sh
set -eu
repo=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$repo"
[ "$(lsb_release -rs)" = 22.04 ]
[ "${ROS_DISTRO:-}" = humble ]
[ "$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')" = 3.10 ]
python3 tools/check_repo_contracts.py
python3 tools/check_public_secrets.py
git rev-parse HEAD
id -nG
ls -l /dev/serial/by-id/usb-WCH.CN_USB_Quad_Serial_BD89B1ABCD-if06
printf '%s\n' 'PREFLIGHT_ONLY / NO_DO_WRITE / no hardware communication performed'
