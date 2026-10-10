#!/usr/bin/env sh
set -eu
if [ "$#" -ne 1 ] || [ "$1" != "--dry-run" ]; then
  printf '%s\n' 'Only --dry-run is supported. Deployment owner applies reviewed commands.' >&2
  exit 2
fi
printf '%s\n' 'No network settings changed. Review cable/interface identity first.'
printf '%s\n' 'sudo nmcli connection add type ethernet ifname enp3s0 con-name SENSOR-LIDAR ipv4.method manual ipv4.addresses 192.168.1.10/24 ipv4.never-default yes ipv4.gateway "" ipv4.dns "" ipv6.method disabled'
printf '%s\n' 'sudo nmcli connection up SENSOR-LIDAR'
