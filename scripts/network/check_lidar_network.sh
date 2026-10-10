#!/usr/bin/env sh
set -eu
printf '%s\n' 'READ_ONLY_NETWORK_CHECK / NO_DO_WRITE'
ip -4 address show dev enp3s0
ip -4 route get 192.168.1.204
ip -4 route get 192.168.1.205
ip -4 route get 10.0.0.1
nmcli -g ipv4.addresses,ipv4.gateway,ipv4.dns,ipv4.never-default connection show SENSOR-LIDAR
ip -4 route get 192.168.1.204 | grep -q 'dev enp3s0'
ip -4 route get 192.168.1.205 | grep -q 'dev enp3s0'
ip -4 route get 10.0.0.1 | grep -q 'dev enp4s0'
[ "$(nmcli -g ipv4.never-default connection show SENSOR-LIDAR)" = yes ]
[ -z "$(nmcli -g ipv4.gateway connection show SENSOR-LIDAR)" ]
[ -z "$(nmcli -g ipv4.dns connection show SENSOR-LIDAR)" ]
