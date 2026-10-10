#!/bin/sh
set -eu
iface=${1:-enp3s0}
case "$iface" in ''|*[!a-zA-Z0-9_.:-]*) echo 'Invalid interface' >&2; exit 2 ;; esac
printf 'READ_ONLY_PTP_CAPABILITY_PROBE interface=%s\n' "$iface"
for cmd in ethtool ip timedatectl systemctl; do
    command -v "$cmd" >/dev/null || { printf 'MISSING: %s\n' "$cmd"; exit 2; }
done
ethtool -T "$iface"
ip -details link show dev "$iface"
timedatectl show
for svc in ptp4l phc2sys chrony systemd-timesyncd; do
    systemctl --no-pager show "$svc" -p ActiveState -p SubState || true
done
for phc in /dev/ptp*; do
    [ -e "$phc" ] && ls -l "$phc"
done
printf 'PTP_LIVE_STATUS=NOT_RUN\nService state is not clock synchronization evidence.\n'
