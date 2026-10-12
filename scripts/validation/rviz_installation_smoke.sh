#!/usr/bin/env bash
set -euo pipefail
workspace="${1:?workspace required}"
evidence_dir="${2:?external evidence directory required}"
mkdir -p "$evidence_dir"
for view in lidar_installation_204_only lidar_installation_205_only lidar_installation_rough extrinsic_candidate; do
  result=0
  LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a timeout --signal=INT 6s \
    rviz2 -d "$workspace/src/chili_crane_bringup/rviz/$view.rviz" > "$evidence_dir/$view.log" 2>&1 || result=$?
  [ "$result" = 0 ] || [ "$result" = 124 ]
  if grep -Eiq 'segmentation fault|pluginlib::|failed to load library|could not load class|parse error' "$evidence_dir/$view.log"; then
    cat "$evidence_dir/$view.log"
    exit 1
  fi
  grep -q 'OpenGl version' "$evidence_dir/$view.log"
done
printf 'RVIZ_CONFIGURATION_LOAD=PASS\nFIELD_OVERLAY=NOT_RUN\n'
