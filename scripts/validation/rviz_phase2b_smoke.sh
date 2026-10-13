#!/bin/sh
set -eu
workspace="${1:?workspace required}"
evidence_dir="${2:?external evidence directory required}"
mkdir -p "$evidence_dir"
result=0
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -a timeout --kill-after=3s --signal=INT 6s \
  rviz2 -d "$workspace/src/chili_crane_bringup/rviz/phase2b_surface_shadow.rviz" > "$evidence_dir/phase2b-rviz.log" 2>&1 || result=$?
[ "$result" = 0 ] || [ "$result" = 124 ]
if grep -Eiq 'segmentation fault|pluginlib::|failed to load library|could not load class|parse error' "$evidence_dir/phase2b-rviz.log"; then
  exit 1
fi
grep -q 'OpenGl version' "$evidence_dir/phase2b-rviz.log"
printf 'PHASE2B_RVIZ_CONFIGURATION_LOAD=PASS\nFIELD_OVERLAY=NOT_RUN\n'
