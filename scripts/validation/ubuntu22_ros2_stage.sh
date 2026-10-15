#!/bin/sh
set -eu
mode="${1:?mode required}"
workspace="${2:?workspace required}"
run_dir="${3:?evidence directory required}"
expected_sha="${4:?SHA required}"
cd "$workspace"
set +u
. /opt/ros/humble/setup.sh
set -u
[ "$ROS_DISTRO" = humble ]
case "$mode" in
  normal) build="$run_dir/build"; install="$run_dir/install"; log="$run_dir/log"; flag="" ;;
  symlink) build="$run_dir/symlink-build"; install="$run_dir/symlink-install"; log="$run_dir/symlink-log"; flag="--symlink-install" ;;
  *) echo "Unknown validation stage" >&2; exit 2 ;;
esac
# No prior installation may be present even at build/configuration time.
python3 tools/check_ros_overlay.py --forbidden-root "$run_dir/install" --forbidden-root "$run_dir/build" \
  --forbidden-root "$run_dir/symlink-install" --forbidden-root "$run_dir/symlink-build" --output "$run_dir/$mode-underlay.json"
if [ "$mode" = normal ]; then
  python3 tools/check_repo_contracts.py
  python3 tools/check_public_secrets.py
  python3 -m compileall -q src tools tests_static
  python3 -m unittest discover -s tests_static -p 'test_*.py'
  python3 -m pytest -q tests_static > "$run_dir/pytest-static.txt"
  cat "$run_dir/pytest-static.txt"
  python3 tools/validate_architecture_scenarios.py > "$run_dir/architecture-scenarios.json"
  cat "$run_dir/architecture-scenarios.json"
  python3 tools/validate_phase2a_synthetic.py --output "$run_dir/phase2a-monte-carlo.json"
  timeout 600s python3 tools/validate_phase2a_r1.py --output "$run_dir/phase2a-r1-sequences.json"
  timeout 600s python3 tools/validate_phase2b_synthetic.py --output "$run_dir/phase2b-sequences.json"
  rosdep check --from-paths src --ignore-src --rosdistro humble
fi
colcon --log-base "$log" build --base-paths "$workspace/src" $flag \
  --build-base "$build" --install-base "$install" --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
set +u
. "$install/local_setup.sh"
set -u
if [ "$mode" = symlink ]; then forbidden_install="$run_dir/install"; forbidden_build="$run_dir/build"
else forbidden_install="$run_dir/symlink-install"; forbidden_build="$run_dir/symlink-build"; fi
python3 tools/check_ros_overlay.py --expected-root "$install" --forbidden-root "$forbidden_install" \
  --forbidden-root "$forbidden_build" --output "$run_dir/$mode-package-resolution.json"
package_test_status=0
colcon --log-base "$log" test --base-paths "$workspace/src" --build-base "$build" --install-base "$install" \
  --return-code-on-test-failure || package_test_status=$?
colcon test-result --test-result-base "$build" --verbose
[ "$package_test_status" = 0 ]
timeout 180s python3 tools/validate_phase2b_r1.py --output "$run_dir/phase2b-r1-$mode.json"
for package in chili_crane_slam chili_crane_hardware chili_crane_control chili_crane_perception chili_crane_commissioning; do
  ros2 pkg executables "$package"
done
if [ "$mode" = normal ]; then
  timeout 35s python3 tools/validate_dual_lidar_ros.py > "$run_dir/dual-lidar-synthetic-ros.json"
  timeout 35s python3 tools/validate_ptp_timescale_ros.py > "$run_dir/ptp-timescale-synthetic-ros.json"
  bash scripts/validation/rviz_installation_smoke.sh "$workspace" "$run_dir/rviz" > "$run_dir/rviz-smoke-result.txt"
fi
timeout 35s python3 tools/validate_installation_calibration_ros.py --evidence-dir "$run_dir/$mode-phase1c" > "$run_dir/installation-$mode.json"
timeout 45s python3 tools/validate_phase2a_ros.py --evidence-dir "$run_dir/phase2a-$mode" > "$run_dir/phase2a-$mode-result.json"
timeout 65s python3 tools/validate_phase2b_ros.py --evidence-dir "$run_dir/phase2b-$mode" > "$run_dir/phase2b-$mode-result.json"
timeout 150s python3 tools/validate_phase2b_r11_ros.py --evidence-dir "$run_dir/phase2b-r11-$mode" > "$run_dir/phase2b-r11-$mode-result.json"
# Explicit short-path stability probe precedes the complete R2 suite.
timeout 300s python3 tools/validate_phase2c_ros.py --probe 10 --evidence-dir "$run_dir/phase2c-r2-probe-$mode" > "$run_dir/phase2c-r2-probe-$mode.json"
cat "$run_dir/phase2c-r2-probe-$mode.json"
timeout 300s python3 tools/validate_phase2c_ros.py --evidence-dir "$run_dir/phase2c-r2-$mode" > "$run_dir/phase2c-r2-$mode.json"
cat "$run_dir/phase2c-r2-$mode.json"
if [ "$mode" = normal ]; then
  baseline_dir="$(mktemp -d "$run_dir/phase2b-r11-baseline-XXXXXX")"
  git archive 8ee41886dfb8d084b2b01d505b770c98bf2fefde config src tools/validate_phase2b_r1.py | tar -x -C "$baseline_dir"
  timeout 180s python3 "$baseline_dir/tools/validate_phase2b_r1.py" --output "$run_dir/phase2b-r11-base-before.json"
  timeout 180s python3 tools/validate_phase2b_r1.py --output "$run_dir/phase2b-r11-current-paired.json"
  timeout 180s python3 "$baseline_dir/tools/validate_phase2b_r1.py" --output "$run_dir/phase2b-r11-base-after.json"
  sh scripts/validation/rviz_phase2b_smoke.sh "$workspace" "$run_dir/phase2b-rviz"
  python3 tools/validate_mock_owned.py --evidence-dir "$run_dir"
fi
printf 'STAGE=%s SHA=%s RESULT=PASS_SOFTWARE_SHADOW_NOT_FIELD\n' "$mode" "$expected_sha"
