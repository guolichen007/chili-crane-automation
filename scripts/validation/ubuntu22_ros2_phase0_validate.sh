#!/usr/bin/env bash
set -euo pipefail

workspace=""
evidence_dir=""
expected_sha=""
while [ "$#" -gt 0 ]; do
  case "$1" in
    --workspace) workspace="${2:?missing workspace}"; shift 2 ;;
    --evidence-dir) evidence_dir="${2:?missing evidence directory}"; shift 2 ;;
    --expected-sha) expected_sha="${2:?missing SHA}"; shift 2 ;;
    *) printf 'Unknown option: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[ -n "$workspace" ] && [ -n "$evidence_dir" ] && [ -n "$expected_sha" ]
workspace="$(realpath "$workspace")"
mkdir -p "$evidence_dir"
evidence_dir="$(realpath "$evidence_dir")"
case "$evidence_dir/" in "$workspace/"*) echo "Evidence must be outside the repository" >&2; exit 2 ;; esac
cd "$workspace"
[ "$(git rev-parse HEAD)" = "$expected_sha" ]
[ -z "$(git status --porcelain)" ] || { echo "Tracked/untracked tree is not clean" >&2; exit 2; }
[ "$(lsb_release -rs)" = "22.04" ] || { echo "Ubuntu 22.04 required" >&2; exit 2; }
set +u
. /opt/ros/humble/setup.bash
set -u
[ "${ROS_DISTRO:-}" = "humble" ]
[ "$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')" = "3.10" ]
gcc_version="$(gcc -dumpversion)"
[ "${gcc_version%%.*}" = "11" ]
for command in ros2 rosdep colcon timeout; do command -v "$command" >/dev/null; done
run_dir="$(mktemp -d "$evidence_dir/ros2-$expected_sha-XXXXXX")"
exec > >(tee "$run_dir/validation.log") 2>&1
printf 'EVIDENCE_SHA: %s\n' "$expected_sha"
printf 'EVIDENCE_SCOPE: UBUNTU22_ROS2_SOFTWARE_ONLY\n'
{
  printf 'IMAGE_REFERENCE: %s\n' "${CHILI_CI_IMAGE_REFERENCE:-NOT_REPORTED}"
  printf 'IMAGE_DIGEST: NOT_CAPTURED (CI tag remains floating)\n'
  printf 'ROS_DISTRO: %s\n' "$ROS_DISTRO"
  lsb_release -a
  python3 --version
  gcc --version
  ros2 pkg xml rclpy
} > "$run_dir/software-environment.txt" 2>&1
dpkg-query -W > "$run_dir/apt-package-versions.txt"
python3 tools/check_repo_contracts.py
python3 -m unittest discover -s tests_static -p "test_*.py"
python3 tools/validate_architecture_scenarios.py | tee "$run_dir/architecture-scenarios.json"
python3 tools/validate_phase2a_synthetic.py --output "$run_dir/phase2a-monte-carlo.json"
timeout 600s python3 tools/validate_phase2a_r1.py --output "$run_dir/phase2a-r1-sequences.json"
rosdep check --from-paths src --ignore-src --rosdistro humble
colcon --log-base "$run_dir/log" build --base-paths "$workspace/src" \
  --build-base "$run_dir/build" --install-base "$run_dir/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon --log-base "$run_dir/log" test --base-paths "$workspace/src" \
  --build-base "$run_dir/build" --install-base "$run_dir/install" \
  --return-code-on-test-failure
colcon test-result --test-result-base "$run_dir/build" --verbose
set +u
. "$run_dir/install/setup.bash"
set -u
timeout 35s python3 tools/validate_dual_lidar_ros.py | tee "$run_dir/dual-lidar-synthetic-ros.json"
timeout 35s python3 tools/validate_ptp_timescale_ros.py | tee "$run_dir/ptp-timescale-synthetic-ros.json"
timeout 35s python3 tools/validate_installation_calibration_ros.py --evidence-dir "$run_dir/phase1c" | tee "$run_dir/installation-capture-synthetic-ros.json"
bash scripts/validation/rviz_installation_smoke.sh "$workspace" "$run_dir/rviz" | tee "$run_dir/rviz-smoke-result.txt"
timeout 45s python3 tools/validate_phase2a_ros.py --evidence-dir "$run_dir/phase2a-normal" | tee "$run_dir/phase2a-normal-result.json"
# Independent install tree catches executable modes hidden by normal install copies.
colcon --log-base "$run_dir/symlink-log" build --symlink-install --base-paths "$workspace/src" \
  --build-base "$run_dir/symlink-build" --install-base "$run_dir/symlink-install"
(
  set +u
  . "$run_dir/symlink-install/setup.bash"
  set -u
  colcon --log-base "$run_dir/symlink-log" test --base-paths "$workspace/src" \
    --build-base "$run_dir/symlink-build" --install-base "$run_dir/symlink-install" \
    --return-code-on-test-failure
  colcon test-result --test-result-base "$run_dir/symlink-build" --verbose
  for package in chili_crane_slam chili_crane_hardware chili_crane_control chili_crane_perception; do
    ros2 pkg executables "$package"
  done
  timeout 35s python3 tools/validate_installation_calibration_ros.py --evidence-dir "$run_dir/symlink-phase1c"
  timeout 45s python3 tools/validate_phase2a_ros.py --evidence-dir "$run_dir/phase2a-symlink"
) 2>&1 | tee "$run_dir/symlink-executables-and-transport.log"
ros2 launch chili_crane_bringup mock_system.launch.py > "$run_dir/mock-launch.log" 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  # Bounded shutdown without deleting build/install or evidence.
  for attempt in 1 2 3 4 5; do
    kill -0 "$launch_pid" 2>/dev/null || break
    sleep 1
  done
  kill -TERM "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
timeout 35s python3 tools/validate_ros2_mock.py --timeout-sec 25 | tee "$run_dir/mock-evidence.json"
kill -0 "$launch_pid"
cat > "$run_dir/result.yaml" <<EOF
EVIDENCE_SHA: $expected_sha
ROS2_BUILD_STATUS: PASS
ROS2_TEST_STATUS: PASS
DUAL_LIDAR_SYNTHETIC_ROS_STATUS: PASS
PTP_TIMESCALE_SYNTHETIC_ROS_STATUS: PASS
INSTALLATION_CAPTURE_TRANSPORT_STATUS: PASS
RVIZ_CONFIGURATION_LOAD_STATUS: PASS
ROS2_MOCK_LAUNCH_STATUS: PASS
MOCK_FAIL_CLOSED_STATUS: PASS
MOCK_SCENARIO_STATUS: PASS
ADAM6052_LIVE_STATUS: NOT_RUN
ADAM6251_LIVE_STATUS: NOT_RUN
PULL_WIRE_LIVE_STATUS: NOT_RUN
PHYSICAL_DO_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
EOF
printf 'RESULT: %s\n' "$run_dir/result.yaml"
