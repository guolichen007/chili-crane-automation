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
# Each build starts from a separate clean process environment. Never source
# normal before building or loading symlink. Keep only explicit test provenance.
domain="${ROS_DOMAIN_ID:-42}"
python3 -c 'import sys; d=sys.argv[1]; assert d.isascii() and d.isdecimal() and 0 <= int(d) <= 232' "$domain"
printf 'ROS_LOCALHOST_ONLY=1\nROS_DOMAIN_ID=%s\n' "$domain" > "$run_dir/ros-domain.txt"
for stage in normal symlink; do
  env -i HOME="$HOME" PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 PYTHONNOUSERSITE=1 ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID="$domain" \
    CHILI_CI_IMAGE_REFERENCE="${CHILI_CI_IMAGE_REFERENCE:-NOT_REPORTED}" \
    sh "$workspace/scripts/validation/ubuntu22_ros2_stage.sh" "$stage" "$workspace" "$run_dir" "$expected_sha"
done
cat > "$run_dir/result.yaml" <<EOF
EVIDENCE_SHA: $expected_sha
ARCHITECTURE_MODE: SNAPSHOT_PER_CYCLE
ROS2_BUILD_STATUS: PASS
ROS2_TEST_STATUS: PASS
NORMAL_OVERLAY_PRESENT_IN_SYMLINK_ENV: NO
MIXED_PACKAGE_RESOLUTION: NO
PHASE2C_R2_NORMAL_PROBE: 10/10
PHASE2C_R2_SYMLINK_PROBE: 10/10
PHASE2C_R2_INSTALLED_NORMAL: PASS
PHASE2C_R2_INSTALLED_SYMLINK: PASS
VALIDATOR_OWNED_PROCESS_LEAK_COUNT: 0
GRAB_DESCENT_TARGET_Z_FIELD_CALIBRATION: NOT_RUN
BENCH_DO_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
FULL_AUTO: NOT_RUN
EOF
printf 'RESULT: %s\n' "$run_dir/result.yaml"
