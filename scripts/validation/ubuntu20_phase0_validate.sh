#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 --workspace PATH --evidence-dir PATH --expected-sha SHA"
}

workspace=""
evidence_dir=""
expected_sha=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workspace)
      workspace="${2:-}"
      shift 2
      ;;
    --evidence-dir)
      evidence_dir="${2:-}"
      shift 2
      ;;
    --expected-sha)
      expected_sha="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$workspace" || -z "$evidence_dir" || -z "$expected_sha" ]]; then
  usage >&2
  exit 2
fi

if [[ ! "$expected_sha" =~ ^[0-9a-f]{40}$ ]]; then
  echo "--expected-sha must be a full 40-character lowercase Git SHA" >&2
  exit 2
fi

workspace="$(realpath "$workspace")"
if [[ ! -d "$workspace/.git" ]]; then
  echo "Workspace is not a Git repository: $workspace" >&2
  exit 2
fi

actual_sha="$(git -C "$workspace" rev-parse HEAD)"
if [[ "$actual_sha" != "$expected_sha" ]]; then
  echo "SHA mismatch: expected $expected_sha, got $actual_sha" >&2
  exit 1
fi

if [[ -n "$(git -C "$workspace" status --porcelain)" ]]; then
  echo "Workspace must be clean before validation" >&2
  exit 1
fi

mkdir -p "$evidence_dir"
evidence_dir="$(realpath "$evidence_dir")"
log_file="$evidence_dir/ubuntu20_phase0_validate.log"
status_file="$evidence_dir/validation_status.txt"
exec > >(tee "$log_file") 2>&1

echo "EVIDENCE_SHA: $actual_sha"
echo "WORKSPACE: $workspace"
echo "EVIDENCE_DIRECTORY: $evidence_dir"
date --iso-8601=seconds

if [[ ! -r /etc/os-release ]]; then
  echo "Cannot read /etc/os-release" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "20.04" ]]; then
  echo "Expected Ubuntu 20.04, got ${ID:-unknown} ${VERSION_ID:-unknown}" >&2
  exit 1
fi
if [[ "${VERSION:-}" != 20.04.6* ]]; then
  echo "Expected Ubuntu 20.04.6 LTS point release, got ${VERSION:-unknown}" >&2
  exit 1
fi
echo "OS: ${PRETTY_NAME:-Ubuntu 20.04}"

if [[ ! -r /opt/ros/noetic/setup.bash ]]; then
  echo "ROS Noetic setup file not found" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /opt/ros/noetic/setup.bash
if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "ROS_DISTRO must be noetic" >&2
  exit 1
fi
echo "ROS_DISTRO: $ROS_DISTRO"

python_version="$(python3 -c 'import sys; print("{}.{}".format(*sys.version_info[:2]))')"
if [[ "$python_version" != "3.8" ]]; then
  echo "Python 3.8 required, got $python_version" >&2
  exit 1
fi
echo "PYTHON: $(python3 --version 2>&1)"

for command in catkin catkin_test_results g++ rosdep roslaunch rostopic timeout; do
  if ! command -v "$command" >/dev/null 2>&1; then
    echo "Required command not found: $command" >&2
    exit 1
  fi
done
gcc_version="$(g++ -dumpfullversion -dumpversion)"
if [[ "${gcc_version%%.*}" != "9" ]]; then
  echo "GCC 9.x required, got $gcc_version" >&2
  exit 1
fi
echo "COMPILER: $(g++ --version | head -n 1)"

cd "$workspace"
rosdep check --from-paths src --ignore-src
catkin init
catkin clean -y
catkin config \
  --extend /opt/ros/noetic \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build --no-status --summarize
catkin test --no-status --summarize
catkin_test_results --all

# shellcheck disable=SC1091
source "$workspace/devel/setup.bash"
roslaunch --files chili_crane_bringup mock_system.launch \
  config_root:="$workspace/config"

launch_log="$evidence_dir/mock_system.launch.log"
roslaunch chili_crane_bringup mock_system.launch \
  config_root:="$workspace/config" >"$launch_log" 2>&1 &
launch_pid=$!
cleanup() {
  kill "$launch_pid" >/dev/null 2>&1 || true
  wait "$launch_pid" >/dev/null 2>&1 || true
}
trap cleanup EXIT

permit_file="$evidence_dir/safety_permit.yaml"
timeout 30 rostopic echo -n 1 /crane_01/safety/permit >"$permit_file"

for field in \
  allow_x_positive allow_x_negative \
  allow_y_positive allow_y_negative \
  allow_lower allow_raise allow_grab_open allow_grab_close \
  allow_unload allow_auto_task; do
  if ! grep -Eiq "^[[:space:]]*${field}:[[:space:]]*false[[:space:]]*$" \
    "$permit_file"; then
    echo "Safety permit did not keep $field false" >&2
    exit 1
  fi
done

hardware_topics=(
  servo_state
  trolley_state
  hoist_state
  load_state
  grab_io_state
  control_board_state
)
for topic in "${hardware_topics[@]}"; do
  state_file="$evidence_dir/${topic}.yaml"
  timeout 30 rostopic echo -n 1 "/crane_01/hardware/${topic}" >"$state_file"
  if ! grep -Eq "^[[:space:]]*validity:[[:space:]]*1[[:space:]]*$" \
    "$state_file"; then
    echo "Mock ${topic} is not NOT_CONFIGURED (validity=1)" >&2
    exit 1
  fi
done

cat >"$status_file" <<EOF
EVIDENCE_SHA: $actual_sha
UBUNTU_BUILD_STATUS: PASS
GTEST_STATUS: PASS
ROSLAUNCH_STATUS: PASS
MOCK_FAIL_CLOSED_STATUS: PASS
BAG_STATUS: NOT_RUN
FIELD_STATUS: NOT_RUN
EOF

echo "Ubuntu Phase 0 validation completed; evidence: $evidence_dir"
