#!/usr/bin/env bash
# Run unit and offline ROS integration checks against this workspace only.
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX ROS_NAMESPACE
unset ROS_DISCOVERY_SERVER FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE CYCLONEDDS_URI
unset RMW_IMPLEMENTATION
source /opt/ros/humble/setup.bash
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace is not built; run $workspace_dir/scripts/build.sh first." >&2
  exit 1
fi
source "$workspace_dir/install/setup.bash"
cd "$workspace_dir"
mkdir -p "$workspace_dir/log/unit_ros"
export ROS_LOG_DIR="$workspace_dir/log/unit_ros"
test_status=0
/usr/bin/colcon test --return-code-on-test-failure --event-handlers console_direct+ "$@" || test_status=$?
/usr/bin/colcon test-result --verbose || test_status=$?
if (( test_status != 0 )); then
  exit "$test_status"
fi
exec /usr/bin/python3 "$workspace_dir/tests/run_integration.py"
