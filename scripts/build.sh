#!/usr/bin/env bash
# Build against the system Humble underlay rather than a previously sourced workspace.
set -eo pipefail
workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export PYTHONNOUSERSITE=1
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX
source /opt/ros/humble/setup.bash
cd "$workspace_dir"
exec /usr/bin/colcon build --symlink-install --parallel-workers 2 \
  "$@" --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
