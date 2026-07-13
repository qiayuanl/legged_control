#!/usr/bin/env bash
set -euo pipefail

if ! command -v isaacsim >/dev/null 2>&1; then
  echo "isaacsim is not on PATH. Activate the Isaac Sim Python environment first." >&2
  exit 1
fi

# Avoid mixing a sourced ROS 1 environment with Isaac Sim's ROS 2 bridge.
unset ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec isaacsim --enable isaacsim.ros2.bridge "$@"
