#!/usr/bin/env bash
set -euo pipefail

# Portable, non-GUI equivalent of the development start_isaac_legged_stack.sh.
# The repository must already be built in the container's catkin workspace.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

CONTAINER_NAME="${CONTAINER_NAME:-ros1_noetic_legged}"
ROS1_WS="${ROS1_WS:-/root/legged_ws}"
ROS2_SOCKET_HOST="${ROS2_SOCKET_HOST:-172.17.0.1}"
ROS2_SOCKET_PORT="${ROS2_SOCKET_PORT:-50055}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LOG_DIR="${ISAAC_STACK_LOG_DIR:-/tmp/isaac_legged_stack}"
HOST_BRIDGE="${REPO_ROOT}/isaac_sim/scripts/ros2_isaac_socket_bridge.py"

mkdir -p "${LOG_DIR}"

container_ros() {
  docker exec "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/noetic/setup.bash && cd '${ROS1_WS}' && source devel/setup.bash && $*"
}

container_ros_detached() {
  docker exec -d "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/noetic/setup.bash && cd '${ROS1_WS}' && source devel/setup.bash && $*"
}

wait_for_container_ros() {
  local description="$1"
  local check="$2"
  local attempts="${3:-60}"
  printf 'Waiting for %s' "${description}"
  for ((i = 0; i < attempts; i++)); do
    if container_ros "${check}" >/dev/null 2>&1; then
      printf ' ready.\n'
      return 0
    fi
    printf '.'
    sleep 1
  done
  printf '\nTimed out waiting for %s.\n' "${description}" >&2
  return 1
}

command -v docker >/dev/null 2>&1 || {
  echo "docker is required." >&2
  exit 1
}
[[ -f "${HOST_BRIDGE}" ]] || {
  echo "Missing host bridge: ${HOST_BRIDGE}" >&2
  exit 1
}

echo "Starting container ${CONTAINER_NAME}..."
docker start "${CONTAINER_NAME}" >/dev/null
docker exec "${CONTAINER_NAME}" mkdir -p "${LOG_DIR}"

existing_bridge_pid=""
if [[ -f "${LOG_DIR}/ros2_bridge.pid" ]] && \
   kill -0 "$(<"${LOG_DIR}/ros2_bridge.pid")" 2>/dev/null; then
  existing_bridge_pid="$(<"${LOG_DIR}/ros2_bridge.pid")"
else
  existing_bridge_pid="$(pgrep -f '[r]os2_isaac_socket_bridge.py' | head -n 1 || true)"
fi

if [[ -n "${existing_bridge_pid}" ]]; then
  echo "ROS 2 socket bridge is already running."
  echo "${existing_bridge_pid}" >"${LOG_DIR}/ros2_bridge.pid"
else
  echo "Starting ROS 2 socket bridge..."
  (
    unset ROS_VERSION ROS_PYTHON_VERSION ROS_PACKAGE_PATH AMENT_PREFIX_PATH
    unset CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    export ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=0
    export ISAAC_SOCKET_PORT="${ROS2_SOCKET_PORT}"
    exec python3 -u "${HOST_BRIDGE}"
  ) >"${LOG_DIR}/ros2_bridge.log" 2>&1 &
  echo "$!" >"${LOG_DIR}/ros2_bridge.pid"
  sleep 1
  if ! kill -0 "$(<"${LOG_DIR}/ros2_bridge.pid")" 2>/dev/null; then
    echo "ROS 2 socket bridge exited; see ${LOG_DIR}/ros2_bridge.log" >&2
    exit 1
  fi
fi

if ! container_ros "rosparam list" >/dev/null 2>&1; then
  echo "Starting roscore..."
  container_ros_detached \
    "exec roscore >'${LOG_DIR}/roscore.log' 2>&1"
fi
wait_for_container_ros "roscore" "rosparam list"

if ! container_ros "rosnode list | grep -qx /isaac_hw" >/dev/null 2>&1; then
  echo "Starting the ROS 1 socket client and Isaac hardware interface..."
  container_ros_detached \
    "export ROBOT_TYPE=go1; exec roslaunch isaac_legged_hw bringup_isaac.launch socket_host:='${ROS2_SOCKET_HOST}' socket_port:='${ROS2_SOCKET_PORT}' >'${LOG_DIR}/bringup.log' 2>&1"
fi
wait_for_container_ros "controller manager" \
  "rosservice list | grep -qx /controller_manager/list_controllers" 90

if ! container_ros \
  "rosservice call /controller_manager/list_controllers | grep -q controllers/legged_cheater_controller" \
  >/dev/null 2>&1; then
  echo "Loading the Isaac cheater controller..."
  container_ros_detached \
    "export ROBOT_TYPE=go1; exec roslaunch isaac_legged_hw control_isaac.launch cheater:=true start_gait_command:=false >'${LOG_DIR}/control.log' 2>&1"
fi
wait_for_container_ros "cheater controller" \
  "rosservice call /controller_manager/list_controllers | grep -q controllers/legged_cheater_controller" 90

echo "Switching the cheater controller to running..."
container_ros "rosservice call /controller_manager/switch_controller \"start_controllers: ['controllers/legged_cheater_controller']
stop_controllers: []
strictness: 2
start_asap: false
timeout: 0.0\"" >/dev/null

wait_for_container_ros "running controller" \
  "rosservice call /controller_manager/list_controllers | grep -A2 controllers/legged_cheater_controller | grep -q 'state:.*running'" 30

cat <<EOF

The ROS/control stack is ready. Logs are under ${LOG_DIR} on the host and in
the same path inside the container.

Next, while the Isaac timeline is stopped:
  1. Run: ${REPO_ROOT}/isaac_sim/run_isaac_sim.sh
  2. Open: ${REPO_ROOT}/isaac_sim/assets/go1_comp.usd
  3. Run isaac_sim/scripts/go1_isaac_bridge.py in Isaac's Script Editor.
  4. Verify the controller is still running, then press Play.
  5. Select a gait in a separate interactive ROS 1 container terminal.
  6. Publish /cmd_vel; always publish an explicit zero command to stop.
EOF
