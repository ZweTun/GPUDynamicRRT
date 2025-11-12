#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}
WORKSPACE_SETUP=${WORKSPACE_SETUP:-/sim_ws/install/local_setup.bash}

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found at $ROS_SETUP" >&2
  exit 1
fi

if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace setup file not found at $WORKSPACE_SETUP" >&2
  exit 1
fi

# Track background job PIDs so we can clean them up if something fails.
declare -a pids=()

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if ((${#pids[@]} > 0)); then
    kill "${pids[@]}" >/dev/null 2>&1 || true
    wait "${pids[@]}" 2>/dev/null || true
  fi
  exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Run a command after sourcing ROS environments and redirecting output to the given log file.
run_with_env() {
  local cmd="$1"
  local log_file="$2"

  (
    set -euo pipefail
    source "$ROS_SETUP"
    source "$WORKSPACE_SETUP"
    eval "$cmd"
  ) &>"$log_file" &
  pids+=("$!")
}

echo "Launching ROS nodes; logs at /sim_ws/*.log"
run_with_env "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" "/sim_ws/f1tenth_gym.log"
run_with_env "python3 /sim_ws/src/lab7_pkg/scripts/pure_pursuit.py" "/sim_ws/pure_pursuit.log"
run_with_env "ros2 run lab7_pkg_cpp rrt_node --ros-args -p sim:=true" "/sim_ws/rrt_node.log"

for pid in "${pids[@]}"; do
  wait "$pid" || { exit_code=$?; exit "$exit_code"; }
done

pids=()
