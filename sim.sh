#!/usr/bin/env bash
# One-command sim: clear stale processes, then launch the whole stack.
# Run from a NATIVE terminal (not VS Code's) — Gazebo's Qt GUI needs it.
#
# The preflight exists because leftover processes from a previous run are the
# single most common failure here, and they fail CONFUSINGLY:
#   - orphaned `gz sim` server  -> duplicate /controller_manager, spawners fail
#   - leftover rosbridge        -> "Address already in use" on :9090
#   - leftover http.server      -> "Address already in use" on :8000
#   - leftover mode_manager     -> stale latched /mode fed to new subscribers
# NOTE: deliberately no `set -u` here. ROS's setup.bash references unset vars
# (AMENT_TRACE_SETUP_FILES) and aborts the script under nounset.

echo "--- preflight: clearing stale processes ---"
for pat in 'gz [s]im' 'rosbridge_websocket' 'http\.server' \
           'lib/robot_base/balance_controller' 'lib/robot_teleop/mode_manager'; do
  if pgrep -f "$pat" > /dev/null 2>&1; then
    echo "  killing: $pat"
    pkill -9 -f "$pat" || true
  fi
done
sleep 1

echo "--- launching sim stack ---"
source /opt/ros/jazzy/setup.bash
source ~/Documents/ros2_ws/BipedV1/install/setup.bash
exec ros2 launch robot_bringup sim.launch.py
