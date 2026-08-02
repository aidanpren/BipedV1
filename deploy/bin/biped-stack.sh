#!/usr/bin/env bash
# Source the ROS 2 environment, then BECOME `ros2 launch`.
#
# This wrapper exists because systemd's ExecStart= is not a shell: it execs a
# binary directly, so there is no way to `source` anything from the unit file.
# Everything a ROS node needs — AMENT_PREFIX_PATH, PYTHONPATH, the library
# path, the workspace overlay — is set up by shell scripts, so a shell has to
# run somewhere. It runs here, and then gets out of the way via `exec`.

set -euo pipefail

[ -r /etc/default/biped ] && . /etc/default/biped

: "${BIPED_WS:?BIPED_WS is not set. Is /etc/default/biped installed?}"
ros_setup="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"
overlay="${BIPED_WS}/install/setup.bash"
pkg="${BIPED_LAUNCH_PKG:-robot_bringup}"
launch_file="${BIPED_LAUNCH_FILE:-real.launch.py}"

if [ ! -r "${ros_setup}" ]; then
    echo "ERROR: ROS underlay not found at ${ros_setup}" >&2
    exit 1
fi
if [ ! -r "${overlay}" ]; then
    echo "ERROR: workspace overlay not found at ${overlay}" >&2
    echo "The workspace has not been built on this machine. Run:" >&2
    echo "  cd ${BIPED_WS} && colcon build --symlink-install" >&2
    exit 1
fi

# ---------------------------------------------------------------------------
# `set +u` around the sourcing is NOT sloppiness — it is required.
#
# ROS's setup.bash (and the ament/colcon scripts it chains into) read variables
# that are legitimately unset on a fresh boot: AMENT_TRACE_SETUP_FILES,
# COLCON_TRACE, AMENT_CURRENT_PREFIX and friends. Under `set -u` the very first
# one aborts the script, and because the abort happens INSIDE a sourced file the
# error message names a path in /opt/ros rather than anything of ours — which
# reads exactly like a broken ROS install.
# ---------------------------------------------------------------------------
set +u
# shellcheck disable=SC1090
. "${ros_setup}"
# shellcheck disable=SC1091
. "${overlay}"
set -u

cd "${BIPED_WS}"

# BIPED_LAUNCH_ARGS is intentionally UNQUOTED: it holds zero or more
# space-separated `key:=value` pairs and must word-split into separate argv
# entries. Quoting it would pass "" as a single empty argument, which ros2
# launch rejects.
# shellcheck disable=SC2086
exec ros2 launch "${pkg}" "${launch_file}" ${BIPED_LAUNCH_ARGS:-}
