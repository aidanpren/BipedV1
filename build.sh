#!/usr/bin/env bash
# Build the workspace. Always use this instead of a bare `colcon build`.
#
# --symlink-install LINKS source files into install/ instead of copying them,
# so after this runs once you can edit any .py and just relaunch — no rebuild.
# That matters most for balance_controller gain tuning.
#
# CAVEAT you WILL hit: symlinks only cover EXISTING files. Adding a new node,
# a new entry_point in setup.py, or a new data_file (launch/, config/) still
# needs a re-run of this script to regenerate the console_scripts wrappers.
# Symptom if you forget: "executable not found" or your new YAML not installed.
#
# NOTE: colcon does NOT read a workspace-local .colcon/defaults.yaml — it only
# reads ~/.colcon/defaults.yaml. Hence the explicit flag here rather than a
# config file that would silently do nothing.
#
# Pass extra args through, e.g.:  ./build.sh --packages-select robot_base
set -e

source /opt/ros/jazzy/setup.bash
cd "$(dirname "$0")"

echo "--- colcon build --symlink-install ---"
colcon build --symlink-install "$@"

echo
echo "Done. Now run:  source install/setup.bash"
