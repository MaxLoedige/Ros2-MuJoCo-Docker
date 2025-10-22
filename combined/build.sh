#!/usr/bin/env bash
set -e  # Exit immediately if a command fails
set -u  # Treat unset variables as errors
set -o pipefail  # Catch errors in pipelines

# Optional: make output more readable
echo "=== Starting build process for Mujoco Plugin ==="

# Navigate to ROS workspace
cd "$(dirname "$0")/ros"  # Go to 'ros' directory relative to script location

echo "--- Building ROS workspace ---"
colcon build

# Ensure target directory exists
PLUGIN_DIR="/root/workdir/ros/install/mujoco_ros2_control/lib/mujoco_ros2_control/mujoco_plugin"
mkdir -p "$PLUGIN_DIR"

# Build the Mujoco plugin
echo "--- Building Mujoco plugin ---"
cd ../plugin
cmake -Bbuild
cmake --build build -j"$(nproc)"

# Copy the compiled shared object to the install directory
echo "--- Copying plugin library ---"
cp build/libMJVRSimPlugin.so "$PLUGIN_DIR/libMJVRSimPlugin.so"

echo "=== Build complete! Plugin installed at: $PLUGIN_DIR ==="
