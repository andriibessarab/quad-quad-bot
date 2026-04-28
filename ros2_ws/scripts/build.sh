#!/bin/bash
# =================================================================
# Script: build.sh
# Description: Compiles the ROS 2 workspace with symlinks and
#              debugging symbols enabled.
# Usage: ./scripts/build.sh
# =================================================================

# Navigate to workstation's base dire (assuming script is in ros2_ws/scripts)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR/.."

# Build project
colcon build \
    --symlink-install \
    --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON' \
    -Wall -Wextra -Wpedantic
