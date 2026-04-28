#!/bin/bash
# =================================================================
# Script:      clean.sh
# Description: Cleans up build artifacts in ROS 2 workspace.
# Usage: ./scripts/clean.sh
# Warning:     This action is permanent and deletes all compiled
#              code and logs.
# =================================================================

# Navigate to workstation's base dire (assuming script is in ros2_ws/scripts)
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd "$SCRIPT_DIR/.."

# Remove all build artifacts
rm -rf build/ install/ log/