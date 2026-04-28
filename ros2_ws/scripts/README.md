# Workspace Scripts

This directory contains utility scripts to streamline the development, compilation, and quality assurance of the ROS 2 workspace.

## 1. build.sh
Compiles the workspace using colcon.
- Features: Enables --symlink-install for faster Python/Config updates and compiles with RelWithDebInfo for a balance of speed and debuggability.
- Usage: ./scripts/build.sh
- Note: Always remember to source install/setup.bash after building a new package.

---

## 2. clean.sh
The _nuclear option_ for the workspace.
- Features: Removes the build/, install/, and log/ directories to ensure a fresh state.
- Usage: ./scripts/clean.sh

---

## 3. quality.sh
Maintains code standards through linting and formatting.
- Features: Processes Python (Ruff), C++ (Clang-Format), and CMake files.
- Usage:
    - Auto-Fix (Default): Corrects formatting and simple lint errors.
      ./scripts/quality.sh fix
    - Check Mode: Only reports issues without modifying files (used for CI/CD).
      ./scripts/quality.sh check

---
