#!/usr/bin/env bash
# =================================================================
# Script: quality.sh
# Description: Automates linting and formatting for Python, C++,
#              and CMake files within the ROS 2 workspace.
# Usage: ./scripts/quality.sh [fix|check]
#        - fix (default): Automatically corrects style issues.
#        - check: Reports issues without modifying files (CI mode).
# =================================================================

# Exit on error (-e), treat unset variables as errors (-u),
# and ensure piped commands propagate exit codes (-o pipefail).
set -euo pipefail

# Set the mode: Use the first argument if provided, otherwise default to "fix"
MODE="${1:-fix}"

# Validation: Only allow "fix" or "check" as arguments
if [[ "${MODE}" != "fix" && "${MODE}" != "check" ]]; then
  echo "Usage: $0 [fix|check]"
  exit 2
fi

# Locate directories relative to the script location
# SCRIPT_DIR: Where this script lives (e.g., scripts/)
# WS_DIR: The ROS 2 workspace (e.g., ros2_ws/)
# ROOT_DIR: The folder containing the workspace
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROOT_DIR="$(cd "${WS_DIR}/.." && pwd)"

# Move to the root directory so 'git ls-files' works correctly
cd "${ROOT_DIR}"

# Use Git to find all relevant source files. This is faster than 'find'
# and automatically ignores files in .gitignore (like build/ and install/).
mapfile -t CPP_FILES < <(git ls-files 'ros2_ws/**/*.c' 'ros2_ws/**/*.cc' 'ros2_ws/**/*.cpp' 'ros2_ws/**/*.cxx' 'ros2_ws/**/*.h' 'ros2_ws/**/*.hh' 'ros2_ws/**/*.hpp' 'ros2_ws/**/*.hxx')
mapfile -t CMAKE_FILES < <(git ls-files 'ros2_ws/**/CMakeLists.txt' 'ros2_ws/**/*.cmake')

# --- FIX MODE: Automatically modify files to match style guides ---
if [[ "${MODE}" == "fix" ]]; then
  echo "[quality] Python lint+fix"
  ruff check ros2_ws --config ros2_ws/pyproject.toml --fix

  echo "[quality] Python format"
  ruff format ros2_ws --config ros2_ws/pyproject.toml

  # If C++ files exist, format them in-place (-i)
  if (( ${#CPP_FILES[@]} > 0 )); then
    echo "[quality] C/C++ format"
    clang-format -i "${CPP_FILES[@]}"
  fi

  # If CMake files exist, format and check for logical errors
  if (( ${#CMAKE_FILES[@]} > 0 )); then
    echo "[quality] CMake format"
    cmake-format -i "${CMAKE_FILES[@]}"
    echo "[quality] CMake lint"
    cmake-lint "${CMAKE_FILES[@]}"
  fi

  echo "[quality] Complete (fix mode)"
  exit 0
fi

# --- CHECK MODE: Report errors without changing any files ---
# (Usually used in CI/CD pipelines like GitHub Actions)
echo "[quality] Python lint"
ruff check ros2_ws --config ros2_ws/pyproject.toml

echo "[quality] Python format check"
ruff format ros2_ws --config ros2_ws/pyproject.toml --check

if (( ${#CPP_FILES[@]} > 0 )); then
  echo "[quality] C/C++ format check"
  # --dry-run prints changes to terminal; --Werror makes it fail if not perfect
  clang-format --dry-run --Werror "${CPP_FILES[@]}"
fi

if (( ${#CMAKE_FILES[@]} > 0 )); then
  echo "[quality] CMake format check"
  cmake-format --check "${CMAKE_FILES[@]}"
  echo "[quality] CMake lint"
  cmake-lint "${CMAKE_FILES[@]}"
fi

echo "[quality] Complete (check mode)"