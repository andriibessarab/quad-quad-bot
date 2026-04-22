#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-fix}"

if [[ "${MODE}" != "fix" && "${MODE}" != "check" ]]; then
  echo "Usage: $0 [fix|check]"
  exit 2
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROOT_DIR="$(cd "${WS_DIR}/.." && pwd)"

cd "${ROOT_DIR}"

mapfile -t CPP_FILES < <(git ls-files 'ros2_ws/**/*.c' 'ros2_ws/**/*.cc' 'ros2_ws/**/*.cpp' 'ros2_ws/**/*.cxx' 'ros2_ws/**/*.h' 'ros2_ws/**/*.hh' 'ros2_ws/**/*.hpp' 'ros2_ws/**/*.hxx')
mapfile -t CMAKE_FILES < <(git ls-files 'ros2_ws/**/CMakeLists.txt' 'ros2_ws/**/*.cmake')

if [[ "${MODE}" == "fix" ]]; then
  echo "[quality] Python lint+fix"
  ruff check ros2_ws --config ros2_ws/pyproject.toml --fix
  echo "[quality] Python format"
  ruff format ros2_ws --config ros2_ws/pyproject.toml

  if (( ${#CPP_FILES[@]} > 0 )); then
    echo "[quality] C/C++ format"
    clang-format -i "${CPP_FILES[@]}"
  fi

  if (( ${#CMAKE_FILES[@]} > 0 )); then
    echo "[quality] CMake format"
    cmake-format -i "${CMAKE_FILES[@]}"
    echo "[quality] CMake lint"
    cmake-lint "${CMAKE_FILES[@]}"
  fi

  echo "[quality] Complete (fix mode)"
  exit 0
fi

echo "[quality] Python lint"
ruff check ros2_ws --config ros2_ws/pyproject.toml
echo "[quality] Python format check"
ruff format ros2_ws --config ros2_ws/pyproject.toml --check

if (( ${#CPP_FILES[@]} > 0 )); then
  echo "[quality] C/C++ format check"
  clang-format --dry-run --Werror "${CPP_FILES[@]}"
fi

if (( ${#CMAKE_FILES[@]} > 0 )); then
  echo "[quality] CMake format check"
  cmake-format --check "${CMAKE_FILES[@]}"
  echo "[quality] CMake lint"
  cmake-lint "${CMAKE_FILES[@]}"
fi

echo "[quality] Complete (check mode)"
