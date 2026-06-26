# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a ROS 2 workspace for a quadruped robot ("quad-quad-bot") that also aims to support a flight mode (quadcopter). The current focus is simulation via Gazebo Harmonic. Real hardware mode is not yet implemented.

## Common Commands

All commands assume the workspace is sourced:
```bash
source install/setup.bash
```

**Build:**
```bash
colcon build
colcon build --packages-select <package_name>  # build a single package
```

**Launch simulation:**
```bash
ros2 launch bot_bringup robot.launch.py run_mode:=sim
# or use the convenience script from the repo root:
src/bot_bringup/scripts/launch_sim
```

**View robot in RViz only:**
```bash
ros2 launch bot_description view_robot.launch.py
```

**Lint and format (auto-fix):**
```bash
scripts/quality.sh        # fix mode (default)
scripts/quality.sh check  # CI/check mode — no file changes
```

First-time quality tool setup:
```bash
pip install ruff cmakelang
sudo apt-get install -y clang-format
```

## Architecture

### Package Structure

| Package           | Language      | Role                                                                                      |
|-------------------|---------------|-------------------------------------------------------------------------------------------|
| `bot_bringup`     | Python        | Top-level launch entrypoint; selects sim vs. real and wires all other packages together   |
| `bot_control`     | C++           | Locomotion nodes: gait planning → IK → joint commands                                     |
| `bot_description` | Xacro/YAML    | URDF robot model, meshes, joint limits                                                    |
| `bot_sim`         | Python        | Gazebo Harmonic launch and robot spawning                                                 |
| `bot_math`        | C++ (library) | Pure math: IK solver and limb trajectory computation — no ROS node, only a shared library |

### Data Flow

```
gait_planner node (LimbTrajectoryGenerator)
  │  publishes geometry_msgs/Point to  <prefix>/target  (per limb)
  ▼
locomotion_commander node (LocomotionCommander)
  │  subscribes to all  <prefix>/target  topics
  │  runs bot_math::IkSolver to convert Cartesian → joint angles
  │  publishes std_msgs/Float64MultiArray to
  ▼
joint_group_position_controller/commands
  │  (ros2_control JointGroupPositionController)
  ▼
Gazebo joints / real hardware
```

The `gait_planner` node computes per-limb Cartesian foot targets using `bot_math::LimbTrajectory` (cubic Bézier swing, linear stance). The `locomotion_commander` converts those targets to joint angles via analytical IK and streams them to the `ros2_control` position controller at 100 Hz.

### Joint Naming Convention

Joint names are formed by concatenating limb prefix + joint suffix:
- Prefixes: `fr`, `fl`, `br`, `bl` (front-right, front-left, back-right, back-left)
- Suffixes: `haa_joint`, `hfe_joint`, `kfe_joint` (hip abduction/adduction, hip flexion/extension, knee flexion/extension)
- Example: `fr_haa_joint`

**Order matters**: `controllers.yaml` and `locomotion_params.yaml` must list joints in the same order (fr→fl→br→bl, haa→hfe→kfe within each limb). The commander packs all angles into a single `Float64MultiArray` in this order.

### Limb Inversion

Because the four limbs are physically mirrored, `LocomotionCommander` applies sign inversions before sending to the IK solver and before packing joint commands:
- Right-side limbs (`is_right: true`): HAA output is negated
- Back limbs (`is_back: true`): X coordinate is negated before IK

These inversions are configured in `bot_control/config/locomotion_params.yaml` under `locomotion_commander.ros__parameters.limb_info`.

### URDF / Xacro Layout

`robot.urdf.xacro` is the root; it includes and composes:
- `body.urdf.xacro` — chassis link
- `limb.urdf.xacro` — `<xacro:limb>` macro instantiated 4× (femur/tibia links currently commented out — WIP)
- `ros2_control.urdf.xacro` — `<ros2_control>` tag, selects `gz_ros2_control/GazeboSimSystem` for sim or a configurable hardware plugin
- `gazebo.urdf.xacro` — Gazebo-specific plugins

Joint limits are loaded at xacro processing time from `bot_description/config/joint_limits.yaml`.

The `solidworks_export_urdf/` directory contains raw SolidWorks-exported URDF references; these are not used directly by the ROS stack — values are manually extracted and reformulated into the hand-authored xacro files.

### `bot_math` Library

`bot_math` is a pure C++ shared library (no executable). Its two main classes:
- `IkSolver` — analytical 3-DOF IK for a single limb given `LimbDimensions` (l1 hip offset, l2 femur, l3 tibia)
- `LimbTrajectory` — per-limb trajectory with configurable `phase_offset` for gait coordination; uses cubic Bézier for swing and linear interpolation for stance

All shared types (`LimbDimensions`, `LimbJointAngles`, `GaitMode`, `LimbTrajectoryConfig`, `LimbTrajectoryInput`) live in `bot_math/include/bot_math/types.hpp`.

## Code Style

- Python: `ruff` (config at `ros2_ws/pyproject.toml`)
- C++: `clang-format` (`.clang-format` at workspace root if present)
- CMake: `cmake-format` + `cmake-lint`

Run `scripts/quality.sh` before committing. CI runs `scripts/quality.sh check`.
