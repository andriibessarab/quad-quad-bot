# Packages

| Package           | Description                                                                                                                               |
|-------------------|-------------------------------------------------------------------------------------------------------------------------------------------|
| `bot_bringup`     | Top-level launch entry point for both sim and real hardware.                                                                              |
| `bot_control`     | Gait planner and locomotion commander nodes - computes foot targets and runs IK to produce joint commands.                                |
| `bot_description` | URDF/xacro robot model, joint limits, motor wiring config, and ros2_control hardware description.                                         |
| `bot_hardware`    | ros2_control `SystemInterface` plugin that drives all 12 STS servos over TTL-level, half-duplex, single-wire asynchronous serial  serial. |
| `bot_math`        | Pure C++ library — IK solver and foot trajectory math.                                                                                    |
| `bot_sim`         | Gazebo Harmonic world and robot spawn setup.                                                                                              |
| `bot_teleop`      | Xbox controller → `/cmd_vel` via `joy_linux` and `teleop_twist_joy`.                                                                      |

