#!/usr/bin/env python3
"""
Zero-offset calibration tool for STS3215-12V servos.

For each joint the script:
  1. Disables torque so the joint can be moved freely
  2. The operator physically positions the joint at its URDF zero pose (see below)
  3. The operator presses Enter — the current tick value is recorded as offset_ticks
  4. Torque is re-enabled and the script moves to the next joint

URDF zero poses (what "angle = 0 rad" looks like physically):
  HAA: shoulder link horizontal, parallel to the body side panel
  HFE: femur hanging straight down from the shoulder
  KFE: tibia fully extended, aligned with the femur (straight leg)

Updated offset_ticks values are written directly back into motor_params.yaml.

Usage:
    python3 calibrate_offsets.py [--port /dev/ttyUSB0] [--joint fr_haa_joint]
                                 [--config ros2_ws/src/bot_hardware/config/motor_params.yaml]

If --joint is omitted, the script walks through all 12 joints in order.

Requires: pip install scservo_sdk pyyaml
"""

import argparse
import sys
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit("Missing dependency: pip install pyyaml")

try:
    from scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS
except ImportError:
    sys.exit(
        "Missing dependency: pip install scservo_sdk\n"
        "Or install from https://github.com/ftservo/FTServo_Linux"
    )

ZERO_POSES = {
    "haa": "shoulder link horizontal, parallel to the body side panel",
    "hfe": "femur hanging straight down from the shoulder",
    "kfe": "tibia fully extended, aligned with femur (straight leg)",
}


def read_position(handler, motor_id):
    pos, _, result, _ = handler.ReadPosSpeed(motor_id)
    if result != COMM_SUCCESS:
        raise IOError(f"Motor {motor_id}: read failed (comm_result={result})")
    return pos


def joint_type(joint_name):
    # e.g. "fr_haa_joint" -> "haa"
    return joint_name.split("_")[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    default_config = Path(__file__).parent.parent.parent / \
        "ros2_ws" / "src" / "bot_hardware" / "config" / "motor_params.yaml"
    parser.add_argument("--port", default=None,
                        help="Serial port (default: read from motor_params.yaml)")
    parser.add_argument("--joint", default=None,
                        help="Single joint to calibrate (default: all joints)")
    parser.add_argument("--config", default=str(default_config),
                        help=f"Path to motor_params.yaml (default: {default_config})")
    args = parser.parse_args()

    yaml_path = Path(args.config)
    if not yaml_path.exists():
        sys.exit(f"Cannot find motor_params.yaml at {yaml_path}")

    with open(yaml_path) as f:
        cfg = yaml.safe_load(f)

    port       = args.port or cfg["hardware"]["serial_port"]
    baud       = cfg["hardware"]["baud_rate"]
    joints_cfg = cfg["joints"]

    if args.joint:
        if args.joint not in joints_cfg:
            sys.exit(f"Unknown joint '{args.joint}'. Options: {list(joints_cfg)}")
        targets = {args.joint: joints_cfg[args.joint]}
    else:
        targets = joints_cfg

    port_handler  = PortHandler(port)
    servo_handler = sms_sts(port_handler)

    if not port_handler.openPort():
        sys.exit(f"Failed to open port: {port}")
    if not port_handler.setBaudRate(baud):
        port_handler.closePort()
        sys.exit(f"Failed to set baud rate: {baud}")

    print(f"\nConnected to {port} at {baud} baud")
    print("=" * 60)

    try:
        for joint_name, params in targets.items():
            motor_id  = params["motor_id"]
            jtype     = joint_type(joint_name)
            pose_hint = ZERO_POSES.get(jtype, "URDF zero position")

            print(f"\nJoint : {joint_name}  (motor ID {motor_id})")
            print(f"Target: {pose_hint}")
            servo_handler.EnableTorque(motor_id, 0)
            print("Torque disabled — move the joint to the zero pose.")

            input("Press Enter when in position...")

            ticks = read_position(servo_handler, motor_id)
            cfg["joints"][joint_name]["offset_ticks"] = int(ticks)
            print(f"Recorded offset_ticks = {ticks}")

            servo_handler.EnableTorque(motor_id, 1)

    finally:
        port_handler.closePort()

    with open(yaml_path, "w") as f:
        yaml.dump(cfg, f, default_flow_style=False, sort_keys=False)

    print(f"\nSaved updated offset_ticks to {yaml_path}")


if __name__ == "__main__":
    main()
