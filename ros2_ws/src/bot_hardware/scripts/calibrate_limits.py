#!/usr/bin/env python3
"""
Joint limit calibration tool for STS3215-12V servos.

For each joint the script:
  1. Disables torque so the joint can be moved freely by hand
  2. Waits for the operator to move the joint to each end stop and press Enter
  3. Records the tick value at each stop
  4. Prints a joint_limits.yaml snippet with a 5-degree safety margin applied

Usage:
    python3 calibrate_limits.py [--port /dev/ttyUSB0] [--joint fr_haa_joint]

If --joint is omitted, the script walks through all 12 joints in order.

Requires: pip install scservo_sdk pyyaml
"""

import argparse
import math
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

SAFETY_MARGIN_RAD = math.radians(5)
TICKS_PER_REV = 4096


def ticks_to_rad(ticks, offset_ticks, direction):
    return direction * (ticks - offset_ticks) / TICKS_PER_REV * (2 * math.pi)


def read_position(handler, motor_id):
    pos, _, result, _ = handler.ReadPosSpeed(motor_id)
    if result != COMM_SUCCESS:
        raise IOError(f"Motor {motor_id}: read failed (comm_result={result})")
    return pos


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default=None,
                        help="Serial port (default: read from motor_params.yaml)")
    parser.add_argument("--joint", default=None,
                        help="Single joint name to calibrate (default: all joints)")
    args = parser.parse_args()

    yaml_path = Path(__file__).parent.parent / "config" / "motor_params.yaml"
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

    results = {}

    try:
        for joint_name, params in targets.items():
            motor_id     = params["motor_id"]
            direction    = params["direction"]
            offset_ticks = params["offset_ticks"]

            print(f"\nJoint: {joint_name}  (motor ID {motor_id})")
            servo_handler.EnableTorque(motor_id, 0)
            print("  Torque disabled — the joint can now be moved freely.")

            input("  Move to MINIMUM (most negative) position, then press Enter...")
            min_ticks = read_position(servo_handler, motor_id)
            min_rad   = ticks_to_rad(min_ticks, offset_ticks, direction)
            print(f"  Min: {min_ticks} ticks  →  {math.degrees(min_rad):.1f}°")

            input("  Move to MAXIMUM (most positive) position, then press Enter...")
            max_ticks = read_position(servo_handler, motor_id)
            max_rad   = ticks_to_rad(max_ticks, offset_ticks, direction)
            print(f"  Max: {max_ticks} ticks  →  {math.degrees(max_rad):.1f}°")

            servo_handler.EnableTorque(motor_id, 1)

            lower_raw = min(min_rad, max_rad)
            upper_raw = max(min_rad, max_rad)
            lower = lower_raw + SAFETY_MARGIN_RAD
            upper = upper_raw - SAFETY_MARGIN_RAD

            if lower >= upper:
                print(f"  WARNING: safety margin left no valid range — skipping {joint_name}")
                continue

            results[joint_name] = {"lower": round(lower, 4), "upper": round(upper, 4)}
            print(f"  → limits (with {math.degrees(SAFETY_MARGIN_RAD):.0f}° margin): "
                  f"[{math.degrees(lower):.1f}°, {math.degrees(upper):.1f}°]")

    finally:
        port_handler.closePort()

    if not results:
        print("\nNo results to write.")
        return

    print("\n" + "=" * 60)
    print("Copy the values below into:")
    print("  ros2_ws/src/bot_description/config/joint_limits.yaml")
    print("=" * 60 + "\n")

    # Aggregate per joint type — tightest range across all limbs of that type.
    types = {}
    for joint_name, lims in results.items():
        jtype = joint_name.split("_")[1]  # e.g. "fr_haa_joint" -> "haa"
        if jtype not in types:
            types[jtype] = {"lower": lims["lower"], "upper": lims["upper"]}
        else:
            types[jtype]["lower"] = max(types[jtype]["lower"], lims["lower"])
            types[jtype]["upper"] = min(types[jtype]["upper"], lims["upper"])

    for jtype, lims in types.items():
        print(f"{jtype}:")
        print(f"  lower: {lims['lower']}")
        print(f"  upper: {lims['upper']}")
        print(f"  effort: 1000.0")
        print(f"  velocity: 5.0")
        print()


if __name__ == "__main__":
    main()
