#!/usr/bin/env python3
"""Read j2 diagnostics from the v2 firmware with diag_rev 1 or 2."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
PROTOCOL_DIR = ROOT / "ros_ws/src/nz2_arm_bridge/scripts"
sys.path.insert(0, str(PROTOCOL_DIR))
from arm_v2_protocol import ArmSerial, ProtocolError


def read_diagnostics(arm):
    if arm.hello.get("diag_rev") not in (1, 2):
        raise ProtocolError("Board requires the nz2_arm_v2_j2diag firmware")
    before = arm.snapshot()
    if before["torque"]:
        raise ProtocolError("Full settings query requires torque OFF")
    arm.send("V")
    deadline = time.monotonic() + 3.0
    rows = []
    while time.monotonic() < deadline:
        line = arm.read_line(deadline)
        if line.startswith("ERR "):
            raise ProtocolError(line)
        try:
            row = json.loads(line)
        except ValueError:
            continue
        if not isinstance(row, dict) or row.get("j2_diag") != 1:
            continue
        rows.append(row)
        if row.get("event") == "end":
            return before, rows
    raise ProtocolError("Incomplete j2 diagnostic response")


def summarize(rows):
    items = {row["item"]: row for row in rows if "item" in row}
    values = {name: row["raw"] for name, row in items.items() if row.get("ok")}
    result = {"raw": values, "read_errors": {
        name: row for name, row in items.items() if not row.get("ok")
    }}
    # Scaling matches the XM430 configuration used by this firmware.
    if "input_voltage" in values:
        result["input_voltage_v"] = round(values["input_voltage"] * 0.1, 2)
    if "current_limit" in values:
        result["current_limit_ma"] = round(values["current_limit"] * 2.69, 1)
    if "present_current" in values:
        signed = (values["present_current"] + 32768) % 65536 - 32768
        result["present_current_ma"] = round(signed * 2.69, 1)
    if "present_pwm" in values:
        result["present_pwm_signed_raw"] = (values["present_pwm"] + 32768) % 65536 - 32768
    expected = {
        "model_number", "operating_mode", "current_limit", "pwm_limit",
        "position_p_gain", "position_i_gain", "position_d_gain",
        "profile_velocity", "profile_acceleration", "torque_enable",
        "input_voltage", "present_current", "present_pwm", "temperature", "hardware_error"
    }
    result["missing_items"] = sorted(expected - items.keys())
    result["complete"] = not result["missing_items"] and not result["read_errors"]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="", help="Optional ttyACM port, still verified by identity")
    args = parser.parse_args()
    arm = ArmSerial.discover(port=args.port, timeout=0.12, settle_sec=0.25)
    try:
        print("IDENTITY", json.dumps(arm.hello))
        before, rows = read_diagnostics(arm)
        for row in rows:
            print(json.dumps(row))
        result = summarize(rows)
        result["before"] = before
        result["after"] = arm.snapshot()
        print("SUMMARY", json.dumps(result, indent=2))
        return 0 if result["complete"] else 2
    finally:
        arm.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ProtocolError) as error:
        print("Diagnostic failed:", error, file=sys.stderr)
        raise SystemExit(1)
