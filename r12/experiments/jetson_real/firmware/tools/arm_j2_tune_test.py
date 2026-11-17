#!/usr/bin/env python3
"""Supported-arm j2 hold, 2-degree rise and optional home trial with RAM P tuning."""
import argparse
import json
import signal
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros_ws/src/nz2_arm_bridge/scripts"))
from arm_v2_protocol import ArmSerial, ProtocolError, validate_pose, validate_fold
from arm_j2_diagnostics import read_diagnostics, summarize

CURRENT_LIMITS = (600, 1200, 900, 600, 400, 300)


def check(status, torque, stopped=False):
    allowed = (0, 2) if stopped else (0,)
    if not torque:
        allowed = (0, 2, 4)
    if (status["state"] not in allowed or status["p_state"] not in allowed
            or status["torque"] != torque or status["fault_code"]
            or status["hold_fail_mask"] or status["offline_mask"]
            or any(status["hardware_errors"])):
        raise ProtocolError("Abnormal status " + json.dumps(status))
    validate_pose(status["position_deg"])
    if any(abs(value) > limit for value, limit in zip(status["current_ma"], CURRENT_LIMITS)):
        raise ProtocolError("Host current stop threshold exceeded")


def load_sample(arm):
    started = time.monotonic()
    arm.send("W")
    deadline = started + 0.08
    rows = []
    values = {}
    while True:
        row = json.loads(arm.read_line(deadline))
        rows.append(row)
        if row.get("event") == "end":
            break
        if "item" in row:
            if not row.get("ok") or row.get("lib_err") or row.get("packet_err"):
                raise ProtocolError("Load read failed " + json.dumps(row))
            values[row["item"]] = row["raw"]
    expected = {"input_voltage", "present_current", "present_pwm", "temperature"}
    if values.keys() != expected:
        raise ProtocolError("Incomplete load response")
    current = (values["present_current"] + 32768) % 65536 - 32768
    pwm = (values["present_pwm"] + 32768) % 65536 - 32768
    arm.send("K")
    return dict(voltage_v=round(values["input_voltage"] * 0.1, 2),
                current_ma=round(current * 2.69, 2), pwm_raw=pwm,
                temperature_c=values["temperature"], raw=rows,
                query_monotonic=started, query_duration_s=time.monotonic()-started)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gain", type=int, choices=(800, 1000), required=True)
    parser.add_argument("--stage", choices=("hold", "rise", "home", "supported-off"), required=True)
    parser.add_argument("--port", default="", help="Optional port, verified by arm identity")
    parser.add_argument("--return-to-start", action="store_true",
                        help="Return to the recorded start pose after success, then retain torque")
    parser.add_argument("--support-confirmed", action="store_true",
                        help="Operator confirms the arm weight is physically supported for supported-off")
    parser.add_argument("--log", type=Path, required=True)
    args = parser.parse_args()
    if args.stage == "supported-off" and not args.support_confirmed:
        parser.error("supported-off requires --support-confirmed after physically supporting the arm")
    if args.stage == "supported-off" and args.return_to_start:
        parser.error("supported-off does not perform motion")

    def interrupted(signum, frame):
        raise KeyboardInterrupt(f"Signal {signum}")

    previous_sigterm = signal.signal(signal.SIGTERM, interrupted)
    arm = None
    attempted = False
    gain_touched = False
    previous_gain = None
    passed = False

    def record(event, value):
        row = dict(time=time.time(), event=event, value=value)
        with args.log.open("a") as stream:
            stream.write(json.dumps(row) + "\n")
        if event != "sample":
            print(json.dumps(row), flush=True)

    def phase(start, goal, name, hold_seconds=0.8):
        span = max(abs(b-a) for a, b in zip(start, goal))
        duration = max(1.0, 1.5 * span / 5.0)
        began = time.monotonic()
        last_load = 0.0
        while True:
            now = time.monotonic()
            elapsed = now - began
            u = min(elapsed / duration, 1.0)
            blend = u*u*(3-2*u)
            target = [a+(b-a)*blend for a, b in zip(start, goal)]
            status = arm.snapshot()
            check(status, True)
            validate_pose(target)
            validate_fold(status["position_deg"], target)
            arm.send("A " + " ".join(f"{value:.3f}" for value in target))
            sample = dict(phase=name, target_deg=target, status=status)
            if now-last_load >= 0.1:
                sample["load"] = load_sample(arm)
                last_load = now
                if abs(sample["load"]["current_ma"]) > CURRENT_LIMITS[1]:
                    record("sample", sample)
                    raise ProtocolError("j2 register current exceeded 1200mA")
            record("sample", sample)
            errors = [abs(p-t) for p, t in zip(status["position_deg"], target)]
            if max(errors) > 6.0:
                raise ProtocolError(f"{name} tracking error {max(errors):.3f} deg")
            if elapsed >= duration+hold_seconds:
                record(name, dict(error_deg=errors, status=status, elapsed_s=elapsed))
                arrival_limit = 1.0 if name == "rise" else 3.0
                if max(errors) > arrival_limit:
                    raise ProtocolError(f"{name} arrival error {max(errors):.3f} deg")
                return target
            time.sleep(0.025)

    try:
        arm = ArmSerial.discover(port=args.port, timeout=0.08, settle_sec=0.25)
        record("identity", arm.hello)
        if arm.hello.get("diag_rev") != 2:
            raise ProtocolError("Diagnostic revision 2 required")
        record("initialization", arm.identity())
        if args.stage == "supported-off":
            record("support_confirmed", True)
            record("stop", arm.query("S", "OK S", timeout=1.0))
            record("disable", arm.query("T 0", "OK T 0", timeout=1.0))
            check(arm.snapshot(), False)
            record("restore_gain", arm.query(f"J {args.gain}", "OK J", timeout=1.0))
            before, rows = read_diagnostics(arm)
            final = summarize(rows)
            record("final_settings", final)
            if (not final["complete"] or final["raw"]["torque_enable"] != 0
                    or final["raw"]["position_p_gain"] != args.gain):
                raise ProtocolError("Supported torque OFF / gain verification failed")
            passed = True
        else:
            before, rows = read_diagnostics(arm)
            check(before, False)
            settings = summarize(rows)
            record("settings", settings)
            if not settings["complete"] or settings["raw"].get("operating_mode") != 3:
                raise ProtocolError("Incomplete settings or unexpected operating mode")
            if "goal_pwm" not in settings["raw"]:
                raise ProtocolError("Actual goal PWM query required")
            previous_gain = settings["raw"]["position_p_gain"]
            if previous_gain != 800:
                raise ProtocolError("Expected baseline P gain 800")
            positions = []
            for _ in range(3):
                status = arm.snapshot()
                check(status, False)
                positions.append(status["position_deg"])
                time.sleep(0.1)
            if any(max(p[i] for p in positions)-min(p[i] for p in positions) > 0.35 for i in range(6)):
                raise ProtocolError("Position unstable before torque ON")
            start = positions[-1]
            rise = list(start)
            rise[1] -= 2.0
            home = [0.0]*5+[start[5]]
            validate_pose(rise)
            validate_pose(home)
            validate_fold(start, rise)
            validate_fold(rise, home)
            record("plan", dict(gain=args.gain, stage=args.stage,
                                start_deg=start, rise_deg=rise, home_deg=home))
            gain_touched = True
            record("gain", arm.query(f"J {args.gain}", "OK J", timeout=1.0))
            record("release", arm.query("R", "OK R", timeout=1.0))
            check(arm.snapshot(), False)
            attempted = True
            try:
                record("enable", arm.query("T 1", "OK T 1", timeout=1.0))
            finally:
                # Arm the firmware watchdog even if the torque-enable ACK was lost.
                arm.send("K")
            phase(start, start, "hold", hold_seconds=1.0)
            if args.stage in ("rise", "home"):
                phase(start, rise, "rise")
            if args.stage == "home":
                phase(rise, home, "home")
            if args.return_to_start:
                phase(arm.snapshot()["position_deg"], start, "return")
            passed = True
    except BaseException as error:
        record("error", repr(error))
    finally:
        if arm:
            try:
                if attempted:
                    # S and the firmware watchdog hold position with torque retained.
                    # A failed stop/USB query must never fall through to torque OFF.
                    record("stop", arm.query("S", "OK S", timeout=1.0))
                final_status = arm.snapshot()
                if final_status["torque"]:
                    check(final_status, True, stopped=True)
                    if gain_touched and args.gain != previous_gain:
                        record("gain_restore_pending", previous_gain)
                    record("torque_retained", True)
                else:
                    check(final_status, False)
                    if gain_touched:
                        record("restore_gain", arm.query(f"J {previous_gain}", "OK J", timeout=1.0))
                    before, rows = read_diagnostics(arm)
                    final = summarize(rows)
                    record("final_settings", final)
                    if (not final["complete"] or final["raw"]["torque_enable"] != 0
                            or (gain_touched and final["raw"]["position_p_gain"] != previous_gain)):
                        raise ProtocolError("Final torque / gain verification failed")
                    if attempted:
                        raise ProtocolError("Torque unexpectedly OFF after motion attempt")
                record("final_status", final_status)
            except BaseException as final_error:
                passed = False
                record("final_error", repr(final_error))
                record("torque_state", "unverified; keep physical support")
            finally:
                arm.close()
        signal.signal(signal.SIGTERM, previous_sigterm)
        record("result", dict(passed=passed, torque_attempted=attempted))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
