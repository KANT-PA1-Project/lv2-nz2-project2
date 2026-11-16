"""Wire protocol only; importing this module never opens serial ports."""
import glob
import json
import math
import re
import time

import serial

ORDER = ("j1", "j2", "j3", "j4", "roll", "grip")
MOTOR_IDS = (11, 12, 13, 14, 16, 15)
MIN_DEG = (-175.0, -95.0, -111.0, -99.0, -170.0, -134.0)
MAX_DEG = (175.0, 113.0, 82.0, 117.0, 170.0, -3.0)
FW = "nz2_arm v2 handshake1"
REV = 8


class ProtocolError(RuntimeError):
    pass


def validate_pose(values):
    if len(values) != 6:
        raise ProtocolError("Exactly six motor angles are required")
    for name, value, low, high in zip(ORDER, values, MIN_DEG, MAX_DEG):
        if not math.isfinite(value) or not low <= value <= high:
            raise ProtocolError(f"{name}={value} deg outside [{low}, {high}]")


def folded(pose):
    return pose[1] < -80.0 and pose[2] > 45.0


def validate_fold(current, target):
    # Reject goals the firmware would silently alter; this is not collision planning.
    if (folded(current) or folded(target)) and target[3] > 0.0:
        raise ProtocolError("Fold interlock: wrist target must be <= 0 deg")
    if not folded(current) and folded(target) and current[3] > 3.0:
        raise ProtocolError("Fold interlock: lower wrist before entering fold")


class ArmSerial:
    """One owner, sequential requests. No automatic reconnect or torque commands."""

    def __init__(self, connection, hello, timeout):
        self.connection = connection
        self.hello = hello
        self.timeout = timeout

    @classmethod
    def discover(cls, port="", timeout=0.08, settle_sec=2.0):
        candidates = [port] if port else sorted(glob.glob("/dev/ttyACM*"))
        found = []
        errors = []
        try:
            for candidate in candidates:
                connection = None
                accepted = False
                try:
                    connection = serial.Serial(
                        port=None, baudrate=115200, timeout=0.005,
                        write_timeout=0.08, exclusive=True)
                    connection.dtr = False
                    connection.rts = False
                    connection.port = candidate
                    connection.open()
                    time.sleep(settle_sec)
                    connection.reset_input_buffer()
                    probe = cls(connection, {}, max(timeout, 1.0))
                    # Until role is known, I is the ONLY transmitted command.
                    probe.send("I")
                    deadline = time.monotonic() + probe.timeout
                    while time.monotonic() < deadline:
                        line = probe.read_line(deadline)
                        try:
                            hello = json.loads(line)
                        except (ValueError, UnicodeError):
                            continue
                        if isinstance(hello, dict) and hello.get("board") == "arm":
                            found.append(cls(connection, hello, timeout))
                            accepted = True
                            break
                except (OSError, serial.SerialException, ProtocolError) as exc:
                    errors.append(f"{candidate}: {exc}")
                finally:
                    if connection is not None and not accepted:
                        connection.close()
            if len(found) != 1:
                raise ProtocolError(
                    f"Expected one arm, found {len(found)}; " + "; ".join(errors))
            arm = found[0]
            hello = arm.hello
            if (hello.get("hello") != 1 or hello.get("proto") != 1
                    or hello.get("kind") != "arm" or hello.get("fw") != FW
                    or hello.get("fw_version") != "2.0"
                    or hello.get("limits_rev") != REV
                    or hello.get("order") != list(ORDER)):
                raise ProtocolError(f"Unsupported arm firmware: {hello}")
            return arm
        except BaseException:
            for arm in found:
                arm.close()
            raise

    def send(self, command):
        if "\n" in command or "\r" in command:
            raise ProtocolError("One command per line required")
        wire = (command + "\n").encode("ascii")
        if len(wire) > 95:
            raise ProtocolError("Command exceeds firmware line buffer")
        if self.connection.write(wire) != len(wire):
            raise ProtocolError("Short serial write")

    def read_line(self, deadline):
        data = bytearray()
        while time.monotonic() < deadline:
            char = self.connection.read(1)
            if not char:
                continue
            if char == b"\n":
                return data.decode("ascii").strip()
            data.extend(char)
            if len(data) > 512:
                raise ProtocolError("Oversized serial response")
        raise ProtocolError("Serial response timeout")

    def query(self, command, prefix, timeout=None):
        self.send(command)
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            line = self.read_line(deadline)
            if line.startswith("ERR "):
                raise ProtocolError(line)
            if line.startswith(prefix):
                return line
        raise ProtocolError(f"No response for {command}")

    def snapshot(self):
        # Never use V: the original v2 + I does not implement it.
        p = self.query("Q", "P ").split()
        d = self.query("D", "D ").split()
        e = self.query("E", "E ").split()
        if len(p) != 14 or len(d) != 11 or len(e) != 7:
            raise ProtocolError("Malformed Q/D/E response length")
        angles = [float(v) for v in p[2:8]]
        current = [float(v) for v in p[8:14]]
        diag = [int(v) for v in d[1:]]
        errors = [int(v) for v in e[1:]]
        p_state = int(p[1])
        if (p_state not in range(5) or diag[0] not in range(5)
                or diag[1] not in range(4) or diag[5] not in (0, 1)
                or diag[8] != REV
                or any(not 0 <= value <= 255 for value in errors)
                or any(not math.isfinite(value) for value in current)):
            raise ProtocolError("Invalid Q/D/E values or limits revision")
        return {
            "p_state": p_state, "state": diag[0], "fault_code": diag[1],
            "fault_motor": diag[2], "hold_fail_mask": diag[3],
            "offline_mask": diag[4], "torque": bool(diag[5]),
            "cmd_age_ms": diag[6], "bad_lines": diag[7],
            "limits_rev": diag[8], "estop_ms": diag[9],
            "position_deg": angles, "current_ma": current,
            "hardware_errors": errors, "received_monotonic": time.monotonic(),
        }

    def identity(self):
        line = self.query("?", "NZ2_ARM ", timeout=1.0)
        if not re.fullmatch(
                r"NZ2_ARM fw=2\.0 limits_rev=8 ok=6/6 fail=-", line):
            raise ProtocolError(f"Arm not fully initialized: {line}")
        return line

    def close(self):
        # Deliberately no S/T0 here: no new stop profile or sudden torque release.
        self.connection.close()
