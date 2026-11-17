#!/usr/bin/env python3
"""Joint-space ROS bridge; no IK, model inference or automatic torque enable."""
import json
import math
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger

from arm_v2_protocol import (
    ArmSerial, FW, ORDER, ProtocolError, validate_fold, validate_pose,
)


class ArmBridge(Node):
    def __init__(self):
        super().__init__("arm_bridge")
        self.arm = None
        self.armed = False
        self.target = None
        self.sent = None
        self.kind = None
        self.current_ma = 150.0
        self.last_input = 0.0
        self.last_send = 0.0
        self.last_poll = 0.0
        self.last_status = 0.0
        self.snapshot_data = None
        self.error = ""
        self.phase = "connecting"

        def parameter(name, default):
            return self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value

        port = str(parameter("port", ""))
        self.motion_allowed = bool(parameter("allow_motion", False))
        self.names = list(parameter("joint_names", list(ORDER)))
        self.command_timeout = float(parameter("command_timeout_sec", 0.25))
        self.telemetry_timeout = float(parameter("telemetry_timeout_sec", 0.5))
        self.tx_hz = float(parameter("command_rate_hz", 20.0))
        self.poll_hz = float(parameter("poll_rate_hz", 5.0))
        self.speed = float(parameter("max_speed_deg_s", 10.0))
        serial_timeout = float(parameter("serial_timeout_sec", 0.08))
        settle = float(parameter("serial_settle_sec", 2.0))
        if (len(self.names) != 6 or len(set(self.names)) != 6
                or any(not isinstance(name, str) or not name for name in self.names)):
            raise ValueError("joint_names must contain six unique names in motor order")
        numeric = (self.command_timeout, self.telemetry_timeout, self.tx_hz,
                   self.poll_hz, self.speed, serial_timeout, settle)
        if (not all(math.isfinite(v) for v in numeric)
                or not 0.05 <= self.command_timeout <= 0.5
                or not 0.2 <= self.telemetry_timeout <= 1.0
                or not 10.0 <= self.tx_hz <= 30.0
                or not 1.0 <= self.poll_hz <= 10.0
                or self.telemetry_timeout < 1.5 / self.poll_hz
                or not 0.1 <= self.speed <= 30.0
                or not 0.01 <= serial_timeout <= 0.1
                or not 0.0 <= settle <= 5.0):
            raise ValueError("Invalid rate, timeout or speed parameters")

        # No historical command replay; keep only the most recent live sample.
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        self.joint_pub = self.create_publisher(JointState, "/arm/joint_states", 1)
        self.status_pub = self.create_publisher(String, "/arm/status", 1)
        self.create_subscription(
            JointState, "/arm/joint_command", self.on_joint_command, command_qos)
        self.create_subscription(
            Float64MultiArray, "/arm/gripper_command",
            self.on_gripper_command, command_qos)
        self.create_service(SetBool, "/arm/set_enabled", self.set_enabled)
        self.create_service(Trigger, "/arm/stop", self.stop)
        try:
            self.arm = ArmSerial.discover(port, serial_timeout, settle)
            self.refresh()
            self.phase = "read_only" if not self.motion_allowed else "disabled"
            self.get_logger().info(
                f"{self.arm.connection.port}: {FW}; allow_motion={self.motion_allowed}")
        except Exception as exc:
            if self.arm is not None:
                self.arm.close()
                self.arm = None
            self.phase = "disconnected"
            self.error = str(exc)
            self.get_logger().error(self.error)
        self.timer = self.create_timer(0.01, self.tick)

    def halt_stream(self, reason):
        # Stop A/G/K. Original firmware watchdog holds after 150 ms.
        # Do NOT automatically send S (profile change) or T0 (arm can fall).
        self.armed = False
        self.target = None
        self.kind = None
        self.phase = "halted"
        self.error = reason
        self.get_logger().error(reason)

    def healthy(self, snapshot, allowed_states):
        if (snapshot["state"] not in allowed_states
                or snapshot["p_state"] not in allowed_states
                or snapshot["fault_code"] or snapshot["hold_fail_mask"]
                or snapshot["offline_mask"] or any(snapshot["hardware_errors"])):
            raise ProtocolError("Firmware state/fault/hold/offline/error check failed")
        validate_pose(snapshot["position_deg"])

    def refresh(self):
        snapshot = self.arm.snapshot()
        self.snapshot_data = snapshot
        self.last_poll = time.monotonic()
        if all(math.isfinite(v) for v in snapshot["position_deg"]):
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self.names
            message.position = [math.radians(v) for v in snapshot["position_deg"]]
            # Q currents are mA, NOT torque in Nm. Never put them in effort.
            self.joint_pub.publish(message)
        return snapshot

    def require_stream(self):
        if self.arm is None or not self.motion_allowed or not self.armed:
            raise ProtocolError("Motion disabled; enable explicitly first")
        snapshot = self.snapshot_data
        if (snapshot is None or time.monotonic() - snapshot["received_monotonic"]
                > self.telemetry_timeout):
            raise ProtocolError("Telemetry stale")
        self.healthy(snapshot, (0,))
        if not snapshot["torque"]:
            raise ProtocolError("Firmware reports torque OFF")
        return snapshot

    def accept_target(self, target, kind, current_ma=None):
        snapshot = self.require_stream()
        validate_pose(target)
        validate_fold(snapshot["position_deg"], target)
        if current_ma is not None:
            if not math.isfinite(current_ma) or not 0.0 <= current_ma <= 300.0:
                raise ProtocolError("Gripper current must be 0..300 mA")
            self.current_ma = current_ma
        self.target = target
        self.kind = kind
        self.last_input = time.monotonic()
        self.phase = "streaming"
        self.error = ""

    def on_joint_command(self, message):
        try:
            if (len(message.name) != 6 or len(set(message.name)) != 6
                    or set(message.name) != set(self.names)
                    or len(message.position) != 6
                    or message.velocity or message.effort):
                raise ProtocolError("Provide six named positions in rad; velocity/effort empty")
            stamp = message.header.stamp
            if stamp.sec or stamp.nanosec:
                age = (self.get_clock().now().nanoseconds
                       - (stamp.sec * 1000000000 + stamp.nanosec)) / 1e9
                if age < -0.05 or age > self.command_timeout:
                    raise ProtocolError("Joint command timestamp stale or in the future")
            by_name = dict(zip(message.name, message.position))
            target = [math.degrees(by_name[name]) for name in self.names]
            self.accept_target(target, "arm")
        except (ValueError, ProtocolError) as exc:
            if self.armed:
                self.halt_stream(f"Rejected joint command: {exc}")

    def on_gripper_command(self, message):
        try:
            if len(message.data) != 2:
                raise ProtocolError("Gripper command is [motor_angle_rad, current_mA]")
            snapshot = self.require_stream()
            target = list(self.sent if self.sent is not None else snapshot["position_deg"])
            target[5] = math.degrees(message.data[0])
            self.accept_target(target, "gripper", float(message.data[1]))
        except (ValueError, ProtocolError) as exc:
            if self.armed:
                self.halt_stream(f"Rejected gripper command: {exc}")

    def set_enabled(self, request, response):
        if not self.motion_allowed:
            response.success = False
            response.message = "Read-only launch; restart with allow_motion:=true"
            return response
        if self.arm is None:
            response.success = False
            response.message = "Serial disconnected; restart node after checking cable/firmware"
            return response
        try:
            if not request.data:
                self.armed = False
                self.target = None
                self.kind = None
                self.arm.query("T 0", "OK T 0", timeout=1.0)
                snapshot = self.refresh()
                if snapshot["torque"]:
                    raise ProtocolError("T0 not confirmed by D")
                self.phase = "disabled"
                self.error = ""
                response.message = "Firmware software torque OFF confirmed; support the arm"
            else:
                if self.armed:
                    raise ProtocolError("Already enabled; command stream required")
                snapshot = self.refresh()
                self.healthy(snapshot, (0, 1, 2, 4))
                if snapshot["torque"]:
                    raise ProtocolError(
                        "Torque already ON. Support arm and call set_enabled false first")
                self.arm.identity()
                self.arm.query("R", "OK R state=0", timeout=2.0)
                snapshot = self.refresh()
                self.healthy(snapshot, (0,))
                if snapshot["torque"]:
                    raise ProtocolError("Unexpected torque ON before T1")
                self.sent = list(snapshot["position_deg"])
                try:
                    self.arm.query("T 1", "OK T 1", timeout=2.0)
                finally:
                    # Also arm the watchdog if T1 succeeded but its ACK was lost.
                    # A physical serial failure can still prevent this write.
                    self.arm.send("K")
                snapshot = self.refresh()
                self.healthy(snapshot, (0,))
                if not snapshot["torque"]:
                    raise ProtocolError("T1 not confirmed by D")
                self.armed = True
                self.target = None
                self.kind = None
                self.last_input = time.monotonic()
                self.last_send = self.last_input
                self.phase = "awaiting_command"
                self.error = ""
                response.message = (
                    "All six torques ON. Publish a live command stream within "
                    f"{self.command_timeout}s; no automatic keepalive")
            response.success = True
        except Exception as exc:
            self.halt_stream(str(exc))
            response.success = False
            response.message = str(exc) + "; actual torque may remain ON"
        self.publish_status()
        return response

    def stop(self, request, response):
        del request
        self.armed = False
        self.target = None
        self.kind = None
        if self.arm is None or not self.motion_allowed:
            response.success = False
            response.message = "No writable arm connection"
            return response
        try:
            reply = self.arm.query("S", "OK S", timeout=2.0)
            snapshot = self.refresh()
            response.success = (
                snapshot["state"] == 2 and not snapshot["fault_code"]
                and not snapshot["hold_fail_mask"] and not snapshot["offline_mask"])
            response.message = reply + "; firmware hold, torque retained"
            self.phase = "stopped"
            self.error = "" if response.success else "Firmware stop not confirmed"
        except Exception as exc:
            self.halt_stream(str(exc))
            response.success = False
            response.message = str(exc)
        self.publish_status()
        return response

    def send_target(self, now):
        snapshot = self.require_stream()
        dt = min(now - self.last_send, 1.0 / self.tx_hz)
        step = self.speed * max(0.0, dt)
        next_pose = [
            previous + max(-step, min(step, goal - previous))
            for previous, goal in zip(self.sent, self.target)]
        validate_pose(next_pose)
        validate_fold(snapshot["position_deg"], next_pose)
        if self.kind == "gripper":
            self.arm.send(f"G {next_pose[5]:.4f} {self.current_ma:.1f}")
        else:
            self.arm.send("A " + " ".join(f"{value:.4f}" for value in next_pose))
        self.sent = next_pose
        self.last_send = now

    def tick(self):
        if self.arm is None:
            if time.monotonic() - self.last_status >= 1.0:
                self.publish_status()
            return
        try:
            now = time.monotonic()
            if self.armed:
                if now - self.last_input > self.command_timeout:
                    self.halt_stream(
                        "Upstream command expired; A/G/K stopped, firmware watchdog holds")
                elif self.target is not None:
                    if now - self.last_send > 0.12:
                        self.halt_stream(
                            "Host command loop delayed; refusing automatic watchdog recovery")
                    elif now - self.last_send >= 1.0 / self.tx_hz:
                        self.send_target(now)
            if now - self.last_poll >= 1.0 / self.poll_hz:
                snapshot = self.refresh()
                if self.armed:
                    self.healthy(snapshot, (0,))
                    if not snapshot["torque"]:
                        self.halt_stream("Firmware torque OFF")
        except Exception as exc:
            self.halt_stream(f"Serial/telemetry failure: {exc}")
            self.arm.close()
            self.arm = None
            self.phase = "disconnected"
        if time.monotonic() - self.last_status >= 1.0:
            self.publish_status()

    def publish_status(self):
        now = time.monotonic()
        snapshot = self.snapshot_data
        payload = {
            "connected": self.arm is not None, "allow_motion": self.motion_allowed,
            "armed": self.armed, "phase": self.phase, "error": self.error,
            "firmware": FW, "joint_names": self.names,
            "motor_ids": [11, 12, 13, 14, 16, 15],
            "telemetry_age_sec": (
                now - snapshot["received_monotonic"] if snapshot else None),
            "telemetry": (
                {key: ([value if math.isfinite(value) else None for value in item]
                       if key == "position_deg" else item)
                 for key, item in snapshot.items() if key != "received_monotonic"}
                if snapshot else None),
        }
        message = String()
        message.data = json.dumps(payload, allow_nan=False)
        self.status_pub.publish(message)
        self.last_status = now

    def destroy_node(self):
        self.armed = False
        if self.arm is not None:
            self.get_logger().warning(
                "Closing serial; no S/T0 sent. Active firmware watchdog must handle hold.")
            self.arm.close()
            self.arm = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmBridge()
        # Serial and service callbacks intentionally share one executor/thread.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
