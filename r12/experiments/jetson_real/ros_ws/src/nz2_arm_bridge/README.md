# NZ2 arm ROS 2 bridge

## Scope and firmware

This package adapts **existing team v2 + I handshake** to ROS 2. It does not modify
firmware. Compatible source:
`/home/pa33/git/p2_perception/firmware/nz2_arm_v2_handshake/`.

It requires the same open serial connection to report:
- `I`: board=arm, kind=arm, proto=1, fw="nz2_arm v2 handshake1",
  fw_version="2.0", limits_rev=8, order=[j1,j2,j3,j4,roll,grip].
- `?`: fw=2.0, limits_rev=8, ok=6/6 before enabling motion.
- `Q`, `D`, `E`: original v2 telemetry.

diag2/live15/v2b/v2c are not accepted. Scan sends **only I** to unidentified
ports; in particular it never sends E to an unidentified swerve board.
Multiple arm boards, identity mismatch or serial loss prevents control.
There is no automatic reconnect/re-enable. Stop other serial tools before running.
Opening a serial device may reset a board depending on USB/driver behavior; support
the arm before starting/restarting the node. Startup sends no R/T/S/A/G/K.

This is a joint-space serial bridge, **not an autonomous pick-and-place node**.
IK, mission state machine and learned-policy observation/action adapters are separate.
A .pt file cannot be passed directly to this bridge. Hardware validation is pending.

## Build and read-only start

Use system Python/ROS, outside the training venv. On this PC:

```bash
cd "$HOME/git/p2_perception/ros_ws"
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install --packages-select nz2_arm_bridge
source install/setup.bash
ros2 launch nz2_arm_bridge arm_bridge.launch.py
```

On Jetson, source its installed ROS distribution instead (usually Humble).
If rosdep did not already install pyserial: `sudo apt install python3-serial`.

The default start is read-only. Observe in a second sourced terminal:

```bash
ros2 topic echo /arm/status
# Or:
ros2 topic echo /arm/joint_states
```

Use `port:=/dev/ttyACM2` only to narrow discovery; the same I identity check still runs.
A failed connection is visible on /arm/status; fix the cause and restart.

## ROS contract (for IK/policy/mission authors)

| Direction | Name / ROS type | Meaning |
|---|---|---|
| Input | /arm/joint_command, sensor_msgs/msg/JointState | Six absolute motor angles in rad, named j1,j2,j3,j4,roll,grip |
| Input | /arm/gripper_command, std_msgs/msg/Float64MultiArray | Exactly [absolute motor angle in rad, current limit in mA] |
| Output | /arm/joint_states, sensor_msgs/msg/JointState | Q cached motor angles, converted to rad; no velocity or effort estimate |
| Output | /arm/status, std_msgs/msg/String | JSON phase/error, torque, firmware state/fault, positions deg, currents mA, masks and telemetry age |
| Service | /arm/set_enabled, std_srvs/srv/SetBool | true: verify, R, T1, K once. false: T0 |
| Service | /arm/stop, std_srvs/srv/Trigger | Original S latch, retains torque |

Order is ID11,12,13,14,16,15. Message names may arrive in any order but all six must
be present exactly once. velocity/effort must be empty. Header timestamps are
checked when nonzero; zero means receipt time. Publish fresh commands at **20 Hz**.
Both inputs are volatile, depth 1; pre-enable commands are discarded.

These are **firmware motor coordinates**, including its per-motor zero, NOT an
assumed match to arbitrary URDF/training coordinates. In particular grip is a motor
angle in radians, not finger gap in metres. Configuring different joint_names changes
labels only, not axes/signs/zeros. The IK/policy adapter must explicitly map its
joint order, signs, units, action scales and references to these coordinates.
For a five-joint policy, the mission controller must add the commanded gripper angle.
Only one upstream controller should own these inputs. When both topics are used,
the most recently received command replaces the previous command mode.

Joint input sends A with six deg values. Gripper input sends G with deg and mA
(0..300); it does not send an A command to the other joints. A commands retain the
firmware's existing gripper current setting (150 mA after setup unless G changed it).
No automatic "open to zero": grip=0 is outside this firmware's allowed range.

Firmware limits, repeated here without calibration changes:

| Motor ID | Min deg | Max deg |
|---|---:|---:|
| 11 | -175 | 175 |
| 12 | -95 | 113 |
| 13 | -111 | 82 |
| 14 | -99 | 117 |
| 16 | -170 | 170 |
| 15 | -134 | -3 |

Out-of-range/nonfinite targets are rejected, not silently clamped by the host.
The original fold constraint (shoulder<-80, elbow>45, wrist<=0; wrist<=3 before
entering) is checked. This is not collision checking or trajectory planning.
The host ramps outgoing targets at max_speed_deg_s (default 10); this is a bound
on commanded target change, **not a guarantee of measured speed or stopping distance**.

## Enable and stop behavior

Starting with `allow_motion:=true` only permits services; it does not enable torque:

```bash
ros2 launch nz2_arm_bridge arm_bridge.launch.py allow_motion:=true
```

The mission/IK/policy publisher should already be running at 20 Hz with a valid
near-current pose. Its pre-enable messages are ignored. Once the arm is supported
and telemetry is healthy/in bounds, enable explicitly:

```bash
ros2 service call /arm/set_enabled std_srvs/srv/SetBool '{data: true}'
```

**Original T1 enables all six motors**, including for gripper-only commands.
It uses original firmware goal/current handling; the bridge does not fix firmware
write checking or prove that past unexpected motion is resolved.
The node performs Q/D/E and identity checks, then R/T1, and K once to start the
firmware watchdog (T1 alone does not start it). Later only live A/G streams keep
the watchdog alive. Supply the first valid command promptly (20 Hz publisher).
It does not synthesize an indefinite K heartbeat.

If upstream input expires (default 250 ms), telemetry becomes invalid or serial
fails, the node discards the target and stops A/G/K. A host send gap over 120 ms also
halts the stream rather than implicitly resuming a watchdog stop. Firmware's original watchdog
then requests hold after 150 ms from its last A/G/K. A watchdog/latch stop does not
automatically resume on a new topic message; support the arm, explicitly disable
and enable again. No automatic S or T0 is injected by this path.

Explicit original firmware S stop:

```bash
ros2 service call /arm/stop std_srvs/srv/Trigger '{}'
```

S retains torque and uses the original firmware hold/profile behavior. D confirms
software state/hold_fail mask only. It does not prove physical stopping.

To release torque, **support the whole arm first**:

```bash
ros2 service call /arm/set_enabled std_srvs/srv/SetBool '{data: false}'
```

Ctrl-C/normal close sends no S/T0, stops the stream and closes serial. The original
firmware watchdog retains torque while holding; closing a process is not power-off.
No actual motor TORQUE_ENABLE register readback is available in this firmware.
A/G have no ACK; Q is cached (nominal per-motor cycle about 60 ms).
Serial reconnection, firmware faults, calibration and collision safety need real
hardware validation before mission use.

## Implementation status

Added source, launch, config and package installation rules.
2026-11-14: colcon build --symlink-install --packages-select nz2_arm_bridge succeeded
(1 package, 0.69 s). This confirms package build/install, not Python runtime or motor operation.
No automated tests, node execution, serial access, firmware upload or physical motion
performed for this change. Model loading, IK conversion, FollowJointTrajectory and
mission execution are not implemented by this package.
