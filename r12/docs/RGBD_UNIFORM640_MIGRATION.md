# Uniform640 and CAD wrist mount integration

The common camera profile now selects `r12-camera-calibration-v2-uniform640` /
`r12-rgbd-wire-v3-uniform640`: eight RGB8 and four float32 depth arrays,
all 640×480, totaling **12,288,000 bytes**. K = (400,400,319.5,239.5).
The schema implementation also accepts the original explicitly selected mixed
profile (36,864,000 bytes); `WIRE_V2` retains its original meaning. There is
no automatic resize, version alias, or fallback between profiles. Both peers
must load the identical schema and calibration hashes before exchanging pixels.

The authoritative wrist optical configuration now points 30° below arm_link5 +X
(optical forward = [sqrt(3)/2,0,-0.5]). Translation [0.04263,0,0.05244] is the
provisional CAD housing front-face center. Evidence is the rotated camera housing
in CAD136_R12_V10 / arm_link5_electronics.stl, documented in the v46 candidate.
Baseline 0.05m and optical center remain assumptions, not measured hand-eye
calibration. Both ordinary and cached camera FK use this same configuration.
Old forward-facing captures, fitted calibration and view scores are historical;
they do not validate the corrected mount.

## PA33 consumer preparation

The archived pa33-v135 snapshot remains immutable. Create a separate candidate:

```sh
python scripts/prepare_r12_pa33_uniform640.py --r12-root /absolute/path/to/r12 --output /absolute/path/to/runs/pa33-uniform640
```

Preparation copies the PA33 decoder/publisher and dependencies, preserves its send
timing instrumentation, adds dual-profile transport support, installs the same
rig/schema as the producer, and regenerates candidate source hashes.
Existing outputs are refused. Model/assets links point to the specified local
R12 tree: preparation is local and does not deploy or start PA33.
The original host-specific neural model paths and ROS runtime dependencies still
need explicit setup before an actual PA33 inference/ROS run.

## Local render reproduction

Use the Isaac Python environment (includes scipy), on the local PC only:

```sh
/path/to/isaac/python scripts/run_r12_down30_diagnostic.py --output /absolute/path/to/new-output
```

The tool makes a fresh source snapshot from the archived diagnostic, installs the
common rig/schema, uses q=[0,-0.25,0.10,-0.05,0], and delegates PID/lock/GPU/budget
checks to the existing launcher. It captures three fresh frames after settling,
does not dispatch motion, and refuses an existing output.
`--prepare-only` prepares without launching Isaac.
The supervisor tolerates temporary telemetry files disappearing during atomic
rename; the 500MiB disk budget remains enforced.

## Validation boundaries

CPU tests exercise both complete binary payloads through local sockets, reject
wrong profile/schema/K/length combinations before reading payloads, and verify
CAD down30 plus ordinary/cached FK agreement at three joint poses.
Local consumer validation covers the PA33 transport implementation. It does not
claim a remote neural inference, DDS throughput, or full mission pass.

Current local validation: 102 common targeted tests and 104 PA33 candidate tests
passed; a separate-process producer-to-PA33 test passed for both payloads.
The wider native suite had 324 passes and four pre-existing failures:
AdapterHistoricalPoseTests patches a removed transforms symbol; the same four
fail against the pre-change source. These failures are not counted as passes.

Regenerated schema hashes invalidate prior capture/config bindings as intended.
Keep `calibration_accepted=false` and `motion_authorized=false` until independent
calibration and current-run safety requirements pass.
