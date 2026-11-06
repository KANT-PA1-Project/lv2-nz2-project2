# PA24 local RGB-D diagnostic evidence — 2026-11-06

Local headless simulation only. This snapshot is a diagnostic, not a mission deployment or a calibrated grasp controller. The independent v46 candidate is unchanged.

## Verified behavior

- Native 8 RGB / 4 depth streams use 640×480 and a separate diagnostic wire/schema version. The production mixed-resolution IPC contract is unchanged.
- Current issuer verification happens in memory before compression. Archived certificates are records only; no secret-key replay or timestamp refresh is supported.
- Conflicting scale IDs and unconfirmed groups remain visible in `detect_tag_evidence`; legacy `detect_tags` / `agreed_tags` output is unchanged.
- A rejected identity needs two new increasing physics epochs with the same registered object ID. Destination IDs cannot become an object target. This does not prove global uniqueness of a repeated physical marker.
- The duplicate-ID rendered challenge passed: two ID0 objects → rejection → one new observation pending → second new observation confirmed.
- Two isolated executions passed fresh observation → current geometry check → bounded diagnostic joint move → 0.5s native/FD settling → fresh reobservation. The attempted move is capped at 0.0004rad; this proves a small movement connection, not mission pose arrival.
- The same-execution guard covers 447 robot colliders, native/CAD frame alignment, a static external collider, a conservative joint region, native/FD velocity, joint margins, contact veto, and an unchanged environment. Geometry-map values are never used as the observed target.
- Twenty fresh captures passed under a bounded synthetic GPU load: a held 256MiB buffer plus four 2048×2048 matrix multiplications per render. Ready-time p95 78.3ms, max 189.7ms; minimum free GPU memory 3198MiB. This is finite-sample evidence, not an all-load timing guarantee.

The initial moving attempt expired at the 500ms freshness limit and is a failure. The successful variant reduces redundant feedback reads and caps the diagnostic step; it does not relax freshness. Full mission `motion_authorized` and `calibration_accepted` remain false.

## Reproduce in an isolated checkout

Use an existing Isaac Sim Python runtime with NumPy, SciPy, OpenCV, PyTorch and USD support. No dependency installation or remote execution is part of this snapshot.

```bash
export PA24_ISAAC_PYTHON=/path/to/isaac/python
"$PA24_ISAAC_PYTHON" launch_live_view.py
"$PA24_ISAAC_PYTHON" run_identity_reobserve.py
"$PA24_ISAAC_PYTHON" run_live_motion.py
"$PA24_ISAAC_PYTHON" run_bounded_load.py
```

Run these sequentially, never concurrently. Each output directory must be new; existing results are not overwritten. `source/models` and `source/assets` refer to the repository's existing model/assets directories, and are rebased to resolved paths in a new run snapshot. Fetch the existing LFS assets before execution if needed. No models or raw sensor captures are duplicated in Git.

All executions use the shared local lock, inspect active renderer processes, pin source/model hashes and seed, enforce GPU free memory ≥1024MiB, 360s per-run wall budget, and 500MiB per-run disk budget. The lock defaults to `$HOME/isaac-fleet-runs/pa24-r12-experiment.lock`; `PA24_EXPERIMENT_LOCK` can select the same shared lock on another local host. No unrelated process is terminated. A renderer error, missing result, expired observation, failed content/identity check, or failed native motion evidence prevents success even when a child returns exit code zero.

## Simulation calibration candidate and remaining gates

`calibration_views.py` first screens eight stationary initial poses; it does not move between them. After baseline acquisition, `fit_sim_camera.py` uses seven distinct poses for fitting and two excluded poses for validation. Inputs are RGB-D, marker dimensions, schema intrinsics and encoder FK to arm_link5. Fixture target truth and nominal camera mount parameters are excluded from fitting. Post-fit simulation truth is used only by `evaluate_sim_camera_only.py`.

The left-wrist camera candidate had held-out consistency translation RMS 0.183mm, but a separate comparison with the authored simulation mount showed 2.16mm translation difference. It was not adopted. Small residuals are not an absolute error bound. Other camera calibrations, independent pad/TCP measurements, a validated uncertainty budget, and a terminal contact exception are still unverified.

`audit_pad_tcp_frames.py` distinguishes the authored fingertip TCP (link5 X=0.12475m) from the modeled opposing-pad midpoint (X≈0.10445m), a 20.3mm frame-origin difference. These are different frames, not interchangeable definitions. The audit is CAD/collider geometry, not independent metrology. No calibration file is silently replaced.

The current calibration contract therefore continues to reject terminal grasp. No grasp, lift, carry, place, firmware change or physical robot execution is claimed by this snapshot. A stationary or small-motion diagnostic pass must not be used as a full-mission certificate.

`audit_remaining_gates.py` checks the current repository without manufacturing a pose or calibration receipt. The current terminal guard returns `calibration_unverified`; the independent pad/TCP measurements and motor-to-finger force mapping are absent, and L1/L2/L3 marker-to-grid mount transforms remain unverified. These are final mission gates, not failed capture tests. Do not clear them by copying CAD coordinates into a measured-calibration report.

## Checks and evidence

- Diagnostic CPU boundary/math/geometry tests: 62 passed with the local Isaac runtime's OpenCV 4.14.
- Shared legacy, identity-evidence and marker-registry tests: 24 passed with the repository environment's OpenCV 5.0.
- Small tracked summary: `../../evidence/pa24-local-rgbd-20261106.json`.
- Raw NPZ, images, full runtime logs and run outputs are deliberately excluded from Git.
