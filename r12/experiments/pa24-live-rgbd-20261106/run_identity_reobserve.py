"""Rendered duplicate-ID rejection followed by two new observations."""
from pathlib import Path
import sys
import launch_live_view as launch
BASE=Path(__file__).resolve().parent
launch.RUN=BASE/'runs/identity-reobserve-01'
launch.MANIFEST_OVERRIDES=dict(identity_reobserve_challenge=True,calibration_view=False,
    live_motion_diagnostic=False,bounded_gpu_load=False,frames_per_phase=3,
    scope='same-ID ambiguity rejection then two new consistent observations')
if __name__=='__main__':sys.exit(launch.main())
