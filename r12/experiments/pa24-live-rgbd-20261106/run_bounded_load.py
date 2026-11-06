"""One local 20-capture run under specified bounded GPU pressure."""
from pathlib import Path
import sys
import launch_live_view as launch
BASE=Path(__file__).resolve().parent
launch.RUN=BASE/'runs/bounded-load-20-01'
launch.MANIFEST_OVERRIDES=dict(identity_reobserve_challenge=False,calibration_view=False,
    live_motion_diagnostic=False,bounded_gpu_load=True,frames_per_phase=20,
    scope='20 fresh stationary all640 captures with 256MiB held GPU buffer and 4x2048 matmuls per render')
if __name__=='__main__':sys.exit(launch.main())
