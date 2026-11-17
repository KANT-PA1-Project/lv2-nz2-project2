"""One current-execution observe / guarded tiny move / settle / reobserve."""
from pathlib import Path
import sys
import launch_live_view as launch

BASE=Path(__file__).resolve().parent
launch.RUN=BASE/'runs/live-motion-confirm-03'
launch.MANIFEST_OVERRIDES=dict(identity_reobserve_challenge=False,calibration_view=False,
    live_motion_diagnostic=True,scope='current RGB-D guarded diagnostic view adjustment, settle and reobserve')
if __name__=='__main__':sys.exit(launch.main())
