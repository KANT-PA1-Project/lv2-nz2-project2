"""Isolate normal Isaac extension/interpreter teardown without scene or models."""
import argparse
import fcntl
import gc
import json
import os
from pathlib import Path
import time

parser = argparse.ArgumentParser()
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=False)
with (Path("/tmp") / ("r12-native-" + str(os.getuid()) + ".lock")).open("a+") as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    from isaacsim import SimulationApp
    app = SimulationApp(dict(headless=True, fast_shutdown=False, width=640, height=480,
        active_gpu=0, physics_gpu=0, multi_gpu=False))
    (args.output / "before_close.json").write_text(json.dumps(dict(
        pid=os.getpid(), monotonic_ns=time.monotonic_ns(), fast_shutdown=False)))
    app.close()
    del app
    gc.collect()
    (args.output / "after_close.json").write_text(json.dumps(dict(
        close_returned=True, monotonic_ns=time.monotonic_ns(),
        scope="API returned; process exit must be checked separately")))
    print("NORMAL_CLOSE_RETURNED", flush=True)
