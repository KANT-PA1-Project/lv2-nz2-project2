"""Live mission adapter; barcode decoding runs outside the camera loop."""
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

from mission_summary import MissionSession


class MissionLive:
    def __init__(self, out, upscale=1):
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)
        self.upscale = upscale
        self.session = MissionSession()
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None
        self.epoch = self.future_epoch = 0
        self.next_decode = 0.
        self.last_snapshot = None
        self.run_id = time.time_ns()
        self.signal_saved = False
        self.last_signal_time = None
        self.log = (self.out / f"mission_{time.time_ns()}.jsonl").open("w")
        self.publish()

    def event(self, **event):
        self.log.write(json.dumps(event, ensure_ascii=False) + chr(10))

    def publish(self):
        snapshot = self.session.snapshot()
        if snapshot["signal_complete"] and not snapshot["error"] and not self.signal_saved:
            record = dict(sequence=list(snapshot["sequence"]), counts=dict(snapshot["counts"]),
                          signal_complete=True, completed_at_camera_s=self.last_signal_time,
                          run_id=self.run_id, round_index=self.epoch,
                          scope="completed_signal_observation_not_puck_allocation")
            path = self.out / f"signal_record_{self.run_id}_{self.epoch}.json"
            temporary = path.with_suffix(".tmp")
            temporary.write_text(json.dumps(record, ensure_ascii=False, indent=2) + chr(10), encoding="utf-8")
            temporary.replace(path)
            self.signal_saved = True
            print("[mission] SIGNAL SAVED:", path, flush=True)
        encoded = json.dumps(snapshot, ensure_ascii=False, sort_keys=True)
        if encoded != self.last_snapshot:
            if snapshot["error"] is not None:
                self.event(kind="failure", reason=snapshot["error"])
            temporary = self.out / "mission_summary.tmp"
            temporary.write_text(encoded + chr(10), encoding="utf-8")
            temporary.replace(self.out / "mission_summary.json")
            self.last_snapshot = encoded
            print("[mission]", encoded)
        return snapshot

    def process(self, rgb, t_s, color):
        from barcode_support import decode_rgb
        if self.future is not None and self.future.done():
            try:
                decoded = self.future.result()
                if self.future_epoch == self.epoch:
                    self.session.observe_barcodes(decoded["reads"], time_s=t_s)
                    self.event(kind="barcode", t_s=t_s, **decoded)
            except Exception as exc:
                self.session.fail(f"Barcode decoder failed: {exc}")
            self.future = None
        if self.future is None and time.monotonic() >= self.next_decode and not self.session.error:
            self.future_epoch = self.epoch
            self.future = self.pool.submit(decode_rgb, rgb.copy(), upscale=self.upscale)
            self.next_decode = time.monotonic() + .2
        if color is not None and not self.session.recorder.complete:
            self.last_signal_time = t_s
            self.session.observe(t_s, color)
            if self.session.armed:
                self.event(kind="signal", t_s=t_s, color=color)
        return self.publish()

    def arm(self, t_s, color, *, confirmed_off, fixed_settings,
            confirmation_source="operator_o", preparation=None):
        if confirmation_source not in ("operator_o", "prestart_off_contract", "retry_long_off"):
            return False
        if confirmation_source == "prestart_off_contract":
            if not preparation or preparation.get("prestart_off_guaranteed") is not True or preparation.get("stable_off_s", 0) < 1 or preparation.get("baseline_frames", 0) < 5:
                return False
        if confirmation_source == "retry_long_off":
            if (not preparation or preparation.get("recovery_long_off_observed") is not True
                    or preparation.get("stable_off_s", 0) < 2.0
                    or preparation.get("baseline_frames", 0) < 5):
                return False
        accepted = self.session.arm(t_s, color, confirmed_off=confirmed_off,
                                    fixed_settings=fixed_settings)
        if accepted:
            self.event(kind="arm", t_s=t_s, confirmed_off=confirmation_source,
                       fixed_settings=fixed_settings, preparation=preparation)
        self.publish()
        return accepted

    def reset(self):
        self.epoch += 1
        self.signal_saved = False
        self.last_signal_time = None
        self.session = MissionSession()
        self.event(kind="reset")
        self.publish()

    def close(self, error=None):
        if error:
            self.session.fail(error)
        elif self.session.snapshot()["status"] != "ready" and not self.session.error:
            self.session.fail("Stopped before mission requirements completed")
        self.publish()
        self.pool.shutdown(wait=False, cancel_futures=True)
        self.log.close()
