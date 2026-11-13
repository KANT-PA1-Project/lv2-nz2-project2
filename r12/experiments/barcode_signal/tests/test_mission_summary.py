"""Camera-free contract tests; synthetic timing is not real detection evidence."""
import importlib.util
from pathlib import Path
import sys
import unittest

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from mission_summary import MissionSession, requirements
from replay_mission_summary import replay
from barcode_support import decode_rgb
from live_signal_guard import body_present

SEQ = "RGYRGYGRGRGG"


def stream(sequence=SEQ):
    # 20 Hz: each flash 1 second, then OFF. End OFF totals >=3 seconds.
    values = [x for c in sequence for x in [c] * 20 + ["OFF"] * 20]
    values += ["OFF"] * 41
    return [dict(kind="signal", t_s=(i+1)/20, color=c) for i, c in enumerate(values)]


def initial(code="P2-01"):
    return ([dict(kind="barcode", reads=[dict(text=code)]) for _ in range(3)] +
            [dict(kind="arm", t_s=0., confirmed_off="operator_o", fixed_settings=True)])


class MissionSummaryTests(unittest.TestCase):
    def test_four_barcodes(self):
        for code in ("P2-01", "P2-03", "P2-02", "P2-04"):
            out = requirements(SEQ, code)
            self.assertEqual([x["group"] for x in out["locations"]],
                             list("YRG" if code in ("P2-01", "P2-03") else "GRY"))
            self.assertEqual(out["counts"], dict(R=4, Y=2, G=6))
            self.assertEqual(sum(x["count"] for x in out["locations"]), 12)
            self.assertEqual(out["puck_allocation"], "pending_inventory")
            self.assertNotIn("targets", out["locations"][0])

    def test_ties_keep_RYG_in_both_directions(self):
        for code in ("P2-01", "P2-02"):
            self.assertEqual([x["group"] for x in requirements("RYG"*4, code)["locations"]], list("RYG"))

    def test_invalid_requirements(self):
        for seq, code in [(SEQ[:-1], "P2-01"), ("R"*12, "P2-01"),
                          (SEQ[:-1]+"X", "P2-01"), (SEQ, "P2-99")]:
            with self.assertRaises(ValueError):
                requirements(seq, code)

    def test_complete_replay_with_actual_barcode_assets(self):
        for code in ("P2-01", "P2-02", "P2-03", "P2-04"):
            image = np.asarray(Image.open(ROOT / "references" / (code+".png")).convert("RGB"))
            reads = decode_rgb(image)["reads"]
            events = [dict(kind="barcode", reads=reads) for _ in range(3)] + initial(code)[3:] + stream()
            out = replay(events)
            self.assertEqual(out["status"], "ready")
            self.assertEqual(out["sequence"], list(SEQ))

    def test_not_complete_before_three_seconds_OFF(self):
        out = replay(initial() + stream()[:-2])
        self.assertFalse(out["signal_complete"])
        self.assertEqual(out["locations"], [])

    def test_same_color_flashes_are_separate(self):
        out = replay(initial() + stream("RRRRYYYYGGGG"))
        self.assertEqual(out["status"], "ready")
        self.assertEqual(out["counts"], dict(R=4, Y=4, G=4))

    def test_unknown_and_gap_latch_error(self):
        for event in [dict(kind="signal", t_s=.05, color="UNKNOWN"),
                      dict(kind="signal", t_s=.3, color="R")]:
            out = replay(initial() + [event] + stream())
            self.assertEqual(out["status"], "error")
            self.assertEqual(out["locations"], [])

    def test_missing_barcode_never_ready(self):
        out = replay(initial()[3:] + stream())
        self.assertTrue(out["signal_complete"])
        self.assertEqual(out["locations"], [])
        self.assertNotEqual(out["status"], "ready")

    def test_barcode_change_invalidates_ready(self):
        out = replay(initial() + stream() + [dict(kind="barcode", reads=[dict(text="P2-02")])])
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["locations"], [])

    def test_ambiguous_or_unknown_barcode_fails(self):
        for reads in [[dict(text="P2-99")], [dict(text="P2-01"), dict(text="P2-02")]]:
            self.assertEqual(replay([dict(kind="barcode", reads=reads)])["status"], "error")

    def test_arm_requires_operator_OFF_and_fixed_settings(self):
        for color, off, fixed in [("R", True, True), ("OFF", False, True), ("OFF", True, False)]:
            session = MissionSession()
            self.assertFalse(session.arm(0., color, confirmed_off=off, fixed_settings=fixed))
            self.assertFalse(session.armed)

    def test_reset_discards_result(self):
        out = replay(initial() + stream() + [dict(kind="reset")])
        self.assertEqual(out["status"], "waiting_arm")
        self.assertEqual(out["locations"], [])
        self.assertIsNone(out["barcode"])

    def test_more_flashes_after_ready_invalidates(self):
        out = replay(initial() + stream() + [dict(kind="signal", t_s=26.1, color="R")])
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["locations"], [])

    def test_presence_guard_rejects_empty_fixed_ROI(self):
        image = np.zeros((90, 30, 3), np.uint8)
        boxes = {c: (0, i*30, 30, (i+1)*30, 0.) for i, c in enumerate("RYG")}
        self.assertFalse(body_present(image, boxes))
        for i, rgb in enumerate([(120, 0, 0), (140, 100, 0), (0, 100, 50)]):
            image[i*30:(i+1)*30] = rgb
        self.assertTrue(body_present(image, boxes))
        image[:30] = 255
        self.assertTrue(body_present(image, boxes))

    def test_terminal_failure_persists_in_replay(self):
        out = replay(initial() + stream() + [dict(kind="failure", reason="Camera timeout")])
        self.assertEqual(out["status"], "error")
        self.assertEqual(out["locations"], [])
