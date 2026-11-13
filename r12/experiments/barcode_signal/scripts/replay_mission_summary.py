"""Replay live mission JSONL; does not start Isaac, a camera or a robot."""
import argparse
import json
from pathlib import Path
from mission_summary import MissionSession


def replay(events):
    session = MissionSession()
    for event in events:
        kind = event["kind"]
        if kind == "reset":
            session = MissionSession()
        elif kind == "signal_retry":
            code = session.barcode
            session = MissionSession()
            session.barcode = code
        elif kind == "barcode":
            session.observe_barcodes(event["reads"], time_s=event.get("capture_s", event["t_s"]))
        elif kind == "arm":
            source = event.get("confirmed_off")
            preparation = event.get("preparation") or {}
            confirmed = source == "operator_o" or (
                source == "prestart_off_contract"
                and preparation.get("prestart_off_guaranteed") is True
                and preparation.get("stable_off_s", 0) >= 1
                and preparation.get("baseline_frames", 0) >= 5) or (
                source == "retry_long_off"
                and preparation.get("recovery_long_off_observed") is True
                and preparation.get("stable_off_s", 0) >= 2.0
                and preparation.get("baseline_frames", 0) >= 5)
            accepted = session.arm(event["t_s"], "OFF",
                                   confirmed_off=confirmed,
                                   fixed_settings=event.get("fixed_settings") is True)
            if not accepted:
                session.fail("Replay arm rejected")
        elif kind == "signal":
            session.observe(event["t_s"], event["color"])
        elif kind == "failure":
            session.fail(event["reason"])
        else:
            raise ValueError(f"Unknown event: {kind}")
    return session.snapshot()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    result = replay(json.loads(line) for line in args.jsonl.read_text().splitlines() if line.strip())
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + chr(10))
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result["status"] == "ready" else 1


if __name__ == "__main__":
    raise SystemExit(main())
