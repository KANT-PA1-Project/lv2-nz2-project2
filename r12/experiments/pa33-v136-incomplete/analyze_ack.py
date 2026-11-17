"""Analyze v135 producer frames or v136 producer_ack.jsonl using one ACK endpoint."""
import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "overlay/arm_practice/native_rgbd"))
from ack_metrics import analyze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="record file or run directory")
    parser.add_argument("--baseline", type=Path, help="optional baseline with the same metric definition")
    parser.add_argument("--output", type=Path, help="full JSON report; stdout then shows a compact summary")
    args = parser.parse_args()
    try:
        report = analyze(args.input)
        if args.baseline:
            baseline = analyze(args.baseline)
            comparable = report["valid"] and baseline["valid"]
            report["baseline"] = baseline
            report["candidate_minus_baseline_ms"] = {
                key: report["summary"][key] - baseline["summary"][key] for key in ("p50", "p95", "max")
            } if comparable else None
            report["valid"] = bool(comparable)
        text = json.dumps(report, indent=2, allow_nan=False)
        if args.output:
            protected = {Path(report["input"]).resolve()}
            if "baseline" in report:
                protected.add(Path(report["baseline"]["input"]).resolve())
            if args.output.expanduser().resolve() in protected:
                raise ValueError("analysis_output_must_not_replace_input")
            args.output.write_text(text + "\n", encoding="utf-8")
            print(json.dumps({"output": str(args.output), "valid": report["valid"], "summary": report["summary"],
                "errors": report["errors"], "baseline_errors": report.get("baseline", {}).get("errors"),
                "candidate_minus_baseline_ms": report.get("candidate_minus_baseline_ms")}, indent=2))
        else:
            print(text)
        return 0 if report["valid"] else 2
    except (OSError, ValueError, TypeError, KeyError) as error:
        print(json.dumps({"valid": False, "error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
