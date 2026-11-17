"""Same-host producer ACK records and offline statistics. No runtime startup."""
import json
import math
from pathlib import Path

RECORD_SCHEMA = "v136-producer-ack-v1"
CLOCK = "same-host-monotonic-ns"
DEFINITION = {"start": "capture_monotonic_ns", "end": "producer_transport_timing.ack_receive_end_ns",
              "unit": "ms", "clock": CLOCK, "percentile": "linear interpolation at (n-1)*q"}
STAGES = {"blob_copy_ms": ("blob_copy_start_ns", "blob_copy_end_ns"),
          "serialization_ms": ("serialization_start_ns", "serialization_end_ns"),
          "unix_send_ms": ("unix_send_start_ns", "unix_send_end_ns"),
          "serialization_and_send_ms": ("serialization_start_ns", "unix_send_end_ns"),
          "ack_receive_ms": ("ack_receive_start_ns", "ack_receive_end_ns"),
          "ack_validation_ms": ("ack_receive_end_ns", "ack_validation_end_ns")}


def integer(value, name):
    if type(value) is not int or value < 0:
        raise ValueError("invalid_integer:" + name)
    return value


def capture_identity(run_id, source_sha, body):
    if body["run_id"] != run_id or body["source_sha256"] != source_sha:
        raise ValueError("ACK_capture_source_or_run_mismatch")
    step = integer(body["capture_physics_step"], "capture_physics_step")
    hz = integer(body["physics_hz"], "physics_hz")
    if not hz:
        raise ValueError("invalid_physics_rate")
    return {"run_id": run_id, "source_sha256": source_sha, "clock_epoch": body["clock_epoch"],
            "capture_physics_step": step, "physics_hz": hz, "capture_stamp_ns": step * 10**9 // hz,
            "capture_monotonic_ns": integer(body["capture_monotonic_lower_bound_ns"], "capture_monotonic_ns"),
            "clock_domain": CLOCK}


def make_ack_record(identity, result, timing):
    if result["capture_stamp_ns"] != identity["capture_stamp_ns"]:
        raise ValueError("ACK_capture_stamp_mismatch")
    record = dict(identity, schema=RECORD_SCHEMA, ack_result_kind=result["kind"],
                  all_received=result["all_received"], expected=result["expected"], received=result["received"],
                  errors=result["errors"], ros_domain_id=result["domain"], producer_transport_timing=dict(timing),
                  motion_authorized=False, scope="producer_transport_timing_only")
    normalize(record)
    return record


def normalize(record, *, legacy=False):
    if record.get("schema") != RECORD_SCHEMA or record.get("clock_domain") != CLOCK:
        raise ValueError("unsupported_ACK_schema_or_clock")
    for name in ("run_id", "clock_epoch"):
        if not isinstance(record.get(name), str) or not record[name]:
            raise ValueError("missing_ACK_identity:" + name)
    source = record.get("source_sha256")
    if not (legacy and source is None):
        if type(source) is not str or len(source) != 64 or any(c not in "0123456789abcdef" for c in source):
            raise ValueError("invalid_ACK_source_sha256")
    step = integer(record["capture_physics_step"], "capture_physics_step")
    stamp = integer(record["capture_stamp_ns"], "capture_stamp_ns")
    hz = integer(record["physics_hz"], "physics_hz")
    if not hz or stamp != step * 10**9 // hz:
        raise ValueError("ACK_capture_step_stamp_mismatch")
    expected = integer(record["expected"], "expected")
    received = integer(record["received"], "received")
    if record.get("ack_result_kind") != "publication_result" or record.get("all_received") is not True or record.get("errors") != [] or not expected or expected != received:
        raise ValueError("ACK_publication_incomplete")
    domain = integer(record["ros_domain_id"], "ros_domain_id")
    if domain > 232:
        raise ValueError("invalid_ACK_domain")
    capture = integer(record["capture_monotonic_ns"], "capture_monotonic_ns")
    timing = record["producer_transport_timing"]
    end = integer(timing["ack_receive_end_ns"], "ack_receive_end_ns")
    validation_end = integer(timing.get("ack_validation_end_ns", end), "ack_validation_end_ns")
    if not capture <= end <= validation_end:
        raise ValueError("ACK_clock_order")
    if "capture_monotonic_ns" in timing and timing["capture_monotonic_ns"] != capture:
        raise ValueError("ACK_capture_time_mismatch")
    stages = {}
    for name, (start_key, end_key) in STAGES.items():
        for key in (start_key, end_key):
            if key in timing:
                value = integer(timing[key], key)
                if not capture <= value <= validation_end:
                    raise ValueError("ACK_stage_clock_order:" + key)
        if start_key not in timing or end_key not in timing:
            stages[name] = None
        else:
            start_value, end_value = timing[start_key], timing[end_key]
            if end_value < start_value:
                raise ValueError("ACK_stage_clock_order:" + name)
            stages[name] = (end_value - start_value) / 1e6
    if "unix_send_end_ns" in timing and timing["unix_send_end_ns"] > end:
        raise ValueError("ACK_received_before_request_sent")
    return {"run_id": record["run_id"], "clock_epoch": record["clock_epoch"], "source_sha256": source,
            "capture_physics_step": step, "capture_stamp_ns": stamp, "capture_monotonic_ns": capture,
            "ack_receive_end_ns": end, "producer_ACK_ms": (end - capture) / 1e6,
            "ros_domain_id": domain, "required_samples": expected, "frame": record.get("frame"), "stages": stages}


def legacy_record(row):
    ros = row["ROS_publication"]
    verification = row["native_verification"]
    return {"schema": RECORD_SCHEMA, "clock_domain": CLOCK,
            "run_id": verification["run_id"], "clock_epoch": verification["clock_epoch"],
            "source_sha256": row.get("source_sha256"), "capture_physics_step": row["physics_step"], "physics_hz": 960,
            "capture_stamp_ns": ros["capture_stamp_ns"], "capture_monotonic_ns": row["capture_monotonic_ns"],
            "ack_result_kind": ros["kind"], "all_received": ros["all_received"], "expected": ros["expected"],
            "received": ros["received"], "errors": ros["errors"], "ros_domain_id": ros["domain"],
            "producer_transport_timing": ros["producer_transport_timing"], "frame": row.get("frame")}


def percentile(values, q):
    ordered = sorted(values)
    position = (len(ordered) - 1) * q
    low = math.floor(position)
    high = math.ceil(position)
    fraction = position - low
    delta = ordered[high] - ordered[low]
    return ordered[high] - delta * (1 - fraction) if fraction >= .5 else ordered[low] + delta * fraction


def stats(values):
    if not values:
        return None
    return {"samples": len(values), "p50": percentile(values, .5), "p95": percentile(values, .95), "max": max(values)}


def read_records(path):
    path = Path(path).expanduser().resolve(strict=True)
    if path.is_dir():
        journal = path / "producer_ack.jsonl"
        path = journal if journal.is_file() else path / "native_producer_frames.json"
    text = path.read_text(encoding="utf-8")
    if path.suffix == ".jsonl":
        rows = []
        for index, line in enumerate(text.splitlines(), 1):
            if not line.strip():
                continue
            try:
                rows.append((index, json.loads(line), None))
            except json.JSONDecodeError as error:
                rows.append((index, None, "invalid_JSON_line:" + str(error)))
    else:
        data = json.loads(text)
        data = data if isinstance(data, list) else [data]
        rows = [(index, row, None) for index, row in enumerate(data, 1)]
    return path, rows


def analyze(path):
    path, rows = read_records(path)
    samples, errors = [], []
    seen, last = set(), {}
    skipped = 0
    for index, row, parse_error in rows:
        try:
            if parse_error:
                raise ValueError(parse_error)
            if not isinstance(row, dict):
                raise ValueError("ACK_record_must_be_object")
            legacy = row.get("schema") != RECORD_SCHEMA
            if legacy and row.get("captured") is False:
                skipped += 1
                continue
            sample = normalize(legacy_record(row) if legacy else row, legacy=legacy)
            session = (sample["run_id"], sample["clock_epoch"])
            if last and session not in last:
                raise ValueError("mixed_ACK_sessions")
            key = session + (sample["capture_physics_step"],)
            if key in seen:
                raise ValueError("duplicate_ACK_capture")
            previous = last.get(session)
            if previous:
                if sample["source_sha256"] != previous["source_sha256"] or sample["ros_domain_id"] != previous["ros_domain_id"]:
                    raise ValueError("ACK_session_identity_changed")
                if sample["capture_physics_step"] <= previous["capture_physics_step"] or sample["capture_monotonic_ns"] <= previous["capture_monotonic_ns"]:
                    raise ValueError("reordered_ACK_capture")
            seen.add(key); last[session] = sample
            samples.append(dict(sample, input_record=index, input_layout="v135_frames" if legacy else RECORD_SCHEMA))
        except (KeyError, ValueError, TypeError, OverflowError) as error:
            errors.append({"input_record": index, "reason": str(error)})
    if not samples:
        errors.append({"reason": "no_valid_ACK_samples"})
    subset = stats([sample["producer_ACK_ms"] for sample in samples])
    return {"schema": "v136-ACK-analysis-v1", "input": str(path), "definition": DEFINITION,
            "valid": not errors, "summary": subset if not errors else None,
            "valid_subset_summary": subset if errors else None, "skipped_uncaptured": skipped, "errors": errors,
            "stage_summary": {name: stats([s["stages"][name] for s in samples if s["stages"][name] is not None]) for name in STAGES},
            "stage_missing_samples": {name: sum(s["stages"][name] is None for s in samples) for name in STAGES},
            "samples": samples, "scope": "offline_transport_statistics_not_mission_acceptance"}
