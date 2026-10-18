"""Portable evidence, explicit replay grades, and finite-sample summaries."""
from __future__ import annotations

import gzip
import json
import math
from pathlib import Path

from .runtime import Scenario, digest, environment, simulate, source_manifest


def write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    raw = (json.dumps(data, sort_keys=True, indent=2, allow_nan=False) + "\n").encode()
    if path.suffix == ".gz":
        path.write_bytes(gzip.compress(raw, mtime=0))
    else:
        path.write_bytes(raw)


def read_json(path):
    raw = Path(path).read_bytes()
    return json.loads(gzip.decompress(raw) if str(path).endswith(".gz") else raw)


def compact(run):
    return {k: v for k, v in run.items() if k not in ("rows", "checkpoint")}


def compare(expected, actual, atol=1e-8, rtol=1e-8):
    if digest(expected["rows"]) != expected["trace_hash"]:
        return {"grade": "DIVERGENT", "reason": "Stored trace checksum is invalid"}
    if digest(actual["rows"]) != actual["trace_hash"]:
        return {"grade": "DIVERGENT", "reason": "Actual trace checksum is invalid"}
    if actual["trace_hash"] == expected["trace_hash"]:
        if expected["verdict"] != actual["verdict"]:
            return {"grade": "DIVERGENT", "reason": "Verdict differs despite identical trace",
                    "max_absolute_difference": 0.0, "verdict_equal": False}
        return {"grade": "EXACT", "max_absolute_difference": 0.0}
    a, b = expected["rows"], actual["rows"]
    if len(a) != len(b):
        return {"grade": "DIVERGENT", "reason": "Trace length differs"}
    maxdiff, first, within = 0.0, None, True

    def visit(x, y, location):
        nonlocal maxdiff, first, within
        numeric = type(x) in (int, float) and type(y) in (int, float)
        if numeric:
            d = abs(x - y)
            maxdiff = max(maxdiff, d)
            same = math.isfinite(d) and d <= atol + rtol * abs(x)
        elif isinstance(x, list) and isinstance(y, list) and len(x) == len(y):
            for i, (xx, yy) in enumerate(zip(x, y)):
                visit(xx, yy, location + f"[{i}]")
            return
        elif isinstance(x, dict) and isinstance(y, dict) and x.keys() == y.keys():
            for key in x:
                visit(x[key], y[key], location + "." + key)
            return
        else:
            same = type(x) is type(y) and x == y
        if not same:
            within = False
            if first is None:
                first = location

    for i, (x, y) in enumerate(zip(a, b)):
        visit(x, y, f"row[{i}]")
    verdict_same = expected["verdict"] == actual["verdict"]
    return {"grade": "TOLERANT" if within and verdict_same else "DIVERGENT",
            "max_absolute_difference": maxdiff, "first_outside_tolerance": first,
            "atol": atol, "rtol": rtol, "verdict_equal": verdict_same}


def replay(evidence, contract):
    if (evidence["environment"] != environment() or evidence["contract_hash"] != digest(contract)
            or evidence["source_hash"] != digest(source_manifest())):
        return {"grade": "INCOMPATIBLE", "reason": "Source, contract or environment mismatch; no exact replay claim"}
    actual = simulate(Scenario(**evidence["scenario"]), evidence["controller"], contract,
                      physics_dt=evidence["physics_dt"])
    return compare(evidence, actual)


def wilson(failures, n, z=1.959963984540054):
    if n == 0:
        return None
    p = failures / n
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    radius = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return [max(0.0, center - radius), min(1.0, center + radius)]


def statistics(runs):
    statuses = {s: sum(r["verdict"]["overall"] == s for r in runs)
                for s in ("SAT", "VIOLATED", "INCONCLUSIVE", "NOT_EXERCISED")}
    n = statuses["SAT"] + statuses["VIOLATED"]
    return {"attempts": len(runs), "counts": statuses, "conclusive_exercised": n,
            "failure_fraction": statuses["VIOLATED"] / n if n else None,
            "wilson_95": wilson(statuses["VIOLATED"], n),
            "scope": "Only the declared synthetic sampling distribution; not field reliability."}
