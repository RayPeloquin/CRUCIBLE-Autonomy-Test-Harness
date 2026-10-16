"""Sampled finite-trace monitor. Bounds are dimensionless after declared scaling.

No interpolation or continuous-time safety claim is made. Closed-grid dwell means
both endpoints are included. A known breach survives missing/invalid telemetry.
"""
from __future__ import annotations

import json
import math
from pathlib import Path


def ticks(value, dt):
    q = value / dt
    if not math.isfinite(q) or abs(q - round(q)) > 1e-8:
        raise ValueError(f"Time {value} is not on the {dt} s control grid")
    return int(round(q))


def load_contract(path):
    c = json.loads(Path(path).read_text())
    if c.get("schema") != "crucible-contract/2":
        raise ValueError("Unsupported contract schema")
    dt = c["control_dt"]
    if not (0 < c["physics_dt"] <= dt):
        raise ValueError("Invalid timestep")
    ticks(dt, c["physics_dt"])
    ticks(c["horizon_s"], dt)
    ticks(c["event_s"], dt)
    for name, m in c["models"].items():
        if (name, m["policy"]) not in (("rover", "STOP"), ("multirotor", "HOLD")):
            raise ValueError("Unsupported model/policy binding")
        if not 0 < m["dwell_s"] <= m["deadline_s"]:
            raise ValueError("Invalid dwell/deadline")
        ticks(m["deadline_s"], dt)
        ticks(m["dwell_s"], dt)
        if c["event_s"] + m["deadline_s"] > c["horizon_s"]:
            raise ValueError("Contract horizon cannot complete the deadline")
        for rule in m["targets"] + m["invariants"]:
            if rule["op"] not in ("le", "ge") or rule["scale"] <= 0:
                raise ValueError("Invalid bound or scale")
            if not all(math.isfinite(rule[k]) for k in ("bound", "scale")):
                raise ValueError("Nonfinite contract")
    return c


def margin(row, rule):
    v = row.get(rule["signal"])
    if type(v) not in (float, int) or not math.isfinite(v):
        return None
    d = rule["bound"] - v if rule["op"] == "le" else v - rule["bound"]
    return d / rule["scale"]


def indexed(rows, dt, horizon):
    grid, errors = {}, []
    last = -1
    for i, row in enumerate(rows):
        try:
            if type(row.get("t")) not in (int, float):
                raise ValueError("Invalid timestamp type")
            k = ticks(row["t"], dt)
        except (KeyError, TypeError, ValueError, OverflowError):
            errors.append(f"bad timestamp at row {i}")
            continue
        if k < 0 or k > horizon or k <= last or k in grid:
            errors.append(f"duplicate/out-of-order/out-of-range row {i}")
        # Keep even invalidly ordered observations separately for breach detection.
        if 0 <= k <= horizon and k not in grid:
            grid[k] = row
        last = k
    if set(grid) != set(range(horizon + 1)):
        errors.append("incomplete time grid")
    return grid, errors


def evaluate(rows, contract, model):
    spec = contract["models"][model]
    dt = contract["control_dt"]
    h = ticks(contract["horizon_s"], dt)
    grid, errors = indexed(rows, dt, h)
    invariants = []
    for rule in spec["invariants"]:
        observed = [(r.get("t"), margin(r, rule)) for r in rows]
        known = [(t, m) for t, m in observed if m is not None]
        complete = not errors and all(margin(grid[k], rule) is not None for k in range(h + 1))
        breach = next(((t, m) for t, m in known if m < 0), None)
        invariants.append({
            "rule": rule,
            "status": "VIOLATED" if breach else "SAT" if complete else "INCONCLUSIVE",
            "observed_margin": min((m for _, m in known), default=None),
            "first_breach_s": breach[0] if breach else None,
        })
    event_values_complete = not errors and all(type(grid[k].get("operator_link_lost")) is bool for k in range(h + 1))
    triggers = [k for k in sorted(grid) if grid[k].get("operator_link_lost") is True]
    recovery = {"status": "INCONCLUSIVE", "completion_s": None, "response_s": None,
                "robustness": None, "trigger_s": None}
    if not triggers:
        if event_values_complete:
            recovery["status"] = "NOT_EXERCISED"
    else:
        event = triggers[0]
        recovery["trigger_s"] = event * dt
        # The trigger must be observed on a complete prefix. Later missing data
        # cannot erase a self-contained valid recovery witness.
        prefix_ok = not errors or all(k in grid for k in range(event + 1))
        prefix_ok = prefix_ok and all(type(grid[k].get("operator_link_lost")) is bool for k in range(event + 1))
        deadline = event + ticks(spec["deadline_s"], dt)
        dwell = ticks(spec["dwell_s"], dt)
        point = {}
        for k in range(event, min(deadline, h) + 1):
            vals = [margin(grid.get(k, {}), r) for r in spec["targets"]]
            point[k] = min(vals) if all(v is not None for v in vals) else None
        windows = []
        for end in range(event + dwell, min(deadline, h) + 1):
            vals = [point[k] for k in range(end - dwell, end + 1)]
            if all(v is not None for v in vals):
                windows.append((end, min(vals)))
        recovery["robustness"] = max((v for _, v in windows), default=None)
        witness = next((end for end, v in windows if v >= 0), None)
        complete = prefix_ok and deadline <= h and all(v is not None for v in point.values()) and len(point) == deadline - event + 1
        if prefix_ok and witness is not None:
            recovery.update(status="SAT", completion_s=witness * dt, response_s=(witness - event) * dt)
        elif complete:
            recovery["status"] = "VIOLATED"
    statuses = [r["status"] for r in invariants] + [recovery["status"]]
    if "VIOLATED" in statuses:
        overall = "VIOLATED"
    elif "INCONCLUSIVE" in statuses or errors:
        overall = "INCONCLUSIVE"
    elif recovery["status"] == "NOT_EXERCISED":
        overall = "NOT_EXERCISED"
    else:
        overall = "SAT"
    return {"overall": overall, "recovery": recovery, "invariants": invariants,
            "telemetry_errors": errors, "sample_count": len(rows)}


def boolean_oracle(rows, contract, model):
    """Independent direct predicate implementation; no margins/window helper.

    It deliberately repeats indexing logic instead of delegating to evaluate.
    Agreement is cross-check evidence, not proof of either implementation.
    """
    dt = contract["control_dt"]
    h = round(contract["horizon_s"] / dt)
    spec = contract["models"][model]
    points, malformed = {}, False
    previous = -1
    for row in rows:
        t = row.get("t")
        if type(t) not in (int, float) or not math.isfinite(t):
            malformed = True
            continue
        k = round(t / dt)
        if abs(k * dt - t) > 1e-8:
            malformed = True
            continue
        if k <= previous or k < 0 or k > h:
            malformed = True
        if 0 <= k <= h and k not in points:
            points[k] = row
        previous = k

    def pred(row, rule):
        x = row.get(rule["signal"])
        if type(x) not in (int, float) or not math.isfinite(x):
            return None
        if rule["op"] == "le":
            return x <= rule["bound"]
        return x >= rule["bound"]

    all_present = len(points) == h + 1 and not malformed
    safety = []
    for rule in spec["invariants"]:
        values = [pred(row, rule) for row in rows]
        safety.append("VIOLATED" if False in values else "SAT" if all_present and None not in values else "INCONCLUSIVE")
    trigger = None
    for k in sorted(points):
        if points[k].get("operator_link_lost") is True:
            trigger = k
            break
    rec = "INCONCLUSIVE"
    if trigger is None:
        if all_present and all(type(r.get("operator_link_lost")) is bool for r in points.values()):
            rec = "NOT_EXERCISED"
    else:
        prefix = all(k in points and type(points[k].get("operator_link_lost")) is bool for k in range(trigger + 1))
        end_limit = trigger + round(spec["deadline_s"] / dt)
        width = round(spec["dwell_s"] / dt)
        good, complete = False, prefix and end_limit <= h
        for k in range(trigger, min(end_limit, h) + 1):
            if k not in points or any(pred(points[k], r) is None for r in spec["targets"]):
                complete = False
        for start in range(trigger, min(end_limit, h) - width + 1):
            window_ok = True
            for k in range(start, start + width + 1):
                if k not in points:
                    window_ok = False
                    break
                for rule in spec["targets"]:
                    if pred(points[k], rule) is not True:
                        window_ok = False
            if window_ok and prefix:
                good = True
                break
        rec = "SAT" if good else "VIOLATED" if complete else "INCONCLUSIVE"
    statuses = safety + [rec]
    overall = "VIOLATED" if "VIOLATED" in statuses else "INCONCLUSIVE" if "INCONCLUSIVE" in statuses or malformed or not all_present else "NOT_EXERCISED" if rec == "NOT_EXERCISED" else "SAT"
    return {"overall": overall, "recovery": rec, "invariants": safety}
