#!/usr/bin/env python3
"""CRUCIBLE 0.1: deterministic, deliberately simplified autonomy test pilot.

Standard library only. No simulator, real controller, or hardware connection.
The monitors, search, reducer and replay are shared by both machine adapters.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
import platform
import random
import sys
from dataclasses import asdict, dataclass, replace
from pathlib import Path

VERSION = "0.1.0"
DT = 0.02
LOSS_START = 100  # 2.00 s; all event timing is in integer simulation ticks.
HORIZON = 400
TARGET_RULE = "operator_loss_response"
SCOPE = (
    "Executable workflow demonstrator using synthetic dynamics and intentionally "
    "defective demonstration controllers. No physical validation, real autonomy "
    "stack, Gazebo/PX4 integration, certification, or general safety guarantee."
)


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def digest(value):
    return hashlib.sha256(canonical(value).encode()).hexdigest()


def clamp(value, low, high):
    return max(low, min(high, value))


@dataclass(frozen=True)
class Fault:
    kind: str
    start: int
    duration: int
    value: float = 1.0
    fixed: bool = False

    def active(self, tick):
        return self.start <= tick < self.start + self.duration


PLATFORMS = {
    "rover": {
        "model": "rover_longitudinal_v1", "adapter": "rover_si_v1",
        "initial_speed_mps": 1.5, "initial_altitude_m": 0.0,
        "target_speed_mps": 0.10, "deadline_s": 1.40,
        "speed_limit_mps": 2.0, "altitude_error_limit_m": 0.05,
        "target_altitude_error_m": 0.05,
        "model_parameters": {"max_acceleration_mps2": 3.0, "acceleration_lag_s": 0.08},
        "policy_action": "stop", "observation_frame": "ground-track x; z=0",
    },
    "multirotor": {
        "model": "multirotor_pitch_altitude_v1", "adapter": "multirotor_si_v1",
        "initial_speed_mps": 2.0, "initial_altitude_m": 1.5,
        "target_speed_mps": 0.15, "deadline_s": 1.40,
        "speed_limit_mps": 2.5, "altitude_error_limit_m": 0.35,
        "target_altitude_error_m": 0.25,
        "model_parameters": {"mass_kg": 1.5, "gravity_mps2": 9.81,
                             "pitch_lag_s": 0.18, "max_pitch_deg": 25.0,
                             "horizontal_drag_per_s": 0.18},
        "policy_action": "hover", "observation_frame": "level-world x; z positive up",
    },
}

CONTRACT = {
    "time": "integer ticks; dt_s=0.02; sample then command then integrate",
    "fault_intervals": "start-inclusive, end-exclusive; no sub-tick event claims",
    "observation_units": {"x_m": "m", "z_m": "m", "vx_mps": "m/s",
                          "vz_mps": "m/s", "pitch_rad": "rad"},
    "monitor_source": "privileged simulator truth, independent of faulted navigation",
    "operator_link": "external operator link; independent of onboard navigation freshness",
    "navigation_freeze": "hold x and vx samples and their original tick; altitude/attitude remain fresh",
    "braking_derate": "backend model hook: scale negative rover acceleration or opposing multirotor pitch demand",
    "replay_comparison": "exact SHA-256 of canonical JSON trace in the recorded environment",
}


class RoverModel:
    """One-dimensional ground motion, acceleration lag, no collision/terrain model."""
    def __init__(self, config):
        self.config = config
        self.x = 0.0
        self.vx = config["initial_speed_mps"]
        self.acceleration = 0.0

    def observe(self):
        return {"x_m": self.x, "z_m": 0.0, "vx_mps": self.vx,
                "vz_mps": 0.0, "pitch_rad": 0.0}

    def step(self, command, authority):
        p = self.config["model_parameters"]
        requested = command["acceleration_mps2"]
        if requested < 0:
            requested *= authority
        self.acceleration += (requested - self.acceleration) * DT / p["acceleration_lag_s"]
        previous = self.vx
        self.vx = max(0.0, self.vx + DT * self.acceleration)
        self.x += DT * (previous + self.vx) / 2


class MultirotorModel:
    """Pitch plus x/z translation; no roll, yaw, rotor, wind or aerodynamics fidelity."""
    def __init__(self, config):
        self.config = config
        self.x, self.z = 0.0, config["initial_altitude_m"]
        self.vx, self.vz = config["initial_speed_mps"], 0.0
        p = config["model_parameters"]
        self.pitch = math.atan(p["horizontal_drag_per_s"] * self.vx / p["gravity_mps2"])

    def observe(self):
        return {"x_m": self.x, "z_m": self.z, "vx_mps": self.vx,
                "vz_mps": self.vz, "pitch_rad": self.pitch}

    def step(self, command, authority):
        p = self.config["model_parameters"]
        demanded_pitch = command["pitch_rad"]
        if demanded_pitch * self.vx < 0:
            demanded_pitch *= authority
        self.pitch += (demanded_pitch - self.pitch) * DT / p["pitch_lag_s"]
        thrust_acceleration = command["thrust_N"] / p["mass_kg"]
        ax = thrust_acceleration * math.sin(self.pitch) - p["horizontal_drag_per_s"] * self.vx
        az = thrust_acceleration * math.cos(self.pitch) - p["gravity_mps2"]
        old_vx, old_vz = self.vx, self.vz
        self.vx += ax * DT
        self.vz += az * DT
        self.x += (old_vx + self.vx) * DT / 2
        self.z += (old_vz + self.vz) * DT / 2


class Adapter:
    def __init__(self, config):
        self.config = config
        self.last_nav = None

    def observation(self, truth, tick, operator_link, nav_frozen):
        observation = dict(truth)
        if self.last_nav is None or not nav_frozen:
            self.last_nav = {"x_m": truth["x_m"], "vx_mps": truth["vx_mps"], "nav_tick": tick}
        observation.update(self.last_nav)
        observation.update(operator_link=operator_link, nav_fresh=not nav_frozen)
        return observation

    def command(self, desired_speed, observation):
        v = observation["vx_mps"]
        braking = desired_speed == 0.0 and abs(v) > 0.05
        if self.config["model"] == "rover_longitudinal_v1":
            limit = self.config["model_parameters"]["max_acceleration_mps2"]
            acceleration = -limit if braking else clamp(3.0 * (desired_speed - v), -limit, limit)
            return {"acceleration_mps2": acceleration}
        p = self.config["model_parameters"]
        limit = math.radians(p["max_pitch_deg"])
        if braking:
            pitch = -math.copysign(limit, v)
        else:
            acceleration = 3.0 * (desired_speed - v) + p["horizontal_drag_per_s"] * desired_speed
            pitch = clamp(math.atan(acceleration / p["gravity_mps2"]), -limit, limit)
        vertical_correction = (6.0 * (self.config["initial_altitude_m"] - observation["z_m"])
                               - 4.0 * observation["vz_mps"])
        thrust = p["mass_kg"] * max(0, p["gravity_mps2"] + vertical_correction) / max(0.5, math.cos(observation["pitch_rad"]))
        return {"pitch_rad": pitch, "thrust_N": thrust}


class DemonstrationController:
    """Seeded defect: link handling is incorrectly conditional on fresh navigation."""
    def __init__(self, config, repaired=False):
        self.config, self.repaired = config, repaired
        self.link_up = True

    def step(self, observation):
        if self.repaired or observation["nav_fresh"]:
            self.link_up = observation["operator_link"]
        speed = self.config["initial_speed_mps"] if self.link_up else 0.0
        return speed, "cruise" if self.link_up else self.config["policy_action"]


def ticks(seconds):
    result = round(seconds / DT)
    if not math.isclose(result * DT, seconds, abs_tol=1e-10):
        raise ValueError("Policy timing must align with the declared sampling grid")
    return result


def validate_config(machine, config):
    if machine not in PLATFORMS:
        raise ValueError("Unknown machine")
    expected = PLATFORMS[machine]
    for name in ("model", "adapter", "observation_frame"):
        if config.get(name) != expected[name]:
            raise ValueError("Unregistered or incompatible model/adapter/frame")
    for name in ("initial_speed_mps", "target_speed_mps", "deadline_s", "speed_limit_mps",
                 "altitude_error_limit_m", "target_altitude_error_m"):
        if not math.isfinite(config[name]) or config[name] <= 0:
            raise ValueError("Invalid positive policy/model setting: " + name)
    if not math.isfinite(config["initial_altitude_m"]):
        raise ValueError("Invalid initial altitude")
    if any(not math.isfinite(value) or value <= 0 for value in config["model_parameters"].values()):
        raise ValueError("Invalid model parameter")
    if set(config["model_parameters"]) != set(expected["model_parameters"]):
        raise ValueError("Model parameters do not match the registered model")
    ticks(config["deadline_s"])


def validate_faults(faults):
    allowed = {"operator_link_loss", "navigation_freeze", "braking_derate"}
    for fault in faults:
        if fault.kind not in allowed or not isinstance(fault.start, int) or fault.start < 0:
            raise ValueError("Unsupported fault or invalid start tick")
        if not isinstance(fault.duration, int) or fault.duration <= 0:
            raise ValueError("Fault duration must be positive integer ticks")
        if not math.isfinite(fault.value) or (fault.kind == "braking_derate" and not 0 < fault.value <= 1):
            raise ValueError("Invalid authority multiplier")


def simulate(machine, faults, *, repaired=False, horizon=HORIZON, config=None):
    config = copy.deepcopy(config or PLATFORMS[machine])
    validate_config(machine, config)
    validate_faults(faults)
    model = RoverModel(config) if config["model"] == "rover_longitudinal_v1" else MultirotorModel(config)
    adapter = Adapter(config)
    controller = DemonstrationController(config, repaired)
    trace = []
    for tick in range(horizon + 1):
        active = [fault for fault in faults if fault.active(tick)]
        link = not any(f.kind == "operator_link_loss" for f in active)
        frozen = any(f.kind == "navigation_freeze" for f in active)
        authority = min([1.0] + [f.value for f in active if f.kind == "braking_derate"])
        truth = model.observe()
        delivered = adapter.observation(truth, tick, link, frozen)
        requested_speed, mode = controller.step(delivered)
        command = adapter.command(requested_speed, delivered)
        trace.append({"tick": tick, "t_s": tick * DT, "truth": truth,
                      "delivered": delivered, "command": command, "mode": mode,
                      "operator_link": link, "navigation_frozen": frozen,
                      "braking_authority": authority,
                      "metrics": {"horizontal_speed_mps": abs(truth["vx_mps"]),
                                  "altitude_error_m": abs(truth["z_m"] - config["initial_altitude_m"])}})
        if tick < horizon:
            model.step(command, authority)
    return trace


def validate_trace(trace):
    if not trace:
        return "empty trace"
    if [row.get("tick") for row in trace] != list(range(len(trace))):
        return "missing, duplicated, or non-monotone sample tick"
    try:
        for row in trace:
            if not math.isclose(row["t_s"], row["tick"] * DT, abs_tol=1e-10):
                return "timestamp disagrees with tick"
            if type(row["operator_link"]) is not bool:
                return "operator-link sample is not Boolean"
            for name in ("horizontal_speed_mps", "altitude_error_m"):
                if not math.isfinite(row["metrics"][name]) or row["metrics"][name] < 0:
                    return "invalid metric"
    except (KeyError, TypeError, ValueError):
        return "missing or invalid telemetry"
    return None


def deadline_windows(trace, config):
    """Inclusive reach-by predicate, evaluated separately for every observed loss."""
    start_ticks = [r["tick"] for i, r in enumerate(trace)
                   if not r["operator_link"] and (i == 0 or trace[i - 1]["operator_link"])]
    deadline = ticks(config["deadline_s"])
    windows = []
    for start in start_ticks:
        available = trace[start:]
        # Monitoring continues after link restoration: the illustrative policy is
        # "reach the target by the deadline after each loss", not a dwell policy.
        target = next((r for r in available
                       if r["metrics"]["horizontal_speed_mps"] <= config["target_speed_mps"]
                       and r["metrics"]["altitude_error_m"] <= config["target_altitude_error_m"]), None)
        latency_ticks = None if target is None else target["tick"] - start
        observed = trace[-1]["tick"] - start
        if latency_ticks is not None and latency_ticks <= deadline:
            status = "SATISFIED"
        elif observed >= deadline:
            status = "VIOLATED"
        else:
            status = "INCONCLUSIVE"
        windows.append({"trigger_tick": start, "deadline_tick": start + deadline,
                        "observed_ticks": observed, "latency_ticks": latency_ticks,
                        "latency_s": None if latency_ticks is None else latency_ticks * DT,
                        "status": status})
    return windows


def evaluate(trace, config):
    invalid = validate_trace(trace)
    if invalid:
        return {"trace_valid": False, "reason": invalid, "rules": {},
                "violations": [], "target_objective_s": None, "eligible": False}
    windows = deadline_windows(trace, config)
    statuses = [window["status"] for window in windows]
    response_status = ("VIOLATED" if "VIOLATED" in statuses else
                       "INCONCLUSIVE" if "INCONCLUSIVE" in statuses else
                       "SATISFIED" if statuses else "NOT_EXERCISED")
    rules = {TARGET_RULE: {"status": response_status, "windows": windows,
                           "deadline_s": config["deadline_s"],
                           "target_speed_mps": config["target_speed_mps"],
                           "target_altitude_error_m": config["target_altitude_error_m"]}}
    for rule_id, metric, bound in (
        ("speed_envelope", "horizontal_speed_mps", config["speed_limit_mps"]),
        ("altitude_envelope", "altitude_error_m", config["altitude_error_limit_m"]),
    ):
        # Declared scale is 1 m/s or 1 m respectively; margins are dimensionless.
        margins = [bound - row["metrics"][metric] for row in trace]
        minimum = min(margins)
        worst = margins.index(minimum)
        rules[rule_id] = {"status": "VIOLATED" if minimum < 0 else "SATISFIED",
                         "minimum_normalized_margin": minimum, "scale": 1.0,
                         "limit": bound, "metric": metric, "worst_tick": worst}
    latencies = [w["latency_s"] for w in windows]
    objective = (min(config["deadline_s"] - latency for latency in latencies)
                 if latencies and all(value is not None for value in latencies) else None)
    eligible = response_status in ("SATISFIED", "VIOLATED") and objective is not None
    return {"trace_valid": True, "reason": None, "rules": rules,
            "violations": [name for name, result in rules.items() if result["status"] == "VIOLATED"],
            "target_objective_s": objective, "eligible": eligible}


def run_case(machine, faults, *, repaired=False, horizon=HORIZON, config=None):
    config = copy.deepcopy(config or PLATFORMS[machine])
    trace = simulate(machine, faults, repaired=repaired, horizon=horizon, config=config)
    return {"machine": machine, "controller": "repaired_demo" if repaired else "nav_gated_demo",
            "config": config, "faults": [asdict(f) for f in faults], "horizon_ticks": horizon,
            "evaluation": evaluate(trace, config), "trace_sha256": digest(trace), "trace": trace}


def fault_cost(faults):
    """Declared demo cost: severity-weighted additional fault seconds; trigger fixed."""
    return sum(f.duration * DT * (1 - f.value if f.kind == "braking_derate" else 1)
               for f in faults if not f.fixed)


def fixed_trigger():
    return Fault("operator_link_loss", LOSS_START, 200, fixed=True)


def candidate_schedules(seed=1729):
    candidates = [[fixed_trigger(), Fault("navigation_freeze", 95, ticks(duration)),
                   Fault("braking_derate", LOSS_START, 150, efficiency)]
                  for duration, efficiency in itertools.product((0.2, 0.4, 0.6, 0.8, 0.9), (0.65, 0.75, 0.85))]
    random.Random(seed).shuffle(candidates)
    return candidates


def compact(case):
    result = {key: value for key, value in case.items() if key not in ("trace", "config")}
    result["fault_cost_s"] = fault_cost([Fault(**value) for value in case["faults"]])
    return result


def bounded_search(machine, *, seed=1729, budget=15, schedules=None):
    trials, best = [], None
    for schedule in (schedules if schedules is not None else candidate_schedules(seed))[:budget]:
        case = run_case(machine, schedule)
        trials.append(compact(case))  # Invalid/unexercised/inconclusive attempts consume budget.
        if case["evaluation"]["eligible"] and (best is None or
                case["evaluation"]["target_objective_s"] < best["evaluation"]["target_objective_s"]):
            best = case
    return best, trials


def reduction_moves(faults):
    for index, fault in enumerate(faults):
        if fault.fixed:
            continue
        yield "delete " + fault.kind, faults[:index] + faults[index + 1:]
        for duration in dict.fromkeys((max(1, fault.duration // 2), fault.duration - 1)):
            if 0 < duration < fault.duration:
                yield "shorten " + fault.kind, faults[:index] + [replace(fault, duration=duration)] + faults[index + 1:]
        if fault.kind == "braking_derate" and fault.value < 1:
            yield "weaken braking_derate", faults[:index] + [replace(fault, value=min(1.0, round(fault.value + 0.05, 8)))] + faults[index + 1:]


def reduce_failure(machine, original, *, budget=160):
    current = original
    attempts, accepted = [], []
    while len(attempts) < budget:
        improved = False
        faults = [Fault(**item) for item in current["faults"]]
        for operation, candidate in reduction_moves(faults):
            if len(attempts) >= budget:
                break
            if fault_cost(candidate) >= fault_cost(faults) - 1e-12:
                continue
            trial = run_case(machine, candidate)
            preserves = (trial["evaluation"]["trace_valid"] and
                         trial["evaluation"]["rules"][TARGET_RULE]["status"] == "VIOLATED")
            record = {"operation": operation, "accepted": preserves,
                      "cost_s": fault_cost(candidate), "faults": trial["faults"],
                      "evaluation": trial["evaluation"], "trace_sha256": trial["trace_sha256"]}
            attempts.append(record)
            if preserves:
                accepted.append(record)
                current, improved = trial, True
                break
        if not improved:
            break
    return current, {"budget": budget, "attempts": attempts, "accepted": accepted,
                     "original_cost_s": fault_cost([Fault(**f) for f in original["faults"]]),
                     "reduced_cost_s": fault_cost([Fault(**f) for f in current["faults"]]),
                     "budget_exhausted": len(attempts) >= budget,
                     "claim": "Lower declared cost preserving the target violation; no global minimality claim."}


def replay(case, repeats=3):
    if digest(case["trace"]) != case["trace_sha256"]:
        raise ValueError("Saved trace does not match its stored digest")
    if evaluate(case["trace"], case["config"]) != case["evaluation"]:
        raise ValueError("Saved verdict does not match the recorded trace and policy")
    hashes = []
    for _ in range(repeats):
        repeated = run_case(case["machine"], [Fault(**f) for f in case["faults"]],
                            repaired=case["controller"] == "repaired_demo",
                            horizon=case["horizon_ticks"], config=case["config"])
        hashes.append(repeated["trace_sha256"])
    return {"comparison": CONTRACT["replay_comparison"], "repeats": repeats,
            "matching_traces": sum(value == case["trace_sha256"] for value in hashes),
            "reference_sha256": case["trace_sha256"], "replay_sha256": hashes,
            "claim": "Repeatability under recorded local conditions; no cross-environment guarantee."}


def provenance():
    return {"engine_version": VERSION, "engine_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "python": sys.version, "platform": platform.platform(),
            "dt_s": DT, "integrator": "fixed-step explicit lag updates; trapezoidal position update"}


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def campaign(out):
    out.mkdir(parents=True, exist_ok=True)
    summary = {"version": VERSION, "scope": SCOPE, "contract": CONTRACT,
               "provenance": provenance(), "search_seed": 1729, "machines": {}}
    for machine in PLATFORMS:
        baseline = run_case(machine, [fixed_trigger()])
        original, trials = bounded_search(machine)
        if original is None or TARGET_RULE not in original["evaluation"]["violations"]:
            raise RuntimeError("Campaign did not find the intended demonstration failure: " + machine)
        reduced, reduction = reduce_failure(machine, original)
        repaired = run_case(machine, [Fault(**f) for f in reduced["faults"]], repaired=True)
        cases = {"baseline": baseline, "original": original, "reduced": reduced, "repaired": repaired}
        original_faults = [Fault(**f) for f in original["faults"]]
        for extra in [f for f in original_faults if not f.fixed]:
            cases["only_" + extra.kind] = run_case(machine, [fixed_trigger(), extra])
        cases["not_exercised"] = run_case(machine, [])
        cases["inconclusive"] = run_case(machine, [Fault(**f) for f in original["faults"]], horizon=LOSS_START + 10)
        replay_result = replay(reduced)
        for name, case in cases.items():
            write_json(out / machine / (name + ".json"), {"schema_version": 1, "scope": SCOPE,
                       "contract": CONTRACT, "provenance": summary["provenance"],
                       "case_sha256": digest(case), "case": case})
        write_json(out / machine / "search.json", {"budget": 15, "seed": 1729, "attempts": trials,
                   "selection": "Lowest defined deadline slack among valid, exercised, conclusive trials."})
        write_json(out / machine / "reduction.json", reduction)
        write_json(out / machine / "replay.json", replay_result)
        summary["machines"][machine] = {"cases": {name: compact(case) for name, case in cases.items()},
            "search_attempts": len(trials), "reduction": {key: value for key, value in reduction.items() if key not in ("attempts", "accepted")},
            "reduction_attempts": len(reduction["attempts"]), "accepted_reductions": len(reduction["accepted"]), "replay": replay_result}
    write_json(out / "summary.json", summary)
    for machine, result in summary["machines"].items():
        print(machine)
        for name in ("baseline", "original", "reduced", "repaired", "only_navigation_freeze", "only_braking_derate", "not_exercised", "inconclusive"):
            case = result["cases"][name]
            rule = case["evaluation"]["rules"][TARGET_RULE]
            latency = rule["windows"][0]["latency_s"] if rule["windows"] else None
            print(f"  {name:24s} {rule['status']:16s} response={latency} cost={case['fault_cost_s']:.4f}")
        print("  replay", result["replay"]["matching_traces"], "/", result["replay"]["repeats"])
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run", help="Run the two-machine demonstration campaign")
    run.add_argument("--out", type=Path, default=Path("results"))
    rerun = sub.add_parser("replay", help="Replay a saved case and compare its exact trace")
    rerun.add_argument("evidence", type=Path)
    args = parser.parse_args()
    if args.action == "run":
        campaign(args.out)
    else:
        evidence = json.loads(args.evidence.read_text(encoding="utf-8"))
        if digest(evidence["case"]) != evidence["case_sha256"]:
            raise SystemExit("Case content differs from its saved digest; replay refused.")
        compatible = evidence["provenance"]["engine_source_sha256"] == provenance()["engine_source_sha256"]
        if not compatible:
            raise SystemExit("Source differs from the saved case; replay refused. Keep the original evidence.")
        print(json.dumps({"recorded_python": evidence["provenance"]["python"],
                          "current_python": sys.version, "source_matches": compatible,
                          "recorded_platform": evidence["provenance"]["platform"],
                          "current_platform": platform.platform(),
                          "result": replay(evidence["case"])}, indent=2))


if __name__ == "__main__":
    main()
