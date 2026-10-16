"""Budgeted search, greedy reduction, held-out paired evaluation and controls.

No automatic controller code diagnosis/repair is implemented. Ablation measures
scenario sensitivity only. Search is compared against equal-budget random search.
"""
from __future__ import annotations

import copy
import dataclasses
import math
import random
import time
from pathlib import Path

from .evidence import compact, compare, read_json, replay, statistics, write_json
from .runtime import ROOT, VARIANTS, Scenario, digest, environment, simulate, source_manifest

PRIMES = [2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37]


def halton(index, base):
    value, weight = 0.0, 1.0
    while index:
        index, digit = divmod(index, base)
        weight /= base
        value += digit * weight
    return value


def interpolate(bounds, u):
    return bounds[0] + (bounds[1] - bounds[0]) * u


def scenario_from_unit(model, coordinates, contract, seed, name):
    cfg, u = contract["campaign"], coordinates
    magnitude = interpolate(cfg["wind_mps"], u[2])
    angle = 2 * math.pi * u[3]
    return Scenario(
        model=model, seed=seed, name=name, event_s=contract["event_s"],
        nav_start=cfg["fault_onset_s"],
        nav_duration=round(interpolate(cfg["nav_duration_s"], u[0]) / contract["control_dt"]) * contract["control_dt"],
        nav_kind=cfg["nav_kinds"][min(len(cfg["nav_kinds"]) - 1, int(u[8] * len(cfg["nav_kinds"])))],
        nav_bias=interpolate(cfg["bias_m"], u[7]),
        authority=interpolate(cfg["authority"], u[1]),
        actuator_index=min((2 if model == "rover" else 4) - 1, int(u[9] * (2 if model == "rover" else 4))),
        actuator_start=cfg["fault_onset_s"], actuator_end=cfg["actuator_fault_end_s"],
        wind=magnitude * math.cos(angle), wind_y=magnitude * math.sin(angle),
        wind_start=cfg["wind_onset_s"], wind_end=cfg["wind_end_s"],
        friction=interpolate(cfg["friction"], u[4]) if model == "rover" else .9,
        latency=interpolate(cfg["latency_s"], u[5]),
        lag_scale=interpolate(cfg["motor_lag_scale"], u[6]),
        imu_bias=(u[10] * 2 - 1) * .06, initial_yaw=(u[11] * 2 - 1) * .08,
    )


def fault_cost(s):
    """Declared dimensionless editorial severity score, not physical damage.

    Excludes the fixed operator-loss trigger and seed. Each term is normalized
    by an illustrative campaign scale. Never compare costs between vehicles.
    """
    nav = s.nav_duration / 2.4
    if s.nav_kind == "bias":
        nav *= abs(s.nav_bias) / 2.5
    return (nav + (1 - s.authority) / .55 + math.hypot(s.wind, s.wind_y) / 5
            + max(0, .9 - s.friction) / .65 + max(0, s.latency - .04) / .2
            + max(0, s.lag_scale - 1) / 1.5 + abs(s.imu_bias) / .06
            + abs(s.initial_yaw) / .08)


def attempt(s, variant, contract):
    try:
        return simulate(s, variant, contract)
    except (ValueError, FloatingPointError, OverflowError) as exc:
        # Failed executions consume budget and stay in the denominator ledger;
        # they cannot be silently discarded or represented as passes.
        return {"scenario": dataclasses.asdict(s), "controller": variant,
                "verdict": {"overall": "INCONCLUSIVE", "recovery": {"status": "INCONCLUSIVE", "robustness": None}},
                "error": type(exc).__name__ + ": " + str(exc)}


def search(model, contract):
    cfg = contract["campaign"]
    outputs, best = {}, None
    for method in ("random", "shifted_halton"):
        rng = random.Random(cfg["search_seed"])
        shifts = [rng.random() for _ in PRIMES]
        ledger = []
        for i in range(cfg["search_budget_per_method"]):
            u = [rng.random() for _ in PRIMES] if method == "random" else [(halton(i + 1, p) + shift) % 1 for p, shift in zip(PRIMES, shifts)]
            s = scenario_from_unit(model, u, contract, 4101, f"search-{method}-{i:02d}")
            run = attempt(s, "legacy_nav_gated", contract)
            score = run["verdict"]["recovery"]["robustness"]
            if run["verdict"]["recovery"]["status"] not in ("SAT", "VIOLATED") or run["verdict"]["overall"] == "INCONCLUSIVE":
                score = None
            ledger.append({"attempt": i + 1, "score": score, **compact(run)})
            if score is not None and (best is None or score < best[0]):
                best = (score, run)
        scores = [x["score"] for x in ledger if x["score"] is not None]
        outputs[method] = {"budget": cfg["search_budget_per_method"], "attempts": ledger,
                           "eligible": len(scores), "best_score": min(scores, default=None),
                           "recovery_violations": sum(x["verdict"]["recovery"]["status"] == "VIOLATED" for x in ledger)}
    return outputs, best[1] if best else None


def reduce_case(original, contract):
    cfg = contract["campaign"]
    seeds = cfg["confirmation_seeds"]
    budget = cfg["reduction_simulation_budget"]
    selected = Scenario(**original["scenario"])
    ledger, used = [], 0

    def confirm(s, label):
        nonlocal used
        # Reserve one run to materialize the retained witness.
        if used + len(seeds) > budget - 1:
            return False
        results = []
        for seed in seeds:
            run = attempt(dataclasses.replace(s, seed=seed), "legacy_nav_gated", contract)
            used += 1
            results.append({"seed": seed, "trace_hash": run.get("trace_hash"), "verdict": run["verdict"]})
        preserve = all(r["verdict"]["recovery"]["status"] == "VIOLATED" for r in results)
        ledger.append({"operation": label, "cost": fault_cost(s), "scenario": dataclasses.asdict(s),
                       "preserves_recovery_violation_all_seeds": preserve, "runs": results})
        return preserve

    if original["verdict"]["recovery"]["status"] != "VIOLATED" or not confirm(selected, "confirm-original"):
        return {"status": "NOT_REDUCED", "simulation_budget": budget, "simulations": used,
                "reason": "Original recovery violation did not survive all confirmation seeds", "ledger": ledger}, original
    initial = fault_cost(selected)
    # Deletion then binary weakening. Fixed order, no optimality claim.
    groups = [("navigation", {"nav_duration": 0.0}), ("actuator", {"authority": 1.0}),
              ("wind", {"wind": 0.0, "wind_y": 0.0}), ("friction", {"friction": .9}),
              ("latency", {"latency": .04}), ("lag", {"lag_scale": 1.0}),
              ("imu-bias", {"imu_bias": 0.0}), ("initial-yaw", {"initial_yaw": 0.0})]
    for label, change in groups:
        candidate = dataclasses.replace(selected, **change)
        if fault_cost(candidate) < fault_cost(selected) - 1e-10 and confirm(candidate, "delete-" + label):
            selected = candidate
    for round_number in range(2):
        for label, baseline in groups:
            changes = {key: (getattr(selected, key) + value) / 2 for key, value in baseline.items()}
            if "nav_duration" in changes:
                changes["nav_duration"] = round(changes["nav_duration"] / .02) * .02
            candidate = dataclasses.replace(selected, **changes)
            if used + len(seeds) > budget - 1:
                break
            if fault_cost(candidate) < fault_cost(selected) - 1e-10 and confirm(candidate, f"weaken-{label}-{round_number}"):
                selected = candidate
    result = simulate(selected, "legacy_nav_gated", contract)
    # This final materialization consumes a simulation and is included explicitly.
    used += 1
    return {"status": "REDUCED", "simulation_budget": budget, "simulations": used,
            "initial_cost": initial, "final_cost": fault_cost(selected),
            "fraction_reduced": 1 - fault_cost(selected) / initial if initial else 0,
            "confirmation_seeds": seeds, "selected_property": "recovery VIOLATED",
            "optimality": "Greedy budget-bounded local reduction only; not a minimum or diagnosis.", "ledger": ledger}, result


def ablate(run, contract):
    s = Scenario(**run["scenario"])
    groups = [("navigation", {"nav_duration": 0.0}), ("actuator", {"authority": 1.0}),
              ("wind", {"wind": 0.0, "wind_y": 0.0}), ("latency", {"latency": .04}),
              ("lag", {"lag_scale": 1.0}), ("friction", {"friction": .9})]
    return {name: compact(simulate(dataclasses.replace(s, **changes), "legacy_nav_gated", contract)) for name, changes in groups}


def sensitivity(run, contract):
    finer = simulate(Scenario(**run["scenario"]), run["controller"], contract, physics_dt=contract["physics_dt"] / 2)
    max_speed = max(abs(a["speed"] - b["speed"]) for a, b in zip(run["rows"], finer["rows"]))
    max_z = max(abs(a["z"] - b["z"]) for a, b in zip(run["rows"], finer["rows"]))
    return {"coarse_dt_s": contract["physics_dt"], "fine_dt_s": contract["physics_dt"] / 2,
            "coarse_verdict": run["verdict"], "fine_verdict": finer["verdict"],
            "max_speed_difference_mps": max_speed, "max_altitude_difference_m": max_z,
            "overall_changed": run["verdict"]["overall"] != finer["verdict"]["overall"],
            "meaning": "One-case timestep sensitivity, not continuous-time safety or a convergence proof."}


def run_campaign(contract, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    freeze_path = output / "FREEZE.json"
    if freeze_path.exists():
        raise ValueError("Output already frozen. Use a new output folder to avoid overwriting recorded evidence.")
    manifest = source_manifest()
    freeze = {"schema": "crucible-freeze/2", "created_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
              "source_manifest": manifest, "source_hash": digest(manifest), "contract_hash": digest(contract),
              "contract": contract, "environment": environment(),
              "development_disclosure": "Nominal, long-dropout and severe combined scenarios were inspected while developing models/controllers. These are development examples. Held-out seeds were not inspected/tuned. This is an internal pre-campaign freeze, not third-party preregistration."}
    write_json(freeze_path, freeze)
    summary = {"version": "0.2.0", "freeze": {k: v for k, v in freeze.items() if k != "contract"}, "models": {}}
    all_oracle = True
    for model in contract["models"]:
        print(f"{model}: nominal and budgeted searches", flush=True)
        model_dir = output / model
        nominal = {}
        for variant in VARIANTS:
            r = simulate(Scenario(model, event_s=contract["event_s"]), variant, contract)
            write_json(model_dir / f"nominal-{variant}.json.gz", r)
            nominal[variant] = compact(r)
        searches, original = search(model, contract)
        write_json(model_dir / "search.json", searches)
        if original is None:
            raise RuntimeError("Search produced no eligible evidence")
        reduction, reduced = reduce_case(original, contract)
        write_json(model_dir / "reduction.json", reduction)
        write_json(model_dir / "original.json.gz", original)
        write_json(model_dir / "reduced.json.gz", reduced)
        guarded = simulate(Scenario(**reduced["scenario"]), "guarded_failsafe", contract)
        write_json(model_dir / "guarded-on-reduced.json.gz", guarded)
        ablation = ablate(reduced, contract)
        write_json(model_dir / "ablation.json", ablation)
        print(f"{model}: held-out paired campaign", flush=True)
        # Separate held-out PRNG. It is first instantiated after the freeze and
        # never used by development/search/reduction.
        rng = random.Random(contract["campaign"]["heldout_seed"] + (0 if model == "rover" else 10000))
        heldout = {v: [] for v in VARIANTS}
        for i in range(contract["campaign"]["heldout_count_per_model"]):
            s = scenario_from_unit(model, [rng.random() for _ in PRIMES], contract, 870010 + i, f"heldout-{i:03d}")
            for variant in VARIANTS:
                run = attempt(s, variant, contract)
                write_json(model_dir / "heldout" / f"{i:03d}-{variant}.json.gz", run)
                heldout[variant].append(compact(run))
                all_oracle = all_oracle and run.get("oracle_agreement", False)
            if (i + 1) % 10 == 0:
                print(f"{model}: {i + 1} / {contract['campaign']['heldout_count_per_model']} paired scenarios", flush=True)
        write_json(model_dir / "heldout-index.json", heldout)
        stats = {variant: statistics(runs) for variant, runs in heldout.items()}
        paired = {"both_sat": 0, "legacy_only_violated": 0, "guarded_only_violated": 0,
                  "both_violated": 0, "other": 0}
        for a, b in zip(*heldout.values()):
            sa, sb = a["verdict"]["overall"], b["verdict"]["overall"]
            key = "both_sat" if sa == sb == "SAT" else "both_violated" if sa == sb == "VIOLATED" else "legacy_only_violated" if sa == "VIOLATED" and sb == "SAT" else "guarded_only_violated" if sa == "SAT" and sb == "VIOLATED" else "other"
            paired[key] += 1
        # Checkpoint includes the full MuJoCo integration state/warmstart plus
        # estimator, RNG, delayed-message queue, mode and prefix evidence.
        ck_run = simulate(Scenario(**reduced["scenario"]), "legacy_nav_gated", contract, checkpoint_at=137)
        checkpoint = ck_run.pop("checkpoint")
        write_json(model_dir / "checkpoint.json.gz", checkpoint)
        checkpoint = read_json(model_dir / "checkpoint.json.gz")
        resumed = simulate(Scenario(**checkpoint["scenario"]), checkpoint["controller"], contract,
                           physics_dt=checkpoint["physics_dt"], resume_state=checkpoint)
        perturb = simulate(Scenario(**reduced["scenario"]), "legacy_nav_gated", contract, perturb_at=140)
        incompatible = copy.deepcopy(reduced)
        incompatible["source_hash"] = "negative-control-intentional-source-mismatch"
        replay_results = {"reduced_saved_file": replay(read_json(model_dir / "reduced.json.gz"), contract),
                          "guarded_saved_file": replay(read_json(model_dir / "guarded-on-reduced.json.gz"), contract),
                          "checkpoint_resume": compare(ck_run, resumed),
                          "perturbed_state_negative_control": compare(reduced, perturb),
                          "incompatible_source_negative_control": replay(incompatible, contract)}
        write_json(model_dir / "replay.json", replay_results)
        grid_check = sensitivity(reduced, contract)
        write_json(model_dir / "timestep-sensitivity.json", grid_check)
        no_event = simulate(Scenario(model, event_s=None), "guarded_failsafe", contract)
        short = simulate(Scenario(model, nav_duration=2.4), "legacy_nav_gated", contract, horizon_s=3.0)
        write_json(model_dir / "not-exercised.json.gz", no_event)
        write_json(model_dir / "inconclusive.json.gz", short)
        # Retain a simple controlled seeded-defect comparison independent of
        # the search-selected combination; this is not held-out performance.
        illustrative = {}
        for variant in VARIANTS:
            r = simulate(Scenario(model, name="development-long-outage", nav_duration=2.3), variant, contract)
            write_json(model_dir / f"isolated-outage-{variant}.json.gz", r)
            illustrative[variant] = compact(r)
        summary["models"][model] = {"nominal": nominal, "search": {k: {kk: vv for kk, vv in v.items() if kk != "attempts"} for k, v in searches.items()},
                                    "reduction": {k: v for k, v in reduction.items() if k != "ledger"},
                                    "original": compact(original), "reduced": compact(reduced),
                                    "guarded_on_reduced": compact(guarded), "heldout": stats, "paired": paired,
                                    "replay": replay_results, "timestep_sensitivity": grid_check,
                                    "isolated_outage": illustrative,
                                    "not_exercised_status": no_event["verdict"]["overall"], "short_trace_status": short["verdict"]["overall"]}
        write_json(output / "summary.partial.json", summary)
    if source_manifest() != manifest:
        raise RuntimeError("Source changed after campaign freeze")
    summary["heldout_oracle_agreement_all"] = all_oracle
    write_json(output / "summary.json", summary)
    (output / "summary.partial.json").unlink(missing_ok=True)
    print("Campaign complete; source and contract stayed frozen.", flush=True)
    return summary
