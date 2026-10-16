from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

from .campaign import run_campaign
from .evidence import compact, read_json, replay, write_json
from .monitor import load_contract
from .runtime import ROOT, VARIANTS, Scenario, digest, environment, simulate, source_manifest


def main():
    p = argparse.ArgumentParser(description="CRUCIBLE v0.2 MuJoCo evaluation demonstrator")
    p.add_argument("--contract", default=str(ROOT / "contracts/mission.json"))
    sub = p.add_subparsers(dest="command", required=True)
    campaign = sub.add_parser("campaign", help="Freeze and execute a fresh campaign; refuses to overwrite")
    campaign.add_argument("--output", default=str(ROOT / "results"))
    run = sub.add_parser("run", help="One scenario, saved evidence and recomputed verdict")
    run.add_argument("--model", choices=("rover", "multirotor"), default="rover")
    run.add_argument("--variant", choices=VARIANTS, default="guarded_failsafe")
    run.add_argument("--scenario", help="Scenario JSON, or a previous evidence file")
    run.add_argument("--output", required=True)
    rp = sub.add_parser("replay", help="Replay compatible saved evidence, report explicit grade")
    rp.add_argument("path")
    resume = sub.add_parser("resume", help="Resume a saved JSON checkpoint")
    resume.add_argument("path")
    resume.add_argument("--output", required=True)
    validate = sub.add_parser("validate", help="Run independent monitor/physics/replay checks")
    validate.add_argument("--output", default=str(ROOT / "VALIDATION.json"))
    args = p.parse_args()
    c = load_contract(args.contract)
    if args.command == "campaign":
        result = run_campaign(c, args.output)
        print(json.dumps({m: v["heldout"] for m, v in result["models"].items()}, indent=2))
    elif args.command == "run":
        source = read_json(args.scenario) if args.scenario else {"model": args.model, "event_s": c["event_s"]}
        scenario = Scenario(**source.get("scenario", source))
        result = simulate(scenario, args.variant, c)
        write_json(args.output, result)
        print(json.dumps(compact(result), indent=2))
    elif args.command == "replay":
        result = replay(read_json(args.path), c)
        print(json.dumps(result, indent=2))
        if result["grade"] not in ("EXACT", "TOLERANT"):
            sys.exit(2)
    elif args.command == "resume":
        checkpoint = read_json(args.path)
        result = simulate(Scenario(**checkpoint["scenario"]), checkpoint["controller"], c,
                          physics_dt=checkpoint["physics_dt"], resume_state=checkpoint)
        write_json(args.output, result)
        print(json.dumps(compact(result), indent=2))
    elif args.command == "validate":
        start = time.monotonic()
        completed = subprocess.run([sys.executable, "-m", "unittest", "discover", "-s", "tests", "-v"], cwd=ROOT, capture_output=True, text=True)
        print(completed.stdout + completed.stderr)
        log = {"passed": completed.returncode == 0, "returncode": completed.returncode,
               "elapsed_s": time.monotonic() - start, "environment": environment(),
               "source_hash": digest(source_manifest()), "contract_hash": digest(c),
               "test_output": completed.stdout + completed.stderr}
        write_json(args.output, log)
        sys.exit(completed.returncode)


if __name__ == "__main__":
    main()
