#!/usr/bin/env python3
"""Run actual pilot checks and save their observed outcomes; standard library only."""
import json
from pathlib import Path
import re
import subprocess
import sys


def execute(arguments):
    run = subprocess.run([sys.executable] + arguments, capture_output=True, text=True, cwd=Path(__file__).parent)
    print(run.stdout, end="")
    print(run.stderr, end="")
    if run.returncode:
        raise SystemExit(run.returncode)
    return run


def main():
    tests = execute(["-m", "unittest", "-v"])
    count = int(re.search(r"Ran (\d+) tests?", tests.stderr).group(1))
    run = execute(["crucible_pilot.py", "run", "--out", "results"])
    saved_file_replays = {}
    for machine in ("rover", "multirotor"):
        replay = execute(["crucible_pilot.py", "replay", f"results/{machine}/reduced.json"])
        result = json.loads(replay.stdout)
        if result["result"]["matching_traces"] != 3:
            raise SystemExit("Saved-file replay did not match")
        saved_file_replays[machine] = result
    validation = {"scope": "Synthetic pilot; no hardware or real autonomy stack tested.",
                  "command": "python -m unittest -v", "tests_run": count, "tests_passed": count,
                  "campaign_command": "python crucible_pilot.py run --out results", "campaign_exit_code": run.returncode,
                  "models": ["rover_longitudinal_v1", "multirotor_pitch_altitude_v1"],
                  "reduced_case_replays_per_model": 3, "exact_matching_replay_traces": 6,
                  "saved_file_replay_commands": saved_file_replays}
    (Path(__file__).parent / "VALIDATION.json").write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
