# CRUCIBLE — autonomy testing demonstrator

**Latest: v0.2, physics and evidence.** MuJoCo rigid-body models, a differential-drive rover with wheel contact, a free six-DOF quadrotor, noisy delayed sensors, wind, actuator limits, mission contracts and replay controls.

![Recorded CRUCIBLE v0.2 results](CRUCIBLE-v0.2.png)

**[Download v0.2: runnable source, all recorded traces and offline explorer](CRUCIBLE-v0.2.zip)** · [Inspect v0.2 source and methods](v0.2/README.md) · [Download the standalone evidence explorer](CRUCIBLE-v0.2-Explorer.html)

Download the HTML and open it locally; GitHub displays its source rather than running it. The archive contains all 160 held-out traces and the source/environment freeze. `v0.2/` provides directly inspectable source and summary records; extract the archive for complete saved-file replay inputs.

## Validation by snapshot

| v0.2 source or artifact | Checks passed | What the count covers |
| --- | ---: | --- |
| **Current source** in `v0.2/` | **37** | Both the replay-integrity and checkpoint-timestep patches. [Current validation log](v0.2/validation/checkpoint-timestep.json) |
| Earlier replay-integrity patch | 36 | Added two checks for trace checksums and matching verdicts. [Replay patch log](v0.2/validation/replay-integrity.json) |
| Historical v0.2 ZIP | 34 | Original source and its saved test run. [Release validation log](v0.2/VALIDATION.json) |
| Offline HTML explorer | 34 | Displays the ZIP’s saved results, including the original validation count and the 160 held-out runs. |

## Original v0.2 campaign results

| Held-out contract violations | Legacy controller | Guarded controller |
| --- | ---: | ---: |
| Rover | 8/40 | 0/40 |
| Quadrotor | 31/40 | 27/40 |

All 160 held-out evaluations agreed with the independent Boolean oracle. Six saved-file/checkpoint replay checks matched exactly, and four deliberate state/source changes were detected. Quadrotor pairs include five improvements **and one regression**. The guarded quadrotor still fails 27 cases. Zero rover violations in 40 cases does not establish zero risk; its 95% Wilson upper bound is 8.8% under the declared synthetic distribution.

**Scope:** uncalibrated simulation demonstrators and manually authored controllers, including a deliberately defective legacy baseline. No production autopilot, hardware validity, automatic defect diagnosis/repair, flight readiness, or field reliability is established. The independent oracle and exact replay can share modeling/specification mistakes. See the [full methods and limitations](v0.2/README.md).

## Replay integrity patch

The replay-integrity patch added verification of both trace checksums and required equal verdicts before reporting an EXACT replay. Previously, a copied checksum or a changed verdict could incorrectly pass. The two new regression tests brought that revision to [36 passing checks](v0.2/validation/replay-integrity.json).

The ZIP contains the source and traces for the 160 held-out runs shown in the explorer. Replaying those traces with patched source reports INCOMPATIBLE because the source hash differs. Use the original ZIP to reproduce that campaign or generate a fresh campaign with the patched source.

To validate the patched checkout without replacing the historical validation record:

```bash
cd v0.2
python -m pip install -r requirements.txt
python -m crucible validate --output validation-local.json
```

## Checkpoint timestep compatibility patch

Checkpoint restore now rejects a physics timestep that differs from the saved checkpoint. Previously, the Python API silently resumed under a changed integration step despite the matching source, contract and environment. A regression test reproduces that mismatch; **37 checks pass** in the [current validation record](v0.2/validation/checkpoint-timestep.json), including exact checkpoint round trips for both models. This is simulation validation only. The historical archive, explorer and campaign results are unchanged and have not been recertified.

## Run v0.2

Extract `CRUCIBLE-v0.2.zip`, enter the extracted folder, and use Python 3.12:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m crucible validate --output validation-local.json
python -m crucible replay results/rover/reduced.json.gz
python -m crucible campaign --output results-new
```

Exact replay is checked only for the recorded source, contract and environment. A fresh campaign refuses to overwrite an existing freeze. Mission thresholds and fault ranges are configurable in `contracts/mission.json`; only implemented STOP/HOLD policy bindings are supported.

## Earlier release

[v0.1 source/methods](v0.1/README.md) · [Complete v0.1 archive](CRUCIBLE-Pilot-v0.1.zip) · [v0.1 overview](CRUCIBLE-Pilot-v0.1.png).

The original v0.1 Python files and validation record remain at the repository root. They are the earlier standard-library workflow pilot, not the MuJoCo v0.2 implementation. Its original claims and evidence are preserved for comparison.
