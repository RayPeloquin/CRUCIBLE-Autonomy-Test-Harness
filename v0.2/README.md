# CRUCIBLE v0.2 — physics and evidence

CRUCIBLE exercises a complete fault-evaluation workflow on two MuJoCo vehicle demonstrators: define a mission rule, run fault scenarios, search for violations, reduce a counterexample, compare manually authored controllers, save evidence, and replay it with negative controls.

**This is executable simulation research, not validated autonomy, an automatic repair engine, or flight-ready software.** The quadrotor still fails many stress cases. v0.1 remains available in the parent repository and its original archive.

![Measured v0.2 results](CRUCIBLE-v0.2.png)

Open [the offline evidence explorer](CRUCIBLE-v0.2-Explorer.html) locally after downloading; all data and scripts are embedded. No server or network connection is needed. This report and chart use recorded data, not generated imagery.

## Recorded results

40 held-out scenarios per model × 2 controllers = **160 runs**. Each pair uses the same scenario and exogenous noise seed. All 160 runs were conclusive/exercised and the quantitative monitor agreed with the separately implemented boolean oracle. These fractions describe only the declared synthetic distribution.

| Model | Legacy violations | Guarded violations | Guarded 95% Wilson interval |
| --- | ---: | ---: | --- |
| rover | 8/40 | 0/40 | 0.0%–8.8% |
| multirotor | 31/40 | 27/40 | 52.0%–79.9% |

The rover has 8 improved pairs and no regressed pairs. The quadrotor has 5 improved pairs, 1 regressed pair, 26 pairs violating both and 8 pairs passing both. These small paired results do not establish statistical superiority. Zero rover failures does not establish zero risk: its interval extends to 8.8% under this distribution.

**34 checks passed**, including a 162-case expected-outcome recovery boundary sweep. Checks include independent Boolean/quantitative monitor comparisons, empty/missing/nonfinite telemetry, preserved known safety breaches, analytic free fall, hover force balance, actual wheel contact, force limits, seeded link-processing behavior, checkpoint persistence, corruption detection and accounting for failed search attempts.

6/6 replay checks were exact (two saved evidence files plus one serialized full-state checkpoint per model). All 4 negative controls were detected: state perturbation caused divergence and source mismatch was incompatible on each model. Hashes are not signatures; repeatability and agreement do not prove correctness or physical validity.

## Run it

Recorded environment: Python 3.12.14, MuJoCo 3.16.0, NumPy 2.5.4 on Linux x86_64. Other environments can run the models, but exact replay is checked only when the recorded environment/source/contract match. Install dependencies from official PyPI in an isolated environment:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python -m crucible validate
python -m crucible replay results/rover/reduced.json.gz
python -m crucible replay results/multirotor/reduced.json.gz
python -m crucible resume results/rover/checkpoint.json.gz --output resumed.json.gz
python -m crucible campaign --output results-new
```

The campaign refuses to overwrite an existing `FREEZE.json`. Use a fresh output directory. `python -m crucible run --model multirotor --variant guarded_failsafe --output run.json.gz` runs a nominal mission. Pass `--scenario` with a Scenario JSON or saved evidence file to rerun a scenario. `--contract /path/mission.json` precedes the subcommand for another compatible contract. Only supported rover STOP / quadrotor HOLD policy bindings are accepted; RETURN, LAND and CONTINUE are not implemented.

`render_report.py` optionally regenerates this README, overview and HTML from recorded `results/summary.json`; it needs Matplotlib. It is outside the frozen evaluator and makes no simulation changes. The viewer works without Python.

## What the models actually contain

| Layer | Rover | Quadrotor |
| --- | --- | --- |
| Plant | 6.76 kg free chassis, two hinged wheels, two low-friction support spheres, actual ground contact | 1.5 kg free rigid body, all 6 pose/velocity DOFs, 4 offset rotor thrust sites and yaw torques, ground contact |
| Actuation | Individual bounded wheel torque, exact first-order activation filter | Individual 0–7.5 N rotor force, exact first-order activation filter, clipped mixer |
| Disturbance | Tire-friction uncertainty; simple air-relative drag and finite wind interval | Simple quadratic air-relative drag, angular damping and finite wind interval |
| Sensors | 10 Hz GNSS with source timestamps, delayed queue, dropout/stale/bias hooks; 50 Hz simulated IMU/attitude aid and wheel odometry | Same GNSS and simulated IMU/attitude aiding; no wheel odometry |
| Estimation | IMU prediction + complementary GNSS/odometry correction | IMU prediction + complementary GNSS correction |
| Controller | Cruise → STOP / DEGRADED_STOP | Cruise → HOLD / DEGRADED_HOLD, geometric attitude PD and bounded disturbance integral |

World frame: ENU metres/seconds/radians, body x-forward y-left z-up. The accelerometer is derived from consecutive truth velocities, transformed to body specific force and given noise/bias. **Attitude aiding is generated from true attitude plus noise, not an AHRS implementation.** Controller inputs use the simulated sensor bus; monitors use privileged simulator truth. This privileged truth is an explicit simulation assumption and is unavailable directly on a vehicle.

Actuator degradation scales one selected actuator's force/torque gain over [1.8,6.0) s. Activation lag and tire friction are plant uncertainties present from startup. Wind is an air-velocity field over [2.0,6.0) s. Navigation faults act on 10 Hz source samples over [1.8,1.8+duration) s. Stale faults preserve the old source timestamp; arrival time is never substituted. Bias changes world-x GNSS position only. GNSS transport latency is constant per run. Operator-link loss is an independent latched event at 2.0 s, not a navigation failure.

The legacy controller intentionally retains a known defect: it processes link loss only when navigation updates. The guarded variant processes it independently, rejects old/repeated/high-innovation fixes and adds a small bounded quadrotor integral. **These controller changes were manually authored.** A comparison measures their behavior; it is not automatic code diagnosis or synthesis of a repair. The legacy baseline is a deliberately defective demonstration baseline, not a commercial autopilot.

## Mission requirements and monitor semantics

`contracts/mission.json` expresses policy bindings, frame/units, target and safety bounds, normalization scales, response deadline, dwell time, controller parameters and campaign distribution. Each deadline includes a written illustrative budget rationale. **A mission owner must justify/approve real vehicle requirements; none of these values are field standards.**

- Rover: finish a 0.4 s dwell at horizontal speed ≤0.2 m/s within 2.6 s of link loss; safety limits speed, tilt and horizontal displacement from the loss position.
- Quadrotor: finish a 0.5 s dwell within 3.0 s of loss, simultaneously satisfying horizontal speed ≤0.3 m/s, vertical speed ≤0.25 m/s, altitude error ≤0.35 m and horizontal displacement from actual loss position ≤1.5 m. Safety limits altitude, tilt and a wider 6 m containment radius.
- A dwell window includes both sampled endpoints. Its completion must be at or before the deadline. Safety invariants apply over the entire observed 8 s trace, including after recovery.
- SAT needs the required witness and complete safety telemetry. A complete trace with no trigger is NOT_EXERCISED. A missing target sample can leave recovery INCONCLUSIVE; an already witnessed safety breach remains VIOLATED even if other data is absent or invalid. Empty traces never pass.
- The quantitative recovery score is the maximum, over eligible dwell windows ending by the deadline, of the minimum normalized predicate margin in that window. Safety scores are observed minimum normalized margins. They are **sampled** robustness, not continuous-time guarantees or a cross-vehicle risk scale.
- The Boolean oracle uses direct comparisons and independently implemented indexing/window loops. It shares the contract, not the quantitative margin implementation. Hand-specified expected boundary cases and analytic physics checks provide additional reference points; agreement is still not a proof.

Control/monitor samples are 20 ms; physics steps are 5 ms. Numerical sensitivity halves physics steps to 2.5 ms on each retained reduced case. Both retained verdicts were unchanged; rover max speed difference was 0.0217 m/s, quadrotor 0.00183 m/s. This is only a two-case sensitivity experiment.

## Search, reduction and held-out design

`results/FREEZE.json` records the full contract, environment and SHA-256 manifest for the evaluator, tests, plant XMLs and pinned requirements **before generating held-out scenarios**. Nominal, long-dropout and severe combined development examples were inspected beforehand; this is an internally frozen campaign, not external preregistration. No controller/threshold tuning followed inspection of held-out results.

Search compares 12 pseudorandom attempts with 12 shifted-Halton attempts per model, fixed seeds and identical ranges. Every attempt consumes budget. Only conclusive exercised recovery results have an eligible objective. Search optimizes the legacy controller's recovery robustness. One run per method is insufficient to establish algorithm superiority; all attempt ledgers are saved.

Reduction deletes hook groups then weakens them in a fixed greedy order. Each candidate is checked on seeds 4101/4102/4103, preserving `recovery VIOLATED` on all three. Confirmation and final materialization simulations count toward the 72-run cap. The resulting rover cost fell 84.3% in 34 simulations; quadrotor cost fell 76.3% in 31. The reduced rover failure is a 2.1 s navigation dropout. The reduced quadrotor failure is a wind-only recovery failure, and the guarded controller **also fails it**. Neither is a global minimum. Leave-one-hook-out ablations measure scenario sensitivity, not defect location.

Fault cost is a declared, dimensionless editorial sum: navigation duration/2.4 (multiplied by absolute bias/2.5 for bias faults), lost authority/0.55, wind magnitude/5, friction reduction/0.65, added latency/0.2, added lag/1.5, absolute IMU bias/0.06 and absolute initial yaw/0.08. The fixed link-loss trigger is excluded. It is not physical damage, reliability improvement, or a metric to compare across vehicles.

Held-out sampling uses a separate PRNG, seeds 87001 (rover) and 97001 (quadrotor), 40 draws each, after freezing. Coordinates are independent uniform values over the JSON ranges; wind has uniform magnitude/azimuth, navigation kind and weakened actuator index are categorical. Scenario noise seeds are 870010+i and common to each controller pair. All raw held-out traces, parameters, verdicts and hashes are retained. Confidence intervals use only conclusive exercised runs; invalid/incomplete/not-exercised counts are reported separately rather than dropped.

## Replay and evidence layout

`results/{rover,multirotor}/` contains compressed JSON evidence for nominal, original/reduced search cases, guard-on-reduced, isolated-outage development comparisons, no-trigger/short traces and every held-out run; plus search/reduction/ablation/replay/timestep ledgers and held-out indexes. `VALIDATION.json` stores the full check log. `SHA256SUMS.json` covers packaged files.

Replay grades are EXACT, TOLERANT (declared 1e-8 absolute/relative tolerance with equal verdict data), DIVERGENT or INCOMPATIBLE. Exact hashes cover canonical complete sample rows. Checkpoint JSON stores MuJoCo's full integration state including warmstart, current model actuator gains, controller/estimator state, delayed messages, source timestamps, RNG state and evidence prefix. It uses no pickle. A stored environment/source/contract mismatch blocks an exact replay claim. Replay is version/architecture dependent; portability is not asserted.

## Limits and next validation gates

The dynamics, airframe, drag coefficients, sensor model and scenario distribution are illustrative and unfitted. There is no aerodynamic rotor model, propeller interaction, battery, structural flexibility, terrain map, obstacle planning, realistic radio stack, AHRS, production flight stack, hardware-in-the-loop or flight test. Simplified support spheres are not a validated caster model. Discrete monitoring can miss between-sample violations. A common modeling or specification mistake can survive oracle agreement and exact replay.

The next evidence-bearing steps are measured plant/sensor identification, requirements signed off by a mission owner, a real autopilot adapter with verified fault-hook semantics, unseen faults/longer missions, improved fault-tolerant quadrotor control evaluated on a **fresh** held-out set, then SITL/HIL and controlled physical validation. PX4 fault injection is environment-dependent; an accepted command alone does not demonstrate that a fault was applied.

## Primary references

- [MuJoCo official Python bindings and installation](https://mujoco.readthedocs.io/en/stable/python.html)
- [MuJoCo dynamics, integration state and reproducibility](https://mujoco.readthedocs.io/en/stable/computation/index.html)
- [MuJoCo actuator/site transmission XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [PX4 failure injection and actual support limitations](https://docs.px4.io/main/en/debug/failure_injection)

Automated falsification and counterexample analysis are established techniques. This release claims an inspectable implementation and bounded recorded results, not novelty of those methods.
