"""MuJoCo models, simulated sensors, example controllers and closed-loop runner.

Controllers receive SensorBus/Estimator outputs, never the monitor's truth row.
Attitude is synthetic aided attitude (not a validated AHRS). IMU acceleration is
derived from consecutive world velocities, then transformed into specific force.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import platform
from dataclasses import asdict, dataclass
from pathlib import Path

import mujoco
import numpy as np

from .monitor import boolean_oracle, evaluate

ROOT = Path(__file__).resolve().parents[1]
VARIANTS = ("legacy_nav_gated", "guarded_failsafe")


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def source_manifest():
    paths = sorted(list((ROOT / "crucible").glob("*.py")) + list((ROOT / "tests").glob("*.py")) + list((ROOT / "models").glob("*.xml")) + [ROOT / "contracts/mission.json", ROOT / "requirements.txt"])
    return {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}


@dataclass
class Scenario:
    model: str
    seed: int = 4101
    name: str = "nominal"
    event_s: float | None = 2.0
    nav_kind: str = "dropout"
    nav_start: float = 1.8
    nav_duration: float = 0.0
    nav_bias: float = 0.0
    latency: float = 0.04
    authority: float = 1.0
    actuator_index: int = 0
    actuator_start: float = 1.8
    actuator_end: float = 6.0
    wind: float = 0.0
    wind_y: float = 0.0
    wind_start: float = 2.0
    wind_end: float = 6.0
    friction: float = 0.9
    lag_scale: float = 1.0
    imu_bias: float = 0.0
    initial_yaw: float = 0.0

    def validate(self):
        if self.model not in ("rover", "multirotor") or self.nav_kind not in ("dropout", "stale", "bias"):
            raise ValueError("Unsupported model or fault hook")
        if type(self.seed) is not int or self.seed < 0 or type(self.actuator_index) is not int:
            raise ValueError("Seed and actuator index must be nonnegative integers")
        numeric = [v for k, v in asdict(self).items() if k not in ("model", "name", "nav_kind", "event_s")]
        if not all(type(v) in (int, float) and math.isfinite(v) for v in numeric):
            raise ValueError("Nonfinite scenario")
        if not 0 <= self.authority <= 1 or not 0.05 <= self.friction <= 2 or not 0 <= self.latency <= 2 or not 0.1 <= self.lag_scale <= 10:
            raise ValueError("Fault parameter out of range")
        if self.nav_duration < 0 or self.nav_start < 0 or self.actuator_start < 0 or self.actuator_end < self.actuator_start or self.wind_end < self.wind_start:
            raise ValueError("Invalid fault interval")
        if self.event_s is not None and (not math.isfinite(self.event_s) or self.event_s < 0):
            raise ValueError("Invalid event time")
        if not 0 <= self.actuator_index < (2 if self.model == "rover" else 4):
            raise ValueError("Invalid actuator index")


class Plant:
    def __init__(self, scenario, contract, physics_dt=None):
        scenario.validate()
        self.s = scenario
        self.model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / f"{scenario.model}.xml"))
        self.model.opt.timestep = physics_dt or contract["physics_dt"]
        self.data = mujoco.MjData(self.model)
        self.body_id = self.model.body("chassis" if scenario.model == "rover" else "airframe").id
        yaw = scenario.initial_yaw
        self.data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        speed = contract["models"][scenario.model]["controller"]["cruise_v"]
        self.data.qvel[0:3] = [speed, 0, 0]
        self.model.actuator_dynprm[:, 0] *= scenario.lag_scale
        if scenario.model == "rover":
            self.data.qvel[6:] = speed / 0.12
            for name in ("left_tire", "right_tire"):
                self.model.geom_friction[self.model.geom(name).id, 0] = scenario.friction
        else:
            self.data.act[:] = 1.5 * 9.81 / 4
            self.data.ctrl[:] = self.data.act
        mujoco.mj_forward(self.model, self.data)

    def truth(self):
        # mj_forward refreshes derived pose/force quantities after each step.
        mujoco.mj_forward(self.model, self.data)
        vel = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.body_id, vel, 0)
        return {"p": self.data.xpos[self.body_id].copy(), "v": vel[3:].copy(),
                "omega": self.data.qvel[3:6].copy(),
                "R": self.data.xmat[self.body_id].reshape(3, 3).copy(),
                "wheel_v": float(np.mean(self.data.qvel[6:]) * 0.12) if self.s.model == "rover" else None}

    def advance(self, command, interval):
        n = int(round(interval / self.model.opt.timestep))
        if abs(n * self.model.opt.timestep - interval) > 1e-10:
            raise ValueError("Controller step must contain whole physics steps")
        for _ in range(n):
            t = float(self.data.time)
            active = self.s.actuator_start - 1e-9 <= t < self.s.actuator_end - 1e-9
            self.model.actuator_gainprm[:, 0] = 1
            if active:
                self.model.actuator_gainprm[self.s.actuator_index, 0] = self.s.authority
            self.data.ctrl[:] = command
            self.data.xfrc_applied[:] = 0
            # Explicit simple air-relative drag. Wind is a velocity field, not
            # a direct force injection. Coefficients are illustrative/unfitted.
            velocity = self.data.qvel[:3]
            wind_active = self.s.wind_start - 1e-9 <= t < self.s.wind_end - 1e-9
            air = np.array([self.s.wind, self.s.wind_y, 0.0]) if wind_active else np.zeros(3)
            rel = velocity - air
            coefficient = 0.16 if self.s.model == "multirotor" else 0.035
            self.data.xfrc_applied[self.body_id, :3] = -coefficient * np.linalg.norm(rel) * rel
            if self.s.model == "multirotor":
                rotation = self.data.xmat[self.body_id].reshape(3, 3)
                self.data.xfrc_applied[self.body_id, 3:] = -0.02 * (rotation @ self.data.qvel[3:6])
            mujoco.mj_step(self.model, self.data)


class SensorBus:
    """10 Hz GNSS queue with source timestamps; 50 Hz noisy IMU/attitude aid.

    The PRNG draw count is independent of controller state/fault selection so
    paired controller runs consume identical exogenous noise streams.
    """
    def __init__(self, s, dt, initial):
        self.s, self.dt = s, dt
        self.rng = np.random.default_rng(s.seed)
        self.queue = []
        self.last_fix = None
        self.stale_fix = None
        self.previous_v = initial["v"].copy()
        self.previous_R = initial["R"].copy()

    def sample(self, k, truth):
        t = k * self.dt
        # Always draw the same 18 numbers on every controller tick.
        noise = self.rng.standard_normal(18)
        attitude_error = noise[:3] * 0.002
        skew = np.array([[0, -attitude_error[2], attitude_error[1]],
                         [attitude_error[2], 0, -attitude_error[0]],
                         [-attitude_error[1], attitude_error[0], 0]])
        Rn = truth["R"] @ (np.eye(3) + skew)
        # Project noisy rotation back onto SO(3); no ground-truth attitude is
        # passed separately to the controller.
        u, _, vh = np.linalg.svd(Rn)
        Rn = u @ vh
        acceleration = (truth["v"] - self.previous_v) / self.dt if k else np.zeros(3)
        specific = self.previous_R.T @ (acceleration + [0, 0, 9.81])
        specific += noise[3:6] * 0.035 + np.array([self.s.imu_bias, 0.01, 0.015])
        gyro = truth["omega"] + noise[6:9] * 0.004 + [0.001, -0.001, 0.002]
        self.previous_v, self.previous_R = truth["v"].copy(), truth["R"].copy()
        if k % 5 == 0:
            fix = {"stamp": t, "p": truth["p"] + noise[9:12] * 0.025,
                   "v": truth["v"] + noise[12:15] * 0.025}
            affected = self.s.nav_start - 1e-9 <= t < self.s.nav_start + self.s.nav_duration - 1e-9
            if affected and self.s.nav_kind == "stale":
                if self.stale_fix is None:
                    self.stale_fix = copy.deepcopy(self.last_fix or fix)
                fix = copy.deepcopy(self.stale_fix)
            elif affected and self.s.nav_kind == "bias":
                fix["p"][0] += self.s.nav_bias
            if not (affected and self.s.nav_kind == "dropout"):
                self.queue.append((t + self.s.latency, fix))
                self.last_fix = copy.deepcopy(fix)
        delivered = [fix for arrival, fix in self.queue if arrival <= t + 1e-9]
        self.queue = [(arrival, fix) for arrival, fix in self.queue if arrival > t + 1e-9]
        return {"R": Rn, "accel": specific, "gyro": gyro, "fixes": delivered,
                "wheel_v": truth["wheel_v"] + noise[15] * 0.015 if truth["wheel_v"] is not None else None,
                "operator_link_lost": self.s.event_s is not None and t >= self.s.event_s - 1e-9}


class Estimator:
    def __init__(self, model, spec):
        self.model, self.spec = model, spec
        self.p = np.array([0.0, 0.0, 2.0 if model == "multirotor" else 0.22])
        self.v = np.array([spec["cruise_v"], 0.0, 0.0])
        self.last_stamp = -1.0
        self.rejected = 0

    def update(self, sense, t, dt, guarded):
        a = sense["R"] @ sense["accel"] - [0, 0, 9.81]
        self.p += self.v * dt + 0.5 * a * dt * dt
        self.v += a * dt
        accepted = False
        for fix in sense["fixes"]:
            age = t - fix["stamp"]
            predicted_position = fix["p"] + fix["v"] * age
            reject = (age > self.spec["nav_max_age_s"] + 1e-9 or fix["stamp"] <= self.last_stamp
                      or np.linalg.norm(predicted_position - self.p) > self.spec["innovation_pos_m"]
                      or np.linalg.norm(fix["v"] - self.v) > self.spec["innovation_vel_mps"])
            if guarded and reject:
                self.rejected += 1
                continue
            # Complementary correction, not an EKF or integrity-certified fusion.
            self.p += 0.65 * (predicted_position - self.p)
            self.v += 0.75 * (fix["v"] - self.v)
            self.last_stamp = fix["stamp"]
            accepted = True
        if self.model == "rover":
            direction = sense["R"][:, 0]
            self.v += 0.35 * (sense["wheel_v"] * direction - self.v)
            self.p[2], self.v[2] = 0.22, 0.0
        return accepted


class Controller:
    def __init__(self, model, variant, spec):
        if variant not in VARIANTS:
            raise ValueError("Unknown controller variant")
        self.model, self.variant, self.spec = model, variant, spec
        self.est = Estimator(model, spec)
        self.failed = False
        self.hold = None
        self.integral = np.zeros(2)
        # [thrust, roll, pitch, yaw] = allocation @ individual rotor forces.
        self.allocation = np.array([[1, 1, 1, 1], [.18, .18, -.18, -.18],
                                    [-.18, .18, .18, -.18], [.018, -.018, .018, -.018]])
        self.mixer = np.linalg.inv(self.allocation)

    def update(self, sense, t, dt):
        guarded = self.variant == "guarded_failsafe"
        nav_updated = self.est.update(sense, t, dt if t > 0 else 0, guarded)
        # Explicit retained v0.1 mutant: link processing depends on navigation.
        # The guard is manually authored, not an engine-generated repair.
        if (guarded or nav_updated) and sense["operator_link_lost"]:
            if not self.failed:
                self.hold = self.est.p.copy()
                self.hold[2] = 2.0
            self.failed = True
        age = t - self.est.last_stamp if self.est.last_stamp >= 0 else t + 1.0
        mode = "CRUISE"
        if self.failed:
            mode = "STOP" if self.model == "rover" else "HOLD"
            if guarded and age > self.spec["nav_max_age_s"]:
                mode = "DEGRADED_" + mode
        R, omega = sense["R"], sense["gyro"]
        if self.model == "rover":
            speed = float(self.est.v @ R[:, 0])
            target = 0 if self.failed else self.spec["cruise_v"]
            common = self.spec["speed_gain"] * (target - speed)
            yaw = math.atan2(R[1, 0], R[0, 0])
            turn = -self.spec["yaw_gain"] * yaw - self.spec["yaw_rate_gain"] * omega[2]
            raw = np.array([common - turn, common + turn])
            cmd = np.clip(raw, -1.5, 1.5)
        else:
            if self.failed:
                error = self.hold[:2] - self.est.p[:2]
                desired_a = self.spec["position_gain"] * error - self.spec["velocity_gain"] * self.est.v[:2]
            else:
                desired_a = self.spec["velocity_gain"] * (np.array([self.spec["cruise_v"], 0]) - self.est.v[:2])
            if guarded and age <= self.spec["nav_max_age_s"]:
                # Small bounded velocity integral compensates steady disturbances.
                desired_v = np.zeros(2) if self.failed else np.array([self.spec["cruise_v"], 0])
                self.integral = np.clip(self.integral + 0.18 * (desired_v - self.est.v[:2]) * dt, -.8, .8)
            desired_a += self.integral if guarded else 0
            limit = 9.81 * math.tan(self.spec["tilt_command_rad"])
            desired_a *= min(1.0, limit / max(float(np.linalg.norm(desired_a)), 1e-12))
            az = self.spec["altitude_gain"] * (2.0 - self.est.p[2]) - self.spec["vertical_velocity_gain"] * self.est.v[2]
            force = 1.5 * np.array([desired_a[0], desired_a[1], np.clip(az + 9.81, 3.0, 17.0)])
            zb = force / np.linalg.norm(force)
            yb = np.cross(zb, [1.0, 0.0, 0.0])
            yb /= np.linalg.norm(yb)
            Rd = np.column_stack((np.cross(yb, zb), yb, zb))
            skew = 0.5 * (Rd.T @ R - R.T @ Rd)
            eR = np.array([skew[2, 1], skew[0, 2], skew[1, 0]])
            torque = -self.spec["attitude_gain"] * eR - self.spec["rate_gain"] * omega
            thrust = float(force @ R[:, 2])
            raw = self.mixer @ np.r_[thrust, torque]
            cmd = np.clip(raw, 0, 7.5)
        return cmd, {"mode": mode, "nav_age": age, "nav_rejected": self.est.rejected,
                     "saturated": bool(np.any(abs(raw - cmd) > 1e-8)),
                     "p_est": self.est.p.copy(), "v_est": self.est.v.copy()}


def environment():
    return {"mujoco": mujoco.__version__, "numpy": np.__version__,
            "python": platform.python_version(), "machine": platform.machine(), "system": platform.system()}


def jsonable(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {k: jsonable(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [jsonable(v) for v in value]
    return value


def capture(plant, ctrl, bus, k, rows, event_position, max_force, contact_samples, contract):
    flags = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.zeros(mujoco.mj_stateSize(plant.model, flags))
    mujoco.mj_getState(plant.model, plant.data, state, flags)
    return jsonable({
        "schema": "crucible-checkpoint/2", "source_hash": digest(source_manifest()),
        "contract_hash": digest(contract), "environment": environment(),
        "scenario": asdict(plant.s), "controller": ctrl.variant,
        "physics_dt": float(plant.model.opt.timestep), "k": k,
        "mujoco_integration_state": state, "actuator_gainprm": plant.model.actuator_gainprm.copy(),
        "controller_state": {"failed": ctrl.failed, "hold": ctrl.hold, "integral": ctrl.integral,
                             "est_p": ctrl.est.p, "est_v": ctrl.est.v,
                             "last_stamp": ctrl.est.last_stamp, "rejected": ctrl.est.rejected},
        "sensor_state": {"queue": bus.queue, "last_fix": bus.last_fix, "stale_fix": bus.stale_fix,
                         "previous_v": bus.previous_v, "previous_R": bus.previous_R,
                         "rng": bus.rng.bit_generator.state},
        "rows": rows, "event_position": event_position,
        "max_force": max_force, "contact_samples": contact_samples,
    })


def restore(checkpoint, plant, ctrl, bus, contract):
    if checkpoint["schema"] != "crucible-checkpoint/2" or checkpoint["contract_hash"] != digest(contract):
        raise ValueError("Incompatible checkpoint contract")
    if checkpoint["source_hash"] != digest(source_manifest()) or checkpoint["environment"] != environment():
        raise ValueError("Incompatible checkpoint source/environment")
    mujoco.mj_setState(plant.model, plant.data, np.array(checkpoint["mujoco_integration_state"]), mujoco.mjtState.mjSTATE_INTEGRATION)
    plant.model.actuator_gainprm[:] = checkpoint["actuator_gainprm"]
    cs = checkpoint["controller_state"]
    ctrl.failed = cs["failed"]
    ctrl.hold = np.array(cs["hold"]) if cs["hold"] is not None else None
    ctrl.integral = np.array(cs["integral"])
    ctrl.est.p, ctrl.est.v = np.array(cs["est_p"]), np.array(cs["est_v"])
    ctrl.est.last_stamp, ctrl.est.rejected = cs["last_stamp"], cs["rejected"]
    ss = checkpoint["sensor_state"]

    def fix(f):
        return {"stamp": f["stamp"], "p": np.array(f["p"]), "v": np.array(f["v"])} if f is not None else None

    bus.queue = [(arrival, fix(f)) for arrival, f in ss["queue"]]
    bus.last_fix, bus.stale_fix = fix(ss["last_fix"]), fix(ss["stale_fix"])
    bus.previous_v, bus.previous_R = np.array(ss["previous_v"]), np.array(ss["previous_R"])
    bus.rng.bit_generator.state = ss["rng"]


def simulate(scenario, variant, contract, physics_dt=None, horizon_s=None, perturb_at=None,
             checkpoint_at=None, resume_state=None):
    plant = Plant(scenario, contract, physics_dt)
    dt = contract["control_dt"]
    ctrl = Controller(scenario.model, variant, contract["models"][scenario.model]["controller"])
    initial = plant.truth()
    bus = SensorBus(scenario, dt, initial)
    count = int(round((horizon_s if horizon_s is not None else contract["horizon_s"]) / dt))
    rows, event_position = [], None
    max_force, contact_samples, start = 0.0, 0, 0
    checkpoint = None
    if resume_state is not None:
        if resume_state["scenario"] != asdict(scenario) or resume_state["controller"] != variant:
            raise ValueError("Checkpoint scenario/controller mismatch")
        restore(resume_state, plant, ctrl, bus, contract)
        start, rows = resume_state["k"], copy.deepcopy(resume_state["rows"])
        event_position = np.array(resume_state["event_position"]) if resume_state["event_position"] is not None else None
        max_force, contact_samples = resume_state["max_force"], resume_state["contact_samples"]
    for k in range(start, count + 1):
        t = k * dt
        if checkpoint_at == k:
            checkpoint = capture(plant, ctrl, bus, k, copy.deepcopy(rows), event_position,
                                 max_force, contact_samples, contract)
        if perturb_at is not None and k == perturb_at:
            plant.data.qvel[0] += 0.1
        truth = plant.truth()
        sense = bus.sample(k, truth)
        command, diagnostic = ctrl.update(sense, t, dt)
        if sense["operator_link_lost"] and event_position is None:
            event_position = truth["p"].copy()
        p, v = truth["p"], truth["v"]
        row = {"t": t, "x": float(p[0]), "y": float(p[1]), "z": float(p[2]),
               "vx": float(v[0]), "vy": float(v[1]), "vz": float(v[2]),
               "speed": float(np.linalg.norm(v[:2])),
               "vertical_speed": abs(float(v[2])),
               "tilt": math.acos(float(np.clip(truth["R"][2, 2], -1, 1))),
               "altitude_error": abs(float(p[2]) - 2.0),
               "post_event_travel": float(np.linalg.norm((p - event_position)[:2])) if event_position is not None else 0.0,
               "operator_link_lost": sense["operator_link_lost"],
               "mode": diagnostic["mode"], "nav_age": float(diagnostic["nav_age"]),
               "nav_rejected": int(diagnostic["nav_rejected"]), "saturated": diagnostic["saturated"],
               "est_error": float(np.linalg.norm(diagnostic["p_est"] - p)),
               "v_est_x": float(diagnostic["v_est"][0]),
               "contacts": int(plant.data.ncon),
               "actuator_forces": plant.data.actuator_force.tolist(), "commands": command.tolist()}
        rows.append(row)
        max_force = max(max_force, max(abs(float(x)) for x in plant.data.actuator_force))
        contact_samples += int(plant.data.ncon > 0)
        if k < count:
            plant.advance(command, dt)
    verdict = evaluate(rows, contract, scenario.model)
    oracle = boolean_oracle(rows, contract, scenario.model)
    agreement = (verdict["overall"] == oracle["overall"]
                 and verdict["recovery"]["status"] == oracle["recovery"]
                 and [x["status"] for x in verdict["invariants"]] == oracle["invariants"])
    if not agreement:
        raise AssertionError("Quantitative monitor and independent boolean oracle disagree")
    result = {"schema": "crucible-evidence/2", "scenario": asdict(scenario), "controller": variant,
            "contract_hash": digest(contract), "environment": environment(),
            "source_hash": digest(source_manifest()),
            "physics_dt": float(plant.model.opt.timestep), "rows": rows, "trace_hash": digest(rows),
            "verdict": verdict, "oracle_agreement": agreement,
            "metrics": {"max_actuator_force": max_force, "contact_samples": contact_samples,
                        "saturated_fraction": sum(r["saturated"] for r in rows) / len(rows),
                        "max_est_error_m": max(r["est_error"] for r in rows),
                        "max_speed_mps": max(r["speed"] for r in rows)}}
    if checkpoint is not None:
        result["checkpoint"] = checkpoint
    return result
