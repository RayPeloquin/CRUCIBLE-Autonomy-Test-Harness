import unittest
from pathlib import Path

import mujoco
import numpy as np

from crucible.monitor import load_contract
from crucible.runtime import Plant, Scenario, simulate

ROOT = Path(__file__).resolve().parents[1]


class PhysicsChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = load_contract(ROOT / "contracts/mission.json")

    def test_free_fall_against_analytic_velocity_and_position(self):
        m = mujoco.MjModel.from_xml_string('<mujoco><option timestep="0.001" gravity="0 0 -9.81"/><worldbody><body pos="0 0 10"><freejoint/><geom type="sphere" size="0.1" mass="1"/></body></worldbody></mujoco>')
        d = mujoco.MjData(m)
        mujoco.mj_step(m, d, nstep=500)
        self.assertAlmostEqual(d.qvel[2], -9.81 * .5, places=10)
        # Semi-implicit position error is bounded by g*T*dt/2.
        self.assertLessEqual(abs(d.qpos[2] - (10 - .5 * 9.81 * .5**2)), .5 * 9.81 * .5 * .001 + 1e-10)

    def test_quad_hover_force_balance(self):
        p = Plant(Scenario("multirotor"), self.c)
        p.data.qvel[:] = 0
        p.advance(np.full(4, 1.5 * 9.81 / 4), 1.0)
        self.assertAlmostEqual(p.data.qpos[2], 2.0, places=10)
        self.assertLess(np.linalg.norm(p.data.qvel), 1e-9)

    def test_actual_contact_and_force_limits(self):
        r = simulate(Scenario("rover"), "guarded_failsafe", self.c)
        self.assertGreater(r["metrics"]["contact_samples"], 300)
        self.assertLessEqual(r["metrics"]["max_actuator_force"], 1.5 + 1e-10)
        self.assertEqual(r["verdict"]["overall"], "SAT")

    def test_unequal_rotor_force_has_expected_roll_sign(self):
        p = Plant(Scenario("multirotor"), self.c)
        p.data.qvel[:] = 0
        p.advance(np.array([4.2, 3.6, 3.6, 3.6]), .1)
        self.assertGreater(p.data.qvel[3], 0)

    def test_actuator_clip_in_backend(self):
        p = Plant(Scenario("multirotor"), self.c)
        p.advance(np.full(4, 100), .5)
        self.assertLessEqual(np.max(p.data.actuator_force), 7.5 + 1e-10)

    def test_independent_watchdog_observes_loss_during_outage(self):
        s = Scenario("rover", nav_duration=2.3)
        old = simulate(s, "legacy_nav_gated", self.c)
        new = simulate(s, "guarded_failsafe", self.c)
        first_old = next(r["t"] for r in old["rows"] if r["mode"] != "CRUISE")
        first_new = next(r["t"] for r in new["rows"] if r["mode"] != "CRUISE")
        self.assertGreater(first_old, 4)
        self.assertEqual(first_new, 2.0)

    def test_stale_source_timestamps_rejected(self):
        r = simulate(Scenario("multirotor", nav_kind="stale", nav_duration=1.4), "guarded_failsafe", self.c)
        self.assertGreater(r["rows"][-1]["nav_rejected"], 8)
        self.assertTrue(any(r["mode"] == "DEGRADED_HOLD" for r in r["rows"]))

    def test_scenario_validation(self):
        with self.assertRaises(ValueError):
            Scenario("submarine").validate()
        with self.assertRaises(ValueError):
            Scenario("rover", authority=-1).validate()
        with self.assertRaises(ValueError):
            Scenario("rover", actuator_index=2).validate()


if __name__ == "__main__":
    unittest.main()
