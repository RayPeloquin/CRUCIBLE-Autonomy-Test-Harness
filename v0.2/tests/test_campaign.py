import dataclasses
import unittest
from pathlib import Path
from unittest.mock import patch

from crucible.campaign import fault_cost, reduce_case, scenario_from_unit, search
from crucible.monitor import load_contract
from crucible.runtime import Scenario

ROOT = Path(__file__).resolve().parents[1]


class CampaignChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = load_contract(ROOT / "contracts/mission.json")

    def test_equal_search_budgets_and_failed_attempts_retained(self):
        def failed(s, *_):
            return {"scenario": dataclasses.asdict(s), "verdict": {"overall": "INCONCLUSIVE", "recovery": {"status": "INCONCLUSIVE", "robustness": None}}}
        with patch("crucible.campaign.attempt", failed):
            methods, best = search("rover", self.c)
        self.assertIsNone(best)
        for method in methods.values():
            self.assertEqual(len(method["attempts"]), 12)
            self.assertEqual(method["eligible"], 0)

    def test_reduction_counts_every_confirmation_and_materialization(self):
        calls = []

        def synthetic(s, *_):
            calls.append(s)
            return {"scenario": dataclasses.asdict(s), "verdict": {"overall": "VIOLATED", "recovery": {"status": "VIOLATED"}}}
        s = Scenario("rover", nav_duration=2, authority=.6, wind=3, friction=.4, lag_scale=2, latency=.2, imu_bias=.03, initial_yaw=.04)
        original = synthetic(s)
        calls.clear()
        with patch("crucible.campaign.attempt", synthetic), patch("crucible.campaign.simulate", synthetic):
            log, reduced = reduce_case(original, self.c)
        self.assertEqual(len(calls), log["simulations"])
        self.assertLessEqual(log["simulations"], self.c["campaign"]["reduction_simulation_budget"])
        self.assertLessEqual(log["final_cost"], log["initial_cost"])
        accepted = [r["cost"] for r in log["ledger"] if r["preserves_recovery_violation_all_seeds"]]
        self.assertTrue(all(b <= a for a, b in zip(accepted, accepted[1:])))

    def test_reducer_rejects_unreplicated_failure(self):
        def inconsistent(s, *_):
            return {"scenario": dataclasses.asdict(s), "verdict": {"overall": "SAT" if s.seed == 4102 else "VIOLATED", "recovery": {"status": "SAT" if s.seed == 4102 else "VIOLATED"}}}
        original = inconsistent(Scenario("rover", nav_duration=2))
        with patch("crucible.campaign.attempt", inconsistent):
            log, retained = reduce_case(original, self.c)
        self.assertEqual(log["status"], "NOT_REDUCED")
        self.assertEqual(log["simulations"], 3)
        self.assertEqual(retained, original)

    def test_fixed_sampler_reproducible_and_within_domain(self):
        for u in ([0] * 12, [.999999] * 12):
            s = scenario_from_unit("rover", u, self.c, 123, "test")
            s.validate()
            self.assertEqual(s, scenario_from_unit("rover", u, self.c, 123, "test"))
            self.assertLessEqual(s.nav_duration, 2.4 + 1e-9)
            self.assertGreaterEqual(s.authority, .45)
            self.assertLessEqual(s.authority, 1)


if __name__ == "__main__":
    unittest.main()
