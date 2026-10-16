import copy
import math
import unittest
from pathlib import Path

from crucible.monitor import boolean_oracle, evaluate, load_contract, ticks

ROOT = Path(__file__).resolve().parents[1]


class MonitorChecks(unittest.TestCase):
    def setUp(self):
        self.c = load_contract(ROOT / "contracts/mission.json")

    def trace(self, recover=2.0, trigger=True):
        return [{"t": k * .02, "operator_link_lost": trigger and k >= 100,
                 "speed": 0.1 if k * .02 >= recover else 1.2,
                 "tilt": 0.0, "post_event_travel": 0.0} for k in range(401)]

    def check(self, rows, expected, recovery=None):
        v = evaluate(rows, self.c, "rover")
        o = boolean_oracle(rows, self.c, "rover")
        self.assertEqual(v["overall"], expected)
        self.assertEqual(v["overall"], o["overall"])
        self.assertEqual(v["recovery"]["status"], o["recovery"])
        self.assertEqual([r["status"] for r in v["invariants"]], o["invariants"])
        if recovery:
            self.assertEqual(v["recovery"]["status"], recovery)
        return v

    def test_dwell_completion_included_at_deadline(self):
        v = self.check(self.trace(recover=4.2), "SAT")
        self.assertAlmostEqual(v["recovery"]["completion_s"], 4.6)

    def test_dwell_one_tick_late_fails(self):
        self.check(self.trace(recover=4.22), "VIOLATED")

    def test_transient_crossing_does_not_pass(self):
        rows = self.trace(recover=20)
        rows[140]["speed"] = 0.0
        self.check(rows, "VIOLATED")

    def test_closed_window_includes_start(self):
        rows = self.trace(recover=4.2)
        rows[210]["speed"] = .21
        self.check(rows, "VIOLATED")

    def test_equal_bound_passes(self):
        rows = self.trace()
        for row in rows:
            row["speed"] = .2
        self.check(rows, "SAT")

    def test_empty_never_passes(self):
        self.check([], "INCONCLUSIVE")

    def test_no_trigger(self):
        self.check(self.trace(trigger=False), "NOT_EXERCISED")

    def test_unknown_trigger_never_not_exercised(self):
        rows = self.trace(trigger=False)
        del rows[120]["operator_link_lost"]
        self.check(rows, "INCONCLUSIVE")

    def test_short_trace(self):
        self.check(self.trace(recover=20)[:150], "INCONCLUSIVE")

    def test_bad_other_signal_cannot_hide_known_breach(self):
        rows = self.trace()
        rows[30]["speed"] = float("nan")
        rows[31]["tilt"] = .8
        v = self.check(rows, "VIOLATED")
        self.assertEqual(v["invariants"][0]["status"], "INCONCLUSIVE")
        self.assertEqual(v["invariants"][1]["status"], "VIOLATED")

    def test_missing_tick_cannot_hide_known_breach(self):
        rows = self.trace()
        rows[50]["speed"] = 3
        del rows[120]
        self.check(rows, "VIOLATED")

    def test_duplicate_out_of_order_inconclusive(self):
        rows = self.trace()
        rows.insert(30, copy.deepcopy(rows[20]))
        self.check(rows, "INCONCLUSIVE")

    def test_missing_signal_inside_dwell(self):
        rows = self.trace(recover=4.2)
        del rows[220]["speed"]
        self.check(rows, "INCONCLUSIVE")

    def test_known_recovery_survives_later_gap(self):
        rows = self.trace()
        del rows[350]
        self.check(rows, "INCONCLUSIVE", "SAT")

    def test_grid_rejects_fractional_deadline(self):
        with self.assertRaises(ValueError):
            ticks(1.401, .02)

    def test_generated_boundary_table(self):
        # Deterministic exhaustive recovery-time sweep, independent expected
        # answer: speed step at tick r + 20 dwell ticks <= deadline tick 230.
        for r in range(90, 252):
            rows = self.trace(recover=r * .02)
            expected = "SAT" if max(100, r) + 20 <= 230 else "VIOLATED"
            self.check(rows, expected)


if __name__ == "__main__":
    unittest.main()
