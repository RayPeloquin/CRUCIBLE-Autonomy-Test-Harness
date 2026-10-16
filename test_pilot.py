"""Checks of evidence semantics and the seeded end-to-end demonstration."""
import copy
import unittest

import crucible_pilot as cp


def trace_with_target(latency, horizon, *, trigger=0):
    trace = []
    for tick in range(horizon + 1):
        trace.append({"tick": tick, "t_s": tick * cp.DT,
                      "operator_link": tick < trigger,
                      "metrics": {"horizontal_speed_mps": 0.0 if latency is not None and tick >= trigger + latency else 1.0,
                                  "altitude_error_m": 0.0}})
    return trace


class MonitorTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(cp.PLATFORMS["rover"])
        self.config["deadline_s"] = 1.0

    def status(self, trace):
        return cp.evaluate(trace, self.config)["rules"][cp.TARGET_RULE]["status"]

    def test_target_exactly_at_deadline(self):
        self.assertEqual(self.status(trace_with_target(50, 50)), "SATISFIED")

    def test_target_one_tick_late(self):
        self.assertEqual(self.status(trace_with_target(51, 51)), "VIOLATED")

    def test_unrecovered_at_deadline(self):
        self.assertEqual(self.status(trace_with_target(None, 50)), "VIOLATED")

    def test_early_termination_is_inconclusive(self):
        self.assertEqual(self.status(trace_with_target(None, 49)), "INCONCLUSIVE")

    def test_target_observed_early_can_satisfy(self):
        self.assertEqual(self.status(trace_with_target(20, 20)), "SATISFIED")

    def test_no_trigger_is_not_exercised(self):
        trace = trace_with_target(None, 80, trigger=100)
        result = cp.evaluate(trace, self.config)
        self.assertEqual(result["rules"][cp.TARGET_RULE]["status"], "NOT_EXERCISED")
        self.assertFalse(result["eligible"])

    def test_empty_trace_is_invalid(self):
        self.assertFalse(cp.evaluate([], self.config)["trace_valid"])

    def test_missing_and_nan_telemetry_are_invalid(self):
        trace = trace_with_target(None, 60)
        trace[10]["metrics"]["horizontal_speed_mps"] = float("nan")
        self.assertFalse(cp.evaluate(trace, self.config)["trace_valid"])
        del trace[10]["metrics"]["horizontal_speed_mps"]
        self.assertFalse(cp.evaluate(trace, self.config)["trace_valid"])

    def test_missing_or_reordered_tick_is_invalid(self):
        trace = trace_with_target(None, 60)
        del trace[10]
        self.assertFalse(cp.evaluate(trace, self.config)["trace_valid"])

    def test_independent_violation_survives_inconclusive_deadline(self):
        trace = trace_with_target(None, 10)
        trace[5]["metrics"]["horizontal_speed_mps"] = 3.0
        result = cp.evaluate(trace, self.config)
        self.assertEqual(result["rules"][cp.TARGET_RULE]["status"], "INCONCLUSIVE")
        self.assertIn("speed_envelope", result["violations"])

    def test_zero_upper_bound_margin_satisfies(self):
        trace = trace_with_target(None, 10)
        trace[5]["metrics"]["horizontal_speed_mps"] = self.config["speed_limit_mps"]
        result = cp.evaluate(trace, self.config)
        self.assertEqual(result["rules"]["speed_envelope"]["minimum_normalized_margin"], 0)
        self.assertEqual(result["rules"]["speed_envelope"]["status"], "SATISFIED")

    def test_each_loss_window_is_reported(self):
        trace = trace_with_target(None, 80)
        for tick in range(10, 20):
            trace[tick]["operator_link"] = True
        result = cp.evaluate(trace, self.config)["rules"][cp.TARGET_RULE]
        self.assertEqual([w["trigger_tick"] for w in result["windows"]], [0, 20])
        self.assertEqual(result["status"], "VIOLATED")

    def test_inconclusive_trials_consume_search_budget(self):
        future = [cp.Fault("operator_link_loss", cp.HORIZON - 1, 100, fixed=True)]
        best, attempts = cp.bounded_search("rover", budget=2, schedules=[future, future, [cp.fixed_trigger()]])
        self.assertEqual(len(attempts), 2)
        self.assertIsNone(best)
        self.assertTrue(all(not t["evaluation"]["eligible"] for t in attempts))

    def test_fault_interval_end_is_exclusive(self):
        fault = cp.Fault("navigation_freeze", 10, 2)
        self.assertFalse(fault.active(9))
        self.assertTrue(fault.active(10))
        self.assertTrue(fault.active(11))
        self.assertFalse(fault.active(12))


class PilotTests(unittest.TestCase):
    def test_incompatible_adapter_is_rejected(self):
        config = copy.deepcopy(cp.PLATFORMS["rover"])
        config["adapter"] = "multirotor_si_v1"
        with self.assertRaisesRegex(ValueError, "incompatible"):
            cp.run_case("rover", [cp.fixed_trigger()], config=config)

    def test_non_grid_deadline_is_rejected(self):
        config = copy.deepcopy(cp.PLATFORMS["rover"])
        config["deadline_s"] = 1.401
        with self.assertRaisesRegex(ValueError, "sampling grid"):
            cp.run_case("rover", [cp.fixed_trigger()], config=config)

    def test_stale_navigation_keeps_timestamp_and_independent_link(self):
        for machine in cp.PLATFORMS:
            with self.subTest(machine=machine):
                faults = [cp.fixed_trigger(), cp.Fault("navigation_freeze", 95, 45)]
                vulnerable = cp.run_case(machine, faults)
                row = vulnerable["trace"][cp.LOSS_START]
                self.assertFalse(row["delivered"]["operator_link"])
                self.assertFalse(row["delivered"]["nav_fresh"])
                self.assertEqual(row["delivered"]["nav_tick"], 94)
                self.assertEqual(row["mode"], "cruise")
                repaired = cp.run_case(machine, faults, repaired=True)
                self.assertEqual(repaired["trace"][cp.LOSS_START]["mode"], cp.PLATFORMS[machine]["policy_action"])

    def test_two_models_search_reduce_repair_replay(self):
        for machine in cp.PLATFORMS:
            with self.subTest(machine=machine):
                baseline = cp.run_case(machine, [cp.fixed_trigger()])
                self.assertEqual(baseline["evaluation"]["rules"][cp.TARGET_RULE]["status"], "SATISFIED")
                original, attempts = cp.bounded_search(machine)
                self.assertEqual(len(attempts), 15)
                self.assertIn(cp.TARGET_RULE, original["evaluation"]["violations"])
                for extra in [cp.Fault(**f) for f in original["faults"] if not f["fixed"]]:
                    single = cp.run_case(machine, [cp.fixed_trigger(), extra])
                    self.assertEqual(single["evaluation"]["rules"][cp.TARGET_RULE]["status"], "SATISFIED")
                reduced, record = cp.reduce_failure(machine, original)
                self.assertLess(record["reduced_cost_s"], record["original_cost_s"])
                self.assertIn(cp.TARGET_RULE, reduced["evaluation"]["violations"])
                self.assertLessEqual(len(record["attempts"]), record["budget"])
                self.assertIn(cp.asdict(cp.fixed_trigger()), reduced["faults"])
                repaired = cp.run_case(machine, [cp.Fault(**f) for f in reduced["faults"]], repaired=True)
                self.assertEqual(repaired["evaluation"]["rules"][cp.TARGET_RULE]["status"], "SATISFIED")
                self.assertEqual(repaired["evaluation"]["violations"], [])
                self.assertEqual(cp.replay(reduced)["matching_traces"], 3)

    def test_changed_trace_is_rejected_for_replay(self):
        case = cp.run_case("rover", [cp.fixed_trigger()])
        case["trace"][0]["truth"]["x_m"] += 1
        with self.assertRaisesRegex(ValueError, "digest"):
            cp.replay(case)

    def test_changed_policy_is_rejected_for_replay(self):
        case = cp.run_case("rover", [cp.fixed_trigger()])
        case["config"]["deadline_s"] = 0.02
        with self.assertRaisesRegex(ValueError, "verdict"):
            cp.replay(case)


if __name__ == "__main__":
    unittest.main(verbosity=2)
