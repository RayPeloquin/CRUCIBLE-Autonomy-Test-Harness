import copy
import json
import tempfile
import unittest
from pathlib import Path

from crucible.evidence import compare, read_json, replay, statistics, wilson, write_json
from crucible.monitor import load_contract
from crucible.runtime import Scenario, digest, simulate

ROOT = Path(__file__).resolve().parents[1]


class EvidenceChecks(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.c = load_contract(ROOT / "contracts/mission.json")

    def test_checkpoint_roundtrip_full_trace_exact_both_models(self):
        for model in self.c["models"]:
            s = Scenario(model, nav_kind="stale", nav_duration=1.1, authority=.8, wind=1.2)
            uninterrupted = simulate(s, "guarded_failsafe", self.c, checkpoint_at=137)
            # Actual JSON boundary, not an in-memory object shortcut.
            checkpoint = json.loads(json.dumps(uninterrupted.pop("checkpoint")))
            resumed = simulate(s, "guarded_failsafe", self.c, physics_dt=checkpoint["physics_dt"], resume_state=checkpoint)
            self.assertEqual(compare(uninterrupted, resumed)["grade"], "EXACT")

    def test_exact_and_perturbation_detection(self):
        s = Scenario("multirotor")
        r = simulate(s, "guarded_failsafe", self.c)
        self.assertEqual(replay(r, self.c)["grade"], "EXACT")
        changed = simulate(s, "guarded_failsafe", self.c, perturb_at=140)
        self.assertEqual(compare(r, changed)["grade"], "DIVERGENT")

    def test_tolerant_grade_requires_matching_verdict(self):
        r = simulate(Scenario("rover"), "guarded_failsafe", self.c)
        a = copy.deepcopy(r)
        a["rows"][10]["x"] += 1e-10
        a["trace_hash"] = digest(a["rows"])
        self.assertEqual(compare(r, a)["grade"], "TOLERANT")
        a["verdict"]["overall"] = "VIOLATED"
        self.assertEqual(compare(r, a)["grade"], "DIVERGENT")

    def test_identical_trace_requires_matching_verdict_in_both_directions(self):
        original = {"rows": [{"x": 0.0}], "verdict": {"overall": "SAT"}}
        original["trace_hash"] = digest(original["rows"])
        changed = copy.deepcopy(original)
        changed["verdict"]["overall"] = "VIOLATED"
        for expected, actual in ((original, changed), (changed, original)):
            with self.subTest(expected=expected["verdict"]):
                self.assertEqual(compare(expected, actual)["grade"], "DIVERGENT")

    def test_actual_trace_checksum_must_match_rows(self):
        original = {"rows": [{"x": 0.0}], "verdict": {"overall": "SAT"}}
        original["trace_hash"] = digest(original["rows"])
        for delta in (1e-10, 1.0):
            for claimed_hash in (original["trace_hash"], "invalid"):
                with self.subTest(delta=delta, claimed_hash=claimed_hash):
                    changed = copy.deepcopy(original)
                    changed["rows"][0]["x"] += delta
                    changed["trace_hash"] = claimed_hash
                    self.assertEqual(compare(original, changed)["grade"], "DIVERGENT")

    def test_tampering_and_incompatible_source_detected(self):
        r = simulate(Scenario("rover"), "guarded_failsafe", self.c)
        a = copy.deepcopy(r)
        a["rows"][10]["x"] += 1
        self.assertEqual(compare(a, r)["grade"], "DIVERGENT")
        a = copy.deepcopy(r)
        a["source_hash"] = "wrong"
        self.assertEqual(replay(a, self.c)["grade"], "INCOMPATIBLE")

    def test_statistics_denominator_and_known_interval(self):
        statuses = ["SAT", "VIOLATED", "INCONCLUSIVE", "NOT_EXERCISED"]
        s = statistics([{"verdict": {"overall": x}} for x in statuses])
        self.assertEqual(s["conclusive_exercised"], 2)
        self.assertEqual(s["failure_fraction"], .5)
        low, high = wilson(0, 40)
        self.assertAlmostEqual(low, 0)
        self.assertAlmostEqual(high, 0.0876216012, places=8)
        self.assertIsNone(wilson(0, 0))

    def test_compressed_file_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "evidence.json.gz"
            data = {"a": [1, 2.5, True], "b": "finite"}
            write_json(path, data)
            self.assertEqual(data, read_json(path))


if __name__ == "__main__":
    unittest.main()
