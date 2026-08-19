#!/usr/bin/env python3
import os
import shutil
import sys
import unittest
from unittest import mock


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import analyze_hazard


FIXTURES = os.path.join(ROOT, "tests", "fixtures")


class HazardAnalysisTest(unittest.TestCase):
    def test_budget_mapping_selects_largest_p95_fit_and_zero(self):
        timing = analyze_hazard.read_timing(os.path.join(FIXTURES, "hazard_timing.csv"))
        mappings = analyze_hazard.map_budgets(timing, (0.01, 0.5, 5.0))
        by_key = {(row["mode"], row["budget_ms"]): row["ray_count"] for row in mappings}
        self.assertEqual(0, by_key["true-brute", 0.01])
        self.assertEqual(5, by_key["true-brute", 0.5])
        self.assertEqual(17, by_key["scene-bvh", 0.5])
        self.assertEqual(361, by_key["scene-bvh", 5.0])

    def test_pairing_rejects_missing_scenarios(self):
        trials = analyze_hazard.read_trials(
            os.path.join(FIXTURES, "hazard_trials_unpaired.csv"))
        timing = [row for row in analyze_hazard.read_timing(
            os.path.join(FIXTURES, "hazard_timing.csv")) if row["ray_count"] == 5]
        with self.assertRaisesRegex(analyze_hazard.ValidationError, "unpaired scenarios"):
            analyze_hazard.validate_pairing(trials, timing)

    def test_paired_bootstrap_is_deterministic(self):
        trials = analyze_hazard.read_trials(os.path.join(FIXTURES, "hazard_trials.csv"))
        grouped = {}
        for row in trials:
            grouped.setdefault((row["mode"], row["ray_count"]), []).append(row)
        first = analyze_hazard.paired_bootstrap_difference(
            grouped["scene-bvh", 17], grouped["true-brute", 5],
            lambda row: float(row["outcome"] == "SafeStop"))
        second = analyze_hazard.paired_bootstrap_difference(
            grouped["scene-bvh", 17], grouped["true-brute", 5],
            lambda row: float(row["outcome"] == "SafeStop"))
        self.assertEqual(first, second)
        self.assertEqual((1.0, 1.0, 1.0, 2), first)

    def test_zero_ray_baseline_does_not_claim_two_x_advantage(self):
        trials = analyze_hazard.read_trials(os.path.join(FIXTURES, "hazard_trials.csv"))
        _, grouped = analyze_hazard.summarize_trials(trials)
        mappings = [
            {"mode": "true-brute", "budget_ms": 0.01, "ray_count": 0},
            {"mode": "scene-bvh", "budget_ms": 0.01, "ray_count": 5},
        ]
        gates = analyze_hazard.evaluate_gates(
            trials, grouped, mappings, [], 1.0)
        status = {row["gate"]: row["status"] for row in gates}
        self.assertEqual("FAIL", status["ray_advantage"])

    def test_analysis_handles_zero_rays_and_measured_convergence(self):
        out_dir = os.path.join(ROOT, "tests", "generated_hazard_analysis")
        self.addCleanup(lambda: shutil.rmtree(out_dir, ignore_errors=True))
        result = analyze_hazard.analyze(
            os.path.join(FIXTURES, "hazard_trials.csv"),
            os.path.join(FIXTURES, "hazard_timing.csv"),
            out_dir, make_plots=False)
        mappings = {(row["mode"], row["budget_ms"]): row["ray_count"]
                    for row in result["mappings"]}
        self.assertEqual(361, mappings["true-brute", 11.0])
        self.assertEqual(361, mappings["scene-bvh", 11.0])
        status = {row["gate"]: row["status"] for row in result["gates"]}
        self.assertEqual("PASS", status["zero_ray_collisions"])
        self.assertEqual("PASS", status["convergence"])
        self.assertTrue(os.path.isfile(os.path.join(out_dir, "hazard_budget_mapping.csv")))

    def test_cli_fails_when_an_acceptance_gate_fails(self):
        argv = ["analyze_hazard.py", "--trials", "trials.csv",
                "--timing", "timing.csv"]
        result = {"gates": [{"gate": "example", "status": "FAIL", "detail": "failed"}]}
        with mock.patch.object(sys, "argv", argv), \
                mock.patch.object(analyze_hazard, "analyze", return_value=result):
            self.assertEqual(1, analyze_hazard.main())


if __name__ == "__main__":
    unittest.main()
