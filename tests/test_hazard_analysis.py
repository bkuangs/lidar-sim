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
        by_key = {
            (row["complexity"], row["mode"], row["budget_ms"]): row["ray_count"]
            for row in mappings
        }
        self.assertEqual(0, by_key["1x", "true-brute", 0.01])
        self.assertEqual(5, by_key["1x", "true-brute", 0.5])
        self.assertEqual(9, by_key["1x", "mesh-bvh", 0.5])
        self.assertEqual(17, by_key["1x", "scene-bvh", 0.5])
        self.assertEqual(361, by_key["1x", "scene-bvh", 5.0])
        self.assertEqual(0, by_key["16x", "true-brute", 5.0])
        self.assertEqual(129, by_key["16x", "mesh-bvh", 5.0])
        self.assertEqual(257, by_key["16x", "scene-bvh", 5.0])

    def test_expected_coverage_requires_complete_complexity_mode_ray_matrix(self):
        trials = analyze_hazard.read_trials(os.path.join(FIXTURES, "hazard_trials.csv"))
        timing = analyze_hazard.read_timing(os.path.join(FIXTURES, "hazard_timing.csv"))
        analyze_hazard.validate_expected_coverage(trials, timing)
        self.assertEqual(81, len(timing))
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError, "timing modes must be exactly"):
            analyze_hazard.validate_expected_coverage(
                trials, [row for row in timing if row["mode"] != "mesh-bvh"])
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "timing complexities must be exactly"):
            analyze_hazard.validate_expected_coverage(
                trials, [row for row in timing if row["complexity"] != "16x"])
        wrong_triangles = [dict(row) for row in timing]
        wrong_triangles[-1]["triangles_per_mesh"] = 1919
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "16x timing rows must have exactly 1920"):
            analyze_hazard.validate_expected_coverage(trials, wrong_triangles)

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
            {"complexity": complexity, "mode": "true-brute",
             "budget_ms": 0.01, "ray_count": 0}
            for complexity in analyze_hazard.EXPECTED_COMPLEXITIES
        ] + [
            {"complexity": complexity, "mode": "scene-bvh",
             "budget_ms": 0.01, "ray_count": 5}
            for complexity in analyze_hazard.EXPECTED_COMPLEXITIES
        ]
        gates = analyze_hazard.evaluate_gates(
            trials, grouped, mappings, [], {
                complexity: (1.0, "test")
                for complexity in analyze_hazard.EXPECTED_COMPLEXITIES
            })
        status = {row["gate"]: row["status"] for row in gates}
        self.assertEqual("FAIL", status["ray_advantage_1x"])

    def test_analysis_handles_zero_rays_and_per_complexity_convergence(self):
        out_dir = os.path.join(ROOT, "tests", "generated_hazard_analysis")
        self.addCleanup(lambda: shutil.rmtree(out_dir, ignore_errors=True))
        result = analyze_hazard.analyze(
            os.path.join(FIXTURES, "hazard_trials.csv"),
            os.path.join(FIXTURES, "hazard_timing.csv"),
            out_dir, make_plots=False)
        mappings = {(row["complexity"], row["mode"], row["budget_ms"]): row["ray_count"]
                    for row in result["mappings"]}
        for complexity, budget in (("1x", 11.0), ("4x", 44.0), ("16x", 176.0)):
            self.assertEqual(361, mappings[complexity, "true-brute", budget])
            self.assertEqual(361, mappings[complexity, "mesh-bvh", budget])
            self.assertEqual(361, mappings[complexity, "scene-bvh", budget])
        status = {row["gate"]: row["status"] for row in result["gates"]}
        self.assertEqual("PASS", status["zero_ray_collisions"])
        self.assertTrue(all(
            status[f"convergence_{complexity}"] == "PASS"
            for complexity in analyze_hazard.EXPECTED_COMPLEXITIES))
        self.assertTrue(os.path.isfile(os.path.join(out_dir, "hazard_budget_mapping.csv")))
        self.assertTrue(os.path.isfile(
            os.path.join(out_dir, "hazard_timing_attribution.csv")))
        self.assertTrue(os.path.isfile(
            os.path.join(out_dir, "hazard_budget_attribution.csv")))
        self.assertTrue(os.path.isfile(
            os.path.join(out_dir, "hazard_complexity_robustness.csv")))

    def test_attribution_covers_both_layers_at_every_ray_and_budget(self):
        out_dir = os.path.join(ROOT, "tests", "generated_hazard_analysis")
        self.addCleanup(lambda: shutil.rmtree(out_dir, ignore_errors=True))
        result = analyze_hazard.analyze(
            os.path.join(FIXTURES, "hazard_trials.csv"),
            os.path.join(FIXTURES, "hazard_timing.csv"),
            out_dir,
            make_plots=False)
        transitions = {
            "true-brute_to_mesh-bvh",
            "mesh-bvh_to_scene-bvh",
        }
        timing_by_transition = {}
        for row in result["timing_attribution"]:
            timing_by_transition.setdefault(
                (row["complexity"], row["transition"]), set()).add(
                row["ray_count"])
        self.assertEqual(
            {
                (complexity, transition)
                for complexity in analyze_hazard.EXPECTED_COMPLEXITIES
                for transition in transitions
            },
            set(timing_by_transition))
        self.assertTrue(all(
            ray_counts == set(analyze_hazard.EXPECTED_RAY_COUNTS)
            for ray_counts in timing_by_transition.values()))

        budgets = {}
        for row in result["mappings"]:
            budgets.setdefault(row["complexity"], set()).add(row["budget_ms"])
        budget_by_transition = {}
        for row in result["budget_attribution"]:
            budget_by_transition.setdefault(
                (row["complexity"], row["transition"]), set()).add(
                row["budget_ms"])
            self.assertAlmostEqual(
                row["to_safe_stop_rate"] - row["from_safe_stop_rate"],
                row["safe_stop_rate_gain"])
        self.assertTrue(all(
            budget_by_transition[complexity, transition] == budgets[complexity]
            for complexity in analyze_hazard.EXPECTED_COMPLEXITIES
            for transition in transitions))

    def test_budget_plot_data_matches_mappings_rates_and_paired_intervals(self):
        out_dir = os.path.join(ROOT, "tests", "generated_hazard_analysis")
        self.addCleanup(lambda: shutil.rmtree(out_dir, ignore_errors=True))
        result = analyze_hazard.analyze(
            os.path.join(FIXTURES, "hazard_trials.csv"),
            os.path.join(FIXTURES, "hazard_timing.csv"),
            out_dir, make_plots=False)
        mapped_rates, differences = analyze_hazard.prepare_budget_plot_data(
            result["summary"], result["mappings"], result["budget_differences"])

        summary_rates = {
            (row["mode"], row["ray_count"]): row["estimate"]
            for row in result["summary"] if row["metric"] == "safe_stop_rate"
        }
        for point in mapped_rates:
            self.assertEqual(
                summary_rates[point["mode"], point["ray_count"]],
                point["safe_stop_rate"])

        by_budget = {}
        for point in mapped_rates:
            by_budget.setdefault(
                (point["complexity"], point["budget_ms"]), {}
            )[point["mode"]] = point
        source_differences = {
            row["comparison"]: row for row in result["budget_differences"]
            if row["metric"] == "safe_stop_rate"
        }
        for difference in differences:
            source = source_differences[difference["comparison"]]
            self.assertEqual(source["ci_low"], difference["ci_low"])
            self.assertEqual(source["ci_high"], difference["ci_high"])
            points = by_budget[
                difference["complexity"], difference["plot_budget_ms"]]
            actual = (points[difference["left_mode"]]["safe_stop_rate"] -
                      points[difference["right_mode"]]["safe_stop_rate"])
            self.assertAlmostEqual(difference["difference"], actual)

    def test_every_complexity_maps_to_the_same_fixed_ray_safety_curve(self):
        out_dir = os.path.join(ROOT, "tests", "generated_hazard_analysis")
        self.addCleanup(lambda: shutil.rmtree(out_dir, ignore_errors=True))
        result = analyze_hazard.analyze(
            os.path.join(FIXTURES, "hazard_trials.csv"),
            os.path.join(FIXTURES, "hazard_timing.csv"),
            out_dir, make_plots=False)
        rates = {}
        for row in result["mappings"]:
            key = (row["mode"], row["ray_count"])
            rates.setdefault(key, set()).add(row["safe_stop_rate"])
        self.assertTrue(all(len(values) == 1 for values in rates.values()))
        for row in result["mappings"]:
            if row["budget_ms"] in analyze_hazard.STANDARD_BUDGETS_MS:
                self.assertIsNotNone(row["ray_count_change_vs_1x"])
                self.assertIsNotNone(row["safe_stop_rate_change_vs_1x"])

    def test_robustness_result_lists_every_adverse_measurement(self):
        timing = analyze_hazard.read_timing(
            os.path.join(FIXTURES, "hazard_timing.csv"))
        trials = analyze_hazard.read_trials(
            os.path.join(FIXTURES, "hazard_trials.csv"))
        summary, _ = analyze_hazard.summarize_trials(trials)
        mappings = analyze_hazard.enrich_budget_mappings(
            summary,
            analyze_hazard.map_budgets(
                timing, analyze_hazard.STANDARD_BUDGETS_MS))
        adverse = [dict(row) for row in timing]
        for ray_count, p95_ms in ((5, 7.0), (17, 31.0)):
            target = next(
                row for row in adverse
                if row["complexity"] == "16x" and
                row["mode"] == "mesh-bvh" and
                row["ray_count"] == ray_count)
            target["p95_ms"] = p95_ms
        robustness = analyze_hazard.robustness_summary(adverse, mappings)
        exceptions = analyze_hazard.robustness_exceptions(adverse)
        by_complexity = {row["complexity"]: row for row in robustness}
        self.assertEqual("NO", by_complexity["16x"]["complexity_result"])
        self.assertEqual("NO", robustness[0]["overall_robustness_result"])
        self.assertEqual([5, 17], [row["ray_count"] for row in exceptions])
        self.assertTrue(all(
            row["mesh_bvh_p95_ms"] >= row["true_brute_p95_ms"]
            for row in exceptions))

    def test_p95_inversions_are_reported_without_changing_discrete_mapping(self):
        timing = analyze_hazard.read_timing(
            os.path.join(FIXTURES, "hazard_timing.csv"))
        noisy = [dict(row) for row in timing]
        row_65 = next(
            row for row in noisy
            if row["complexity"] == "1x" and
            row["mode"] == "true-brute" and row["ray_count"] == 65)
        row_129 = next(
            row for row in noisy
            if row["complexity"] == "1x" and
            row["mode"] == "true-brute" and row["ray_count"] == 129)
        row_65["p95_ms"] = 5.5
        row_129["p95_ms"] = 4.5
        inversions = analyze_hazard.timing_p95_inversions(noisy)
        self.assertEqual(1, len(inversions))
        self.assertEqual(65, inversions[0]["lower_ray_count"])
        self.assertEqual(129, inversions[0]["higher_ray_count"])
        mappings = analyze_hazard.map_budgets(noisy, (5.0,))
        mapped = next(
            row for row in mappings
            if row["complexity"] == "1x" and row["mode"] == "true-brute")
        self.assertEqual(129, mapped["ray_count"])

    def test_fixed_ray_plot_data_is_one_mode_independent_curve(self):
        trials = analyze_hazard.read_trials(os.path.join(FIXTURES, "hazard_trials.csv"))
        summary, grouped = analyze_hazard.summarize_trials(trials)
        curve = analyze_hazard.prepare_fixed_ray_safety(summary, grouped)
        expected_ray_counts = sorted({
            row["ray_count"] for row in summary
            if row["metric"] == "safe_stop_rate"
        })
        self.assertEqual(expected_ray_counts, [row["ray_count"] for row in curve])
        self.assertTrue(all("mode" not in row for row in curve))

    def test_fixed_ray_plot_rejects_paired_mismatch_with_equal_aggregate_rate(self):
        trials = analyze_hazard.read_trials(os.path.join(FIXTURES, "hazard_trials.csv"))
        summary, grouped = analyze_hazard.summarize_trials(trials)
        mismatched = {
            key: [dict(row) for row in rows]
            for key, rows in grouped.items()
        }
        scene_rows = mismatched["scene-bvh", 9]
        safe_stop = next(row for row in scene_rows if row["outcome"] == "SafeStop")
        collision = next(row for row in scene_rows if row["outcome"] == "Collision")
        safe_stop["outcome"], collision["outcome"] = (
            collision["outcome"], safe_stop["outcome"])
        self.assertEqual(
            sum(row["outcome"] == "SafeStop" for row in grouped["scene-bvh", 9]),
            sum(row["outcome"] == "SafeStop" for row in scene_rows))

        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "fixed-ray outcomes differ for 2 paired scenarios"):
            analyze_hazard.prepare_fixed_ray_safety(summary, mismatched)

    def test_fixed_ray_plot_rejects_duplicate_scenario_row(self):
        trials = analyze_hazard.read_trials(os.path.join(FIXTURES, "hazard_trials.csv"))
        summary, grouped = analyze_hazard.summarize_trials(trials)
        duplicated = {
            key: [dict(row) for row in rows]
            for key, rows in grouped.items()
        }
        duplicated["scene-bvh", 9].append(dict(duplicated["scene-bvh", 9][0]))

        with self.assertRaisesRegex(
                analyze_hazard.ValidationError, "rows contain duplicate scenario IDs"):
            analyze_hazard.prepare_fixed_ray_safety(summary, duplicated)

    def test_plot_output_contract_replaces_difference_and_ray_images(self):
        self.assertEqual(
            {
                "hazard_budget_safe_stop_and_rays.png",
                "hazard_safety_by_rays.png",
                "hazard_undetected_collisions.png",
            },
            set(analyze_hazard.PLOT_FILENAMES.values()))
        self.assertNotIn(
            "hazard_budget_safety_difference.png",
            analyze_hazard.PLOT_FILENAMES.values())
        self.assertNotIn(
            "hazard_budget_rays.png",
            analyze_hazard.PLOT_FILENAMES.values())

    def test_cli_fails_when_an_acceptance_gate_fails(self):
        argv = ["analyze_hazard.py", "--trials", "trials.csv",
                "--timing", "timing.csv"]
        result = {"gates": [{"gate": "example", "status": "FAIL", "detail": "failed"}]}
        with mock.patch.object(sys, "argv", argv), \
                mock.patch.object(analyze_hazard, "analyze", return_value=result):
            self.assertEqual(1, analyze_hazard.main())


if __name__ == "__main__":
    unittest.main()
