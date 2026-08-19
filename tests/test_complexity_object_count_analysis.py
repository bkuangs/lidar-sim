#!/usr/bin/env python3
import csv
import importlib.util
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import analyze_complexity_object_count as analysis
import analyze_hazard


TRIALS = os.path.join(ROOT, "tests", "fixtures", "hazard_trials.csv")


def synthetic_timing_rows():
    rows = []
    mode_factor = {
        "true-brute": 4.0,
        "mesh-bvh": 1.0,
        "scene-bvh": 0.5,
    }
    for complexity in analysis.EXPECTED_COMPLEXITIES:
        triangles = analysis.EXPECTED_TRIANGLES_PER_MESH[complexity]
        complexity_factor = triangles / 120
        for object_count in analysis.EXPECTED_OBJECT_COUNTS:
            for mode in analyze_hazard.EXPECTED_MODES:
                for ray_count in analyze_hazard.EXPECTED_RAY_COUNTS:
                    p95_ms = (
                        0.01 + mode_factor[mode] *
                        (0.001 * complexity_factor +
                         object_count / 20000.0) *
                        (1.0 + ray_count / 40.0))
                    rows.append({
                        "complexity": complexity,
                        "object_count": object_count,
                        "triangles_per_mesh": triangles,
                        "mode": mode,
                        "ray_count": ray_count,
                        "median_ms": 0.8 * p95_ms,
                        "p95_ms": p95_ms,
                    })
    return rows


def write_timing(path, rows, fields=analysis.RAW_FIELDS):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fixed_trials():
    return [
        row for row in analyze_hazard.read_trials(TRIALS)
        if row.get("control", "").lower() != "first_frame_braking"
    ]


class ComplexityObjectCountAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp(
            prefix=".complexity-object-count-", dir=ROOT)
        self.addCleanup(
            lambda: shutil.rmtree(self.temp_dir, ignore_errors=True))
        self.timing_path = os.path.join(self.temp_dir, "timing.csv")
        write_timing(self.timing_path, synthetic_timing_rows())

    def analyze(self, **kwargs):
        return analysis.analyze(
            TRIALS, self.timing_path,
            os.path.join(self.temp_dir, "analysis"),
            make_plots=False, **kwargs)

    def test_exact_completeness_extra_missing_duplicate_and_factor_pairing(self):
        timing = analysis.read_timing(self.timing_path)
        analysis.validate_coverage(fixed_trials(), timing)
        self.assertEqual(243, len(timing))

        with self.assertRaisesRegex(
                analyze_hazard.ValidationError, "exactly 243"):
            analysis.validate_coverage(fixed_trials(), timing[:-1])

        extra = synthetic_timing_rows()
        added = dict(extra[0])
        added["complexity"] = "64x"
        extra.append(added)
        write_timing(self.timing_path, extra)
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError, "exactly 243"):
            analysis.validate_coverage(
                fixed_trials(), analysis.read_timing(self.timing_path))

        duplicate = synthetic_timing_rows()
        added = dict(duplicate[0])
        added["triangles_per_mesh"] = 999
        duplicate.append(added)
        write_timing(self.timing_path, duplicate)
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError, "duplicate timing row"):
            analysis.read_timing(self.timing_path)

        wrong_pair = synthetic_timing_rows()
        wrong_pair[-1]["triangles_per_mesh"] = 480
        write_timing(self.timing_path, wrong_pair)
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "16x timing rows must have exactly 1920"):
            analysis.validate_coverage(
                fixed_trials(), analysis.read_timing(self.timing_path))

        extra_fields = (*analysis.RAW_FIELDS, "unexpected")
        write_timing(self.timing_path, synthetic_timing_rows(), extra_fields)
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "timing columns must be exactly"):
            analysis.read_timing(self.timing_path)

    def test_triangle_attribution_convergence_mapping_and_curve_reuse(self):
        result = self.analyze()
        self.assertEqual(9, len(result["convergence_budgets"]))
        self.assertEqual(162, len(result["mappings"]))
        self.assertEqual(9, len(result["robustness"]))
        for row in result["mappings"]:
            self.assertEqual(
                analysis.EXPECTED_TRIANGLES_PER_MESH[row["complexity"]],
                row["triangles_per_mesh"])
        convergence = [
            row for row in result["mappings"]
            if row["budget_kind"] == "convergence"
        ]
        self.assertEqual(27, len(convergence))
        self.assertTrue(all(row["ray_count"] == 361 for row in convergence))

        rates = {}
        for row in result["mappings"]:
            rates.setdefault(row["ray_count"], set()).add(
                row["safe_stop_rate"])
        self.assertTrue(all(len(values) == 1 for values in rates.values()))
        self.assertEqual(9, len(result["safe_curve"]))
        status = {row["gate"]: row["status"] for row in result["gates"]}
        for gate in (
                "matrix_cell_integrity", "tracer_parity",
                "timing_methodology", "timing_completeness",
                "fixed_ray_safety_curve"):
            self.assertEqual("PASS", status[gate])
        for complexity in analysis.EXPECTED_COMPLEXITIES:
            for count in analysis.EXPECTED_OBJECT_COUNTS:
                self.assertEqual(
                    "PASS", status[f"convergence_{complexity}_{count}"])

    def test_both_transition_attribution_counts_and_keys(self):
        result = self.analyze()
        timing = result["timing_attribution"]
        self.assertEqual(162, len(timing))
        keys = {
            (row["complexity"], row["object_count"],
             row["transition"], row["ray_count"])
            for row in timing
        }
        self.assertEqual(162, len(keys))
        self.assertEqual(
            {"true-brute_to_mesh-bvh", "mesh-bvh_to_scene-bvh"},
            {row["transition"] for row in timing})
        self.assertEqual(108, len(result["budget_attribution"]))
        self.assertTrue(all(row["n_paired"] == 2
                            for row in result["budget_attribution"]))

    def test_adverse_corners_for_each_transition_are_descriptive(self):
        rows = synthetic_timing_rows()
        corners = (
            ("1x", 25, 5, "true-brute", "mesh-bvh"),
            ("16x", 400, 361, "mesh-bvh", "scene-bvh"),
        )
        for complexity, count, ray, from_mode, to_mode in corners:
            baseline = next(
                row for row in rows
                if row["complexity"] == complexity
                and row["object_count"] == count
                and row["mode"] == from_mode
                and row["ray_count"] == ray)
            accelerated = next(
                row for row in rows
                if row["complexity"] == complexity
                and row["object_count"] == count
                and row["mode"] == to_mode
                and row["ray_count"] == ray)
            accelerated["p95_ms"] = baseline["p95_ms"]
        write_timing(self.timing_path, rows)
        result = self.analyze()
        exceptions = result["robustness_exceptions"]
        self.assertEqual(2, len(exceptions))
        self.assertEqual(
            {"true-brute_to_mesh-bvh", "mesh-bvh_to_scene-bvh"},
            {row["transition"] for row in exceptions})
        self.assertTrue(all(row["comparison_result"] == "EQUAL"
                            for row in exceptions))
        self.assertFalse(any(
            row["status"] == "FAIL"
            for row in result["gates"]
            if row["gate"].startswith("descriptive_result_")))

    def test_inversions_preserve_all_three_declared_axes(self):
        rows = synthetic_timing_rows()
        by_key = {
            (row["complexity"], row["object_count"],
             row["mode"], row["ray_count"]): row
            for row in rows
        }
        by_key["1x", 25, "mesh-bvh", 9]["p95_ms"] = (
            by_key["1x", 25, "mesh-bvh", 5]["p95_ms"] - .001)
        by_key["4x", 100, "scene-bvh", 361]["p95_ms"] = (
            by_key["1x", 100, "scene-bvh", 361]["p95_ms"] - .001)
        by_key["16x", 400, "true-brute", 257]["p95_ms"] = (
            by_key["16x", 100, "true-brute", 257]["p95_ms"] - .001)
        inversions = analysis.timing_inversions(rows)
        self.assertEqual(
            {"ray_count", "complexity", "object_count"},
            {row["axis"] for row in inversions})
        self.assertTrue(any(
            row["axis"] == "ray_count"
            and row["lower_coordinate"] == 5
            and row["higher_coordinate"] == 9
            for row in inversions))
        self.assertTrue(any(
            row["axis"] == "complexity"
            and row["lower_coordinate"] == "1x"
            and row["higher_coordinate"] == "4x"
            for row in inversions))
        self.assertTrue(any(
            row["axis"] == "object_count"
            and row["lower_coordinate"] == 100
            and row["higher_coordinate"] == 400
            for row in inversions))

    def test_acceptance_failure_and_exact_output_contract(self):
        result = self.analyze(convergence_budget_ms=0.0)
        failures = [
            row for row in result["gates"] if row["status"] == "FAIL"
        ]
        self.assertEqual(9, len(failures))
        self.assertTrue(all(
            row["gate"].startswith("convergence_") for row in failures))
        out_dir = os.path.join(self.temp_dir, "analysis")
        self.assertEqual(
            set(analysis.TABLE_FILENAMES.values()),
            {name for name in os.listdir(out_dir) if name.endswith(".csv")})
        with mock.patch.object(sys, "argv", [
                "analyze_complexity_object_count.py",
                "--trials", TRIALS,
                "--timing", self.timing_path,
                "--out-dir", os.path.join(self.temp_dir, "cli"),
                "--convergence-budget-ms", "0",
                "--no-plots"]):
            self.assertEqual(1, analysis.main())

    @unittest.skipUnless(
        importlib.util.find_spec("matplotlib"), "matplotlib unavailable")
    def test_exact_plot_filename_contract(self):
        out_dir = os.path.join(self.temp_dir, "plot-analysis")
        result = analysis.analyze(
            TRIALS, self.timing_path, out_dir, make_plots=True)
        primary = analysis.primary_heatmap_values(
            result["timing_attribution"])
        detail = analysis.detail_heatmap_values(
            result["timing_attribution"])
        self.assertTrue(all(
            len(values) == 3 and all(len(row) == 3 for row in values)
            for values in primary.values()))
        self.assertTrue(all(
            len(values) == 9 and all(len(row) == 8 for row in values)
            for values in detail.values()))
        timing_by_key = {
            (row["complexity"], row["object_count"], row["transition"]):
                row["p95_speedup"]
            for row in result["timing_attribution"]
            if row["ray_count"] == 361
        }
        for transition, values in primary.items():
            for complexity_index, complexity in enumerate(
                    analysis.EXPECTED_COMPLEXITIES):
                for count_index, object_count in enumerate(
                        analysis.EXPECTED_OBJECT_COUNTS):
                    self.assertEqual(
                        timing_by_key[complexity, object_count, transition],
                        values[complexity_index][count_index])
        self.assertEqual(
            {analysis.PLOT_FILENAME, analysis.DETAIL_PLOT_FILENAME},
            {name for name in os.listdir(out_dir) if name.endswith(".png")})


if __name__ == "__main__":
    unittest.main()
