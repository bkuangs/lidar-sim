#!/usr/bin/env python3
import csv
import os
import shutil
import sys
import tempfile
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import analyze_hazard
import analyze_object_count


TRIALS = os.path.join(ROOT, "tests", "fixtures", "hazard_trials.csv")
FIELDS = (
    "object_count", "triangles_per_mesh", "mode", "ray_count",
    "median_ms", "p95_ms",
)


def synthetic_timing_rows():
    rows = []
    mode_factor = {
        "true-brute": 4.0,
        "mesh-bvh": 1.0,
        "scene-bvh": 0.55,
    }
    for object_count in analyze_object_count.EXPECTED_OBJECT_COUNTS:
        for mode in analyze_hazard.EXPECTED_MODES:
            for ray_count in analyze_hazard.EXPECTED_RAY_COUNTS:
                p95_ms = (
                    0.02 +
                    mode_factor[mode] *
                    (0.03 + object_count / 10000.0) *
                    (1.0 + ray_count / 40.0))
                rows.append({
                    "object_count": object_count,
                    "triangles_per_mesh": 120,
                    "mode": mode,
                    "ray_count": ray_count,
                    "median_ms": 0.8 * p95_ms,
                    "p95_ms": p95_ms,
                })
    return rows


def write_timing(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class ObjectCountAnalysisTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.mkdtemp(prefix="object-count-analysis-")
        self.addCleanup(lambda: shutil.rmtree(self.temp_dir, ignore_errors=True))
        self.timing_path = os.path.join(self.temp_dir, "timing.csv")
        write_timing(self.timing_path, synthetic_timing_rows())

    def test_schema_requires_complete_unique_120_triangle_matrix(self):
        trials = analyze_hazard.read_trials(TRIALS)
        trials = [
            row for row in trials
            if row.get("control") != "first_frame_braking"
        ]
        timing = analyze_object_count.read_timing(self.timing_path)
        analyze_object_count.validate_coverage(trials, timing)
        self.assertEqual(135, len(timing))

        missing = [
            row for row in timing if row["object_count"] != 400
        ]
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "timing object counts must be exactly"):
            analyze_object_count.validate_coverage(trials, missing)

        wrong_triangles = [dict(row) for row in timing]
        wrong_triangles[-1]["triangles_per_mesh"] = 121
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError,
                "must have exactly 120 triangles"):
            analyze_object_count.validate_coverage(
                trials, wrong_triangles)

        duplicated = synthetic_timing_rows()
        duplicated.append(dict(duplicated[0]))
        write_timing(self.timing_path, duplicated)
        with self.assertRaisesRegex(
                analyze_hazard.ValidationError, "duplicate timing row"):
            analyze_object_count.read_timing(self.timing_path)

    def test_analysis_maps_one_safety_curve_and_converges_per_count(self):
        out_dir = os.path.join(self.temp_dir, "analysis")
        result = analyze_object_count.analyze(
            TRIALS, self.timing_path, out_dir, make_plots=False)
        self.assertEqual(45, len(result["timing_attribution"]))
        self.assertEqual(30, len(result["budget_attribution"]))

        rates = {}
        for row in result["mappings"]:
            rates.setdefault(row["ray_count"], set()).add(
                row["safe_stop_rate"])
        self.assertTrue(all(len(values) == 1 for values in rates.values()))

        status = {row["gate"]: row["status"] for row in result["gates"]}
        for object_count in analyze_object_count.EXPECTED_OBJECT_COUNTS:
            self.assertEqual(
                "PASS", status[f"convergence_{object_count}"])
        self.assertTrue(all(
            row["object_count_result"] ==
            "IMPROVED_AT_ALL_NONZERO_RAYS"
            for row in result["robustness"]))
        self.assertTrue(os.path.isfile(os.path.join(
            out_dir, "hazard_object_count_budget_mapping.csv")))
        self.assertTrue(os.path.isfile(os.path.join(
            out_dir, "hazard_object_count_robustness.csv")))

    def test_budget_mapping_reports_zero_and_safe_stop_gain(self):
        trials = [
            row for row in analyze_hazard.read_trials(TRIALS)
            if row.get("control") != "first_frame_braking"
        ]
        summary, grouped = analyze_hazard.summarize_trials(trials)
        safe_curve = analyze_hazard.prepare_fixed_ray_safety(
            summary, grouped)
        tiny_budgets = {
            object_count: (0.001, "test")
            for object_count in analyze_object_count.EXPECTED_OBJECT_COUNTS
        }
        tiny_mappings = analyze_object_count.map_budgets(
            analyze_object_count.read_timing(self.timing_path),
            tiny_budgets, safe_curve)
        zero = next(
            row for row in tiny_mappings
            if row["object_count"] == 400 and
            row["mode"] == "true-brute" and
            row["budget_ms"] == 0.001)
        self.assertEqual(0, zero["ray_count"])
        self.assertIsNone(zero["selected_p95_ms"])

        result = analyze_object_count.analyze(
            TRIALS, self.timing_path,
            os.path.join(self.temp_dir, "analysis"),
            make_plots=False)
        for row in result["budget_attribution"]:
            self.assertAlmostEqual(
                row["scene_bvh_safe_stop_rate"] -
                row["mesh_bvh_safe_stop_rate"],
                row["safe_stop_rate_gain"])

    def test_robustness_lists_every_scene_bvh_regression(self):
        rows = synthetic_timing_rows()
        for ray_count in (5, 17):
            scene = next(
                row for row in rows
                if row["object_count"] == 100 and
                row["mode"] == "scene-bvh" and
                row["ray_count"] == ray_count)
            mesh = next(
                row for row in rows
                if row["object_count"] == 100 and
                row["mode"] == "mesh-bvh" and
                row["ray_count"] == ray_count)
            scene["p95_ms"] = mesh["p95_ms"] + 0.01
        write_timing(self.timing_path, rows)
        result = analyze_object_count.analyze(
            TRIALS, self.timing_path,
            os.path.join(self.temp_dir, "analysis"),
            make_plots=False)
        count_result = next(
            row for row in result["robustness"]
            if row["object_count"] == 100)
        self.assertEqual(
            "MIXED_OR_REGRESSED",
            count_result["object_count_result"])
        self.assertEqual(
            [5, 17],
            [row["ray_count"]
             for row in result["robustness_exceptions"]])

    def test_inversions_report_ray_and_object_axes(self):
        rows = synthetic_timing_rows()
        lower_ray = next(
            row for row in rows
            if row["object_count"] == 25 and
            row["mode"] == "mesh-bvh" and row["ray_count"] == 5)
        higher_ray = next(
            row for row in rows
            if row["object_count"] == 25 and
            row["mode"] == "mesh-bvh" and row["ray_count"] == 9)
        higher_ray["p95_ms"] = lower_ray["p95_ms"] - 0.001

        lower_count = next(
            row for row in rows
            if row["object_count"] == 200 and
            row["mode"] == "scene-bvh" and row["ray_count"] == 361)
        higher_count = next(
            row for row in rows
            if row["object_count"] == 400 and
            row["mode"] == "scene-bvh" and row["ray_count"] == 361)
        higher_count["p95_ms"] = lower_count["p95_ms"] - 0.001

        inversions = analyze_object_count.timing_inversions(rows)
        axes = {row["axis"] for row in inversions}
        self.assertEqual({"ray_count", "object_count"}, axes)
        self.assertTrue(any(
            row["axis"] == "ray_count" and
            row["object_count"] == 25 and
            row["lower_coordinate"] == 5 and
            row["higher_coordinate"] == 9
            for row in inversions))
        self.assertTrue(any(
            row["axis"] == "object_count" and
            row["ray_count"] == 361 and
            row["lower_coordinate"] == 200 and
            row["higher_coordinate"] == 400
            for row in inversions))


if __name__ == "__main__":
    unittest.main()
