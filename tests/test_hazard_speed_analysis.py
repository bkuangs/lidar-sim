#!/usr/bin/env python3
import csv
import importlib.util
import os
import shutil
import sys
import tempfile
import unittest


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import analyze_hazard_speed as analysis


class HazardSpeedAnalysisTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp_dir = tempfile.mkdtemp(prefix=".hazard-speed-", dir=ROOT)
        cls.trials_path = os.path.join(cls.temp_dir, "speed_trials.csv")
        cls.timing_path = os.path.join(ROOT, "tests", "fixtures", "hazard_timing.csv")
        cls.rows = synthetic_rows()
        write_trials(cls.trials_path, cls.rows)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.temp_dir, ignore_errors=True)

    def analyze(self, plots=False):
        out_dir = os.path.join(self.temp_dir, "analysis")
        if not plots and hasattr(self.__class__, "result"):
            return self.__class__.result
        shutil.rmtree(out_dir, ignore_errors=True)
        result = analysis.analyze(
            self.trials_path, self.timing_path, out_dir, make_plots=plots)
        if not plots:
            self.__class__.result = result
        return result

    def test_exact_schema_completeness_uniqueness_and_pairing(self):
        rows = analysis.read_trials(self.trials_path)
        fixed_sets = analysis.validate_trials(rows)
        self.assertEqual(86400, len(rows))
        self.assertEqual(analysis.TOTAL_SCENARIOS, len({
            row["scenario_id"] for row in rows}))
        self.assertEqual(81, len(fixed_sets))
        self.assertTrue(all(len(value) == 960 for value in fixed_sets.values()))
        self.assertTrue(all(
            row["control"] == "fixed_scan" for row in analysis.fixed_rows(rows)))
        self.assertEqual(31 / 32, next(
            row["azimuth_phase"] for row in rows if row["phase_index"] == 31))

        duplicate = [dict(row) for row in self.rows]
        duplicate.append(dict(duplicate[0]))
        path = os.path.join(self.temp_dir, "duplicate.csv")
        write_trials(path, duplicate)
        with self.assertRaisesRegex(Exception, "duplicate trial"):
            analysis.read_trials(path)

        missing = [dict(row) for row in self.rows[:-1]]
        path = os.path.join(self.temp_dir, "missing.csv")
        write_trials(path, missing)
        with self.assertRaisesRegex(Exception, "exactly 86400"):
            analysis.validate_trials(analysis.read_trials(path))

        wrong_schema = os.path.join(self.temp_dir, "wrong-schema.csv")
        with open(wrong_schema, "w", newline="", encoding="utf-8") as csv_file:
            csv.DictWriter(csv_file, fieldnames=analysis.RAW_FIELDS[:-1]).writeheader()
        with self.assertRaisesRegex(Exception, "columns must be exactly"):
            analysis.read_trials(wrong_schema)

        invalid_phase = [dict(row) for row in self.rows]
        invalid_phase[0]["azimuth_phase"] = .1
        path = os.path.join(self.temp_dir, "invalid-phase.csv")
        write_trials(path, invalid_phase)
        with self.assertRaisesRegex(Exception, "phase_index/32"):
            analysis.read_trials(path)

    def test_timing_slice_and_shared_mapping_are_exact(self):
        timing = analysis.read_base_timing(self.timing_path)
        self.assertEqual(27, len(timing))
        base = analysis.base_timing_mapping(timing)
        self.assertEqual(18, len(base))
        self.assertTrue(all(
            row["timing_complexity"] == "1x" and row["object_count"] == 100 and
            row["triangles_per_mesh"] == 120 for row in base))
        result = self.analyze()
        mappings = result["mappings"]
        for speed in analysis.SPEEDS[1:]:
            self.assertEqual(
                {(row["mode"], row["budget_ms"], row["budget_kind"],
                  row["ray_count"], row["selected_p95_ms"],
                  row["timing_complexity"], row["object_count"],
                  row["triangles_per_mesh"], row["convergence_source"])
                 for row in mappings if row["speed_mps"] == analysis.SPEEDS[0]},
                {(row["mode"], row["budget_ms"], row["budget_kind"],
                  row["ray_count"], row["selected_p95_ms"],
                  row["timing_complexity"], row["object_count"],
                  row["triangles_per_mesh"], row["convergence_source"])
                 for row in mappings if row["speed_mps"] == speed})
        gate = {row["gate"]: row["status"] for row in result["gates"]}
        self.assertEqual("PASS", gate["shared_timing_mapping"])

        incomplete = [dict(row) for row in timing][:-1]
        with self.assertRaisesRegex(Exception, "complete published 81-row design"):
            # The selected slice cannot substitute for the full published design.
            analysis.read_base_timing(write_timing(
                os.path.join(self.temp_dir, "incomplete-timing.csv"),
                incomplete))

    def test_monotonicity_failures_are_rejected_by_acceptance_gates(self):
        rows = [dict(row) for row in analysis.read_trials(self.trials_path)]
        for row in rows:
            if (row["speed_mps"] == 2.0 and row["mode"] == "true-brute" and
                    row["control"] == "fixed_scan" and row["ray_count"] == 5):
                row["outcome"] = "SafeStop"
                row["collision_speed"] = None
            if (row["speed_mps"] == 2.0 and row["mode"] == "true-brute" and
                    row["control"] == "fixed_scan" and row["ray_count"] == 9):
                row["outcome"] = "Collision"
                row["collision_speed"] = 1.0
        _, grouped = analysis.summarize(rows)
        gates = analysis.acceptance_gates(
            rows, grouped, [{"speed_mps": speed, "ray_count": ray,
                             "safe_stop_rate": 0.0}
                            for speed in analysis.SPEEDS
                            for ray in analysis.shared.EXPECTED_RAY_COUNTS],
            self.analyze()["mappings"])
        status = {row["gate"]: row["status"] for row in gates}
        self.assertEqual("FAIL", status["monotonic_safety_and_ttc_2_true-brute"])

    def test_budget_attribution_paired_ci_and_reversed_outcomes(self):
        result = self.analyze()
        attribution = result["budget_attribution"]
        self.assertEqual(36, len(attribution))
        self.assertTrue(all(row["n_paired"] == 960 for row in attribution))
        self.assertTrue(all(row["ci_low"] <= row["safe_stop_rate_gain"] <= row["ci_high"]
                            for row in attribution))
        self.assertEqual(36, len([
            row for row in result["paired_differences"]
            if row["comparison_kind"] == "adjacent_speed"]))
        fixed_ray_pairs = [
            row for row in result["paired_differences"]
            if row["comparison_kind"] == "adjacent_speed_fixed_ray"]
        self.assertEqual(54, len(fixed_ray_pairs))
        self.assertTrue(all(
            row["left_ray_count"] == row["right_ray_count"] and
            row["n_paired"] == 960 for row in fixed_ray_pairs))

        adverse = dict(attribution[0])
        adverse["ray_count_gain"] = -4
        adverse["safe_stop_rate_gain"] = -.1
        self.assertEqual("reverses", analysis._within_status(adverse))
        retained = analysis.adverse_outcomes([], [adverse], [])
        self.assertTrue(any(row["status"] == "reverses" for row in retained))
        observed = [
            row for row in result["adverse_outcomes"]
            if row["source"] == "fixed_ray_summary"]
        self.assertEqual(3 * 3 * 9 * 2, len(observed))
        self.assertEqual(
            {"collision_speed", "undetected_collision_rate"},
            {row["metric"] for row in observed})
        self.assertTrue(all(
            row["status"] == f"observed_{row['metric']}" for row in observed))
        summary_outcomes = {
            (row["speed_mps"], row["mode"], row["ray_count"], row["metric"]):
                (row["estimate"], row["n_measured"])
            for row in result["summary"] if row["metric"] in (
                "collision_speed", "undetected_collision_rate")
        }
        self.assertEqual(summary_outcomes, {
            (row["speed_mps"], row["mode"], row["ray_count"], row["metric"]):
                (row["estimate"], row["n_measured"])
            for row in observed
        })

    def test_exact_table_and_plot_contracts(self):
        result = self.analyze()
        out_dir = os.path.join(self.temp_dir, "analysis")
        self.assertEqual(set(analysis.TABLE_FILENAMES.values()), {
            name for name in os.listdir(out_dir) if name.endswith(".csv")})
        self.assertEqual(3 * 3 * 9 * 6, len(result["summary"]))
        self.assertEqual(27, len(result["fixed_ray_curve"]))
        self.assertEqual(126, len(result["paired_differences"]))
        self.assertEqual(60, len(result["acceleration_conclusions"]))
        self.assertTrue(all("mode" not in row for row in result["fixed_ray_curve"]))
        self.assertTrue(all(row["status"] == "PASS" for row in result["gates"]))

        if importlib.util.find_spec("matplotlib"):
            self.analyze(plots=True)
            self.assertEqual(set(analysis.PLOT_FILENAMES.values()), {
                name for name in os.listdir(out_dir) if name.endswith(".png")})


def write_trials(path, rows):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=analysis.RAW_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_timing(path, rows):
    fields = (
        "complexity", "triangles_per_mesh", "mode", "ray_count",
        "median_ms", "p95_ms")
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def synthetic_rows():
    rows = []
    for speed_index, speed in enumerate(analysis.SPEEDS):
        for clearance_index, clearance in enumerate(analysis.CLEARANCES):
            for diameter_index, diameter in enumerate(analysis.DIAMETERS):
                for offset_index, offset in enumerate(analysis.OFFSETS):
                    for phase_index, phase in enumerate(analysis.PHASES):
                        scenario_id = analysis._scenario_id(
                            speed_index, clearance_index, diameter_index,
                            offset_index, phase_index)
                        for mode in analysis.shared.EXPECTED_MODES:
                            for ray in analysis.shared.EXPECTED_RAY_COUNTS:
                                safe = ray >= 361 or (ray > 0 and
                                    phase_index < min(31, ray // 12 + speed_index))
                                rows.append(trial_row(
                                    mode, scenario_id, speed, clearance, diameter,
                                    offset, phase, ray, "fixed_scan", safe))
                            rows.append(trial_row(
                                mode, scenario_id, speed, clearance, diameter,
                                offset, phase, 361, "first_frame_braking", True))
    return rows


def trial_row(mode, scenario_id, speed, clearance, diameter, offset, phase, ray,
              control, safe):
    detected = ray > 0
    return {
        "mode": mode, "scenario_id": scenario_id, "speed_mps": speed,
        "initial_clearance": clearance, "hazard_diameter": diameter,
        "lateral_offset": offset, "azimuth_phase": phase, "ray_count": ray,
        "control": control, "outcome": "SafeStop" if safe else "Collision",
        "detected": str(detected).lower(),
        "detection_range": clearance if detected else "",
        "unbraked_ttc": clearance / speed if detected else "",
        "stopping_margin": .1 + ray / 1000 if detected else "",
        "collision_speed": "" if safe else speed,
        "hit_flags_equal": "true", "object_ids_equal": "true",
        "ranges_equal": "true",
    }


if __name__ == "__main__":
    unittest.main()
