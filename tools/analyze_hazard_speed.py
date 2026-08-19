#!/usr/bin/env python3
"""Analyze the fixed-design speed operating-envelope hazard benchmark."""

import argparse
import csv
import math
import os
import random
import sys
from collections import defaultdict

import analyze_hazard as shared


RAW_FIELDS = (
    "mode", "scenario_id", "speed_mps", "initial_clearance", "hazard_diameter",
    "lateral_offset", "azimuth_phase", "ray_count", "control", "outcome",
    "detected", "detection_range", "unbraked_ttc", "stopping_margin",
    "collision_speed", "hit_flags_equal", "object_ids_equal", "ranges_equal",
)
SPEEDS = (2.0, 3.0, 4.0)
CLEARANCES = (4.0, 8.0)
DIAMETERS = (0.10, 0.25, 0.50)
OFFSETS = (-0.30, -0.15, 0.0, 0.15, 0.30)
PHASE_INDICES = tuple(range(32))
PHASES = tuple(index / 32.0 for index in PHASE_INDICES)
SCENARIOS_PER_SPEED = 960
TOTAL_SCENARIOS = 2880
FIXED_CONTROL = "fixed_scan"
CONTROLS = (FIXED_CONTROL, "first_frame_braking")
ADJACENT_MODE_PAIRS = (
    ("true-brute", "mesh-bvh"),
    ("mesh-bvh", "scene-bvh"),
)
METRICS = (
    ("safe_stop_rate", "Safe-stop rate",
     lambda row: float(row["outcome"] == "SafeStop")),
    ("detection_range", "Detection range", lambda row: row["detection_range"]),
    ("unbraked_ttc_all_scenarios", "All-scenario unbraked TTC",
     lambda row: row["unbraked_ttc"] or 0.0),
    ("stopping_margin", "Stopping margin", lambda row: row["stopping_margin"]),
    ("collision_speed", "Collision speed", lambda row: row["collision_speed"]),
    ("undetected_collision_rate", "Undetected-collision rate",
     lambda row: float(row["outcome"] == "Collision" and not row["detected"])),
)
TABLE_FILENAMES = {
    "summary": "hazard_speed_summary.csv",
    "fixed_ray_curve": "hazard_speed_fixed_ray_curve.csv",
    "paired_differences": "hazard_speed_paired_differences.csv",
    "base_timing_mapping": "hazard_speed_base_timing_mapping.csv",
    "budget_mapping": "hazard_speed_budget_mapping.csv",
    "budget_attribution": "hazard_speed_budget_attribution.csv",
    "adverse_outcomes": "hazard_speed_adverse_outcomes.csv",
    "acceleration_conclusions": "hazard_speed_acceleration_conclusions.csv",
    "acceptance_gates": "hazard_speed_acceptance_gates.csv",
}
PLOT_FILENAMES = {
    "budget": "hazard_speed_safe_stop_by_budget.png",
    "fixed_ray": "hazard_speed_fixed_ray_safety.png",
}
EPSILON = 1e-12


def _number(value, path, line, column, allow_empty=False):
    return shared._number(value, path, line, column, allow_empty)


def _tuple_key(row):
    return (row["initial_clearance"], row["hazard_diameter"],
            row["lateral_offset"], row["azimuth_phase"])


def _scenario_id(speed_index, clearance_index, diameter_index, offset_index,
                 phase_index):
    return (((((speed_index * 2 + clearance_index) * 3 + diameter_index) * 5
              + offset_index) * 32) + phase_index)


def _expected_scenarios(speed_index):
    return {
        _scenario_id(speed_index, clearance, diameter, offset, phase)
        for clearance in range(2)
        for diameter in range(3)
        for offset in range(5)
        for phase in range(32)
    }


def read_trials(path):
    """Read the exact mode-keyed speed-trial schema without coercing its design."""
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if tuple(reader.fieldnames or ()) != RAW_FIELDS:
            raise shared.ValidationError(
                f"{path}: trial columns must be exactly " + ",".join(RAW_FIELDS))
        rows, keys = [], set()
        for line, raw in enumerate(reader, start=2):
            mode = (raw["mode"] or "").strip()
            outcome = (raw["outcome"] or "").strip()
            control = (raw["control"] or "").strip()
            if mode not in shared.EXPECTED_MODES:
                raise shared.ValidationError(f"{path}:{line}: unknown mode {mode!r}")
            if outcome not in ("SafeStop", "Collision"):
                raise shared.ValidationError(
                    f"{path}:{line}: outcome must be 'SafeStop' or 'Collision'")
            if control not in CONTROLS:
                raise shared.ValidationError(
                    f"{path}:{line}: control must be 'fixed_scan' or "
                    "'first_frame_braking'")
            values = {}
            for name in ("scenario_id", "speed_mps", "initial_clearance",
                         "hazard_diameter", "lateral_offset", "azimuth_phase",
                         "ray_count"):
                values[name] = _number(raw[name], path, line, name)
            for name in ("scenario_id", "ray_count"):
                if values[name] < 0 or values[name] != int(values[name]):
                    raise shared.ValidationError(
                        f"{path}:{line}: {name} must be a non-negative integer")
            phase_index = round(values["azimuth_phase"] * 32)
            if (phase_index not in PHASE_INDICES or not math.isclose(
                    values["azimuth_phase"], phase_index / 32.0,
                    rel_tol=0.0, abs_tol=1e-12)):
                raise shared.ValidationError(
                    f"{path}:{line}: azimuth_phase must be phase_index/32 for "
                    "phase_index 0 through 31")
            if values["ray_count"] not in shared.EXPECTED_RAY_COUNTS:
                raise shared.ValidationError(
                    f"{path}:{line}: ray_count must be one of "
                    + ",".join(map(str, shared.EXPECTED_RAY_COUNTS)))
            if control == "first_frame_braking" and values["ray_count"] != 361:
                raise shared.ValidationError(
                    f"{path}:{line}: first_frame_braking must use ray_count 361")
            key = (mode, int(values["scenario_id"]), int(values["ray_count"]), control)
            if key in keys:
                raise shared.ValidationError(f"{path}:{line}: duplicate trial {key}")
            keys.add(key)
            row = {
                "mode": mode, "scenario_id": int(values["scenario_id"]),
                "speed_mps": values["speed_mps"],
                "initial_clearance": values["initial_clearance"],
                "hazard_diameter": values["hazard_diameter"],
                "lateral_offset": values["lateral_offset"],
                "azimuth_phase": values["azimuth_phase"], "phase_index": phase_index,
                "ray_count": int(values["ray_count"]), "control": control,
                "outcome": outcome, "detected": shared._boolean(
                    raw["detected"], path, line, "detected"),
            }
            for name in ("detection_range", "unbraked_ttc", "stopping_margin",
                         "collision_speed"):
                row[name] = _number(raw[name], path, line, name, allow_empty=True)
            for name in ("hit_flags_equal", "object_ids_equal", "ranges_equal"):
                row[name] = shared._boolean(raw[name], path, line, name)
            if not row["detected"]:
                for name in ("detection_range", "unbraked_ttc", "stopping_margin"):
                    row[name] = None
            if row["detected"] and any(row[name] is None for name in (
                    "detection_range", "unbraked_ttc", "stopping_margin")):
                raise shared.ValidationError(
                    f"{path}:{line}: detected trials need detection measurements")
            if outcome == "Collision" and row["collision_speed"] is None:
                raise shared.ValidationError(
                    f"{path}:{line}: collision trials need collision_speed")
            rows.append(row)
    if not rows:
        raise shared.ValidationError(f"{path}: CSV has no trial rows")
    return rows


def validate_trials(rows):
    """Require the complete 86,400-row speed design and full tracer parity."""
    if len(rows) != 86400:
        raise shared.ValidationError("speed trial CSV must contain exactly 86400 rows")
    expected_tuples = {
        (clearance, diameter, offset, phase)
        for clearance in CLEARANCES for diameter in DIAMETERS
        for offset in OFFSETS for phase in PHASES
    }
    by_speed = defaultdict(list)
    for row in rows:
        if row["speed_mps"] not in SPEEDS:
            raise shared.ValidationError("speed_mps must be exactly 2.0, 3.0, or 4.0")
        if (row["initial_clearance"] not in CLEARANCES or
                row["hazard_diameter"] not in DIAMETERS or
                row["lateral_offset"] not in OFFSETS or
                row["azimuth_phase"] not in PHASES):
            raise shared.ValidationError("trial row has a value outside the declared design")
        by_speed[row["speed_mps"]].append(row)
        if not all(row[name] for name in (
                "hit_flags_equal", "object_ids_equal", "ranges_equal")):
            raise shared.ValidationError(
                f"tracer parity failed for scenario {row['scenario_id']}")

    common_tuples = None
    fixed_sets = {}
    for speed_index, speed in enumerate(SPEEDS):
        speed_rows = by_speed[speed]
        if len(speed_rows) != 28800:
            raise shared.ValidationError(f"{speed:g} m/s must contain exactly 28800 rows")
        scenario_rows = defaultdict(list)
        for row in speed_rows:
            scenario_rows[row["scenario_id"]].append(row)
            expected_id = _scenario_id(
                speed_index, CLEARANCES.index(row["initial_clearance"]),
                DIAMETERS.index(row["hazard_diameter"]),
                OFFSETS.index(row["lateral_offset"]), row["phase_index"])
            if row["scenario_id"] != expected_id:
                raise shared.ValidationError(
                    f"scenario_id {row['scenario_id']} does not match its encoded tuple")
        if set(scenario_rows) != _expected_scenarios(speed_index):
            raise shared.ValidationError(
                f"{speed:g} m/s scenario IDs must be its contiguous 960-ID range")
        tuple_set = {_tuple_key(rows[0]) for rows in scenario_rows.values()}
        if tuple_set != expected_tuples or len(tuple_set) != SCENARIOS_PER_SPEED:
            raise shared.ValidationError(f"{speed:g} m/s must contain all 960 tuples")
        if common_tuples is None:
            common_tuples = tuple_set
        elif tuple_set != common_tuples:
            raise shared.ValidationError("speeds do not have identical non-speed tuples")
        for scenario_id, scenario in scenario_rows.items():
            if len(scenario) != 30:
                raise shared.ValidationError(
                    f"scenario {scenario_id} must have 30 mode/ray/control rows")
            keys = {(row["mode"], row["ray_count"], row["control"]) for row in scenario}
            expected = {
                (mode, ray_count, control)
                for mode in shared.EXPECTED_MODES
                for ray_count in shared.EXPECTED_RAY_COUNTS
                for control in (FIXED_CONTROL,)
            }
            expected |= {(mode, 361, "first_frame_braking")
                         for mode in shared.EXPECTED_MODES}
            if keys != expected:
                raise shared.ValidationError(
                    f"scenario {scenario_id} lacks exact mode/ray/control pairing")
        for mode in shared.EXPECTED_MODES:
            for ray_count in shared.EXPECTED_RAY_COUNTS:
                fixed_sets[speed, mode, ray_count] = {
                    row["scenario_id"] for row in speed_rows
                    if row["mode"] == mode and row["ray_count"] == ray_count
                    and row["control"] == FIXED_CONTROL
                }
                if len(fixed_sets[speed, mode, ray_count]) != SCENARIOS_PER_SPEED:
                    raise shared.ValidationError(
                        f"{speed:g} m/s {mode} {ray_count} rays is not fully paired")
    return fixed_sets


def read_base_timing(path):
    """Read only the published base timing schema and select its 1x/120 slice."""
    rows = shared.read_timing(path)
    expected = {
        (complexity, mode, ray)
        for complexity in shared.EXPECTED_COMPLEXITIES
        for mode in shared.EXPECTED_MODES
        for ray in shared.EXPECTED_RAY_COUNTS
    }
    actual = {(row["complexity"], row["mode"], row["ray_count"]) for row in rows}
    if len(rows) != 81 or actual != expected:
        raise shared.ValidationError(
            "base timing CSV must contain the complete published 81-row design")
    for row in rows:
        if row["triangles_per_mesh"] != shared.EXPECTED_TRIANGLES_PER_MESH[
                row["complexity"]]:
            raise shared.ValidationError("published timing triangle counts are inconsistent")
    selected = [row for row in rows if row["complexity"] == "1x"]
    if len(selected) != 27 or any(row["triangles_per_mesh"] != 120 for row in selected):
        raise shared.ValidationError("timing slice must be complexity=1x and triangles=120")
    return selected


def base_timing_mapping(timing):
    """Map standard and one measured convergence budget once for every speed."""
    slowest = max(
        row["p95_ms"] for row in timing if row["ray_count"] == 361)
    budgets = [(value, "standard") for value in shared.STANDARD_BUDGETS_MS]
    budgets.append((1.1 * slowest, "measured_110pct_slowest_361_p95"))
    result = []
    for mode in shared.EXPECTED_MODES:
        candidates = [row for row in timing if row["mode"] == mode]
        for budget, kind in budgets:
            fits = [row for row in candidates if row["p95_ms"] <= budget]
            selected = max(fits, key=lambda row: row["ray_count"]) if fits else None
            result.append({
                "timing_complexity": "1x", "object_count": 100,
                "triangles_per_mesh": 120, "mode": mode, "budget_ms": budget,
                "budget_kind": kind, "ray_count": (
                    selected["ray_count"] if selected else 0),
                "selected_p95_ms": selected["p95_ms"] if selected else None,
                "convergence_source": (
                    "measured_110pct_slowest_361_p95" if kind != "standard" else ""),
            })
    return result


def _mean(rows, metric):
    values = [metric(row) for row in rows]
    values = [value for value in values if value is not None]
    return (sum(values) / len(values), len(values)) if values else (None, 0)


def fixed_rows(rows):
    return [row for row in rows if row["control"] == FIXED_CONTROL]


def summarize(rows):
    grouped = defaultdict(list)
    for row in fixed_rows(rows):
        grouped[row["speed_mps"], row["mode"], row["ray_count"]].append(row)
    summary = []
    for key, values in sorted(grouped.items()):
        speed, mode, ray_count = key
        for metric, label, function in METRICS:
            estimate, measured = _mean(values, function)
            summary.append({
                "speed_mps": speed, "mode": mode, "ray_count": ray_count,
                "metric": metric, "metric_label": label, "estimate": estimate,
                "n_scenarios": len(values), "n_measured": measured,
            })
    return summary, grouped


def fixed_ray_curve(grouped):
    """Return exactly one curve per speed after proving fixed-ray outcomes match."""
    curves = []
    for speed in SPEEDS:
        for ray_count in shared.EXPECTED_RAY_COUNTS:
            reference = {
                row["scenario_id"]: row["outcome"]
                for row in grouped[speed, shared.EXPECTED_MODES[0], ray_count]
            }
            for mode in shared.EXPECTED_MODES[1:]:
                current = {
                    row["scenario_id"]: row["outcome"]
                    for row in grouped[speed, mode, ray_count]
                }
                if current != reference:
                    raise shared.ValidationError(
                        f"fixed-ray outcomes differ at {speed:g} m/s {ray_count} rays")
            curves.append({
                "speed_mps": speed, "ray_count": ray_count,
                "safe_stop_rate": sum(value == "SafeStop" for value in reference.values())
                / len(reference),
            })
    return curves


def paired_bootstrap_difference(left, right, metric, key=_tuple_key,
                                samples=400, seed=20260819):
    """Return deterministic paired mean(left-right) and percentile 95% CI."""
    left_by_key = {key(row): metric(row) for row in left}
    right_by_key = {key(row): metric(row) for row in right}
    if left_by_key.keys() != right_by_key.keys():
        raise shared.ValidationError("paired comparison received different tuple sets")
    differences = [left_by_key[item] - right_by_key[item]
                   for item in sorted(left_by_key)]
    estimate = sum(differences) / len(differences)
    if all(value == differences[0] for value in differences[1:]):
        return estimate, estimate, estimate, len(differences)
    rng, n, bootstrap = random.Random(seed), len(differences), []
    for _ in range(samples):
        bootstrap.append(sum(differences[rng.randrange(n)] for _ in range(n)) / n)
    bootstrap.sort()
    return (estimate, shared._quantile(bootstrap, .025),
            shared._quantile(bootstrap, .975), n)


def _mapped_by_key(mapping):
    return {(row["speed_mps"], row["mode"], row["budget_ms"]): row
            for row in mapping}


def budget_mapping(base_mapping, grouped):
    safe_rates = {
        (speed, mode, ray): _mean(rows, lambda row: row["outcome"] == "SafeStop")[0]
        for (speed, mode, ray), rows in grouped.items()
    }
    rows = []
    for speed in SPEEDS:
        for base in base_mapping:
            rows.append({
                **base, "speed_mps": speed,
                "safe_stop_rate": safe_rates[speed, base["mode"], base["ray_count"]],
            })
    return rows


def paired_differences(grouped, mappings):
    """Compare mapped acceleration and all paired adjacent-speed fixed-ray outcomes."""
    rows = []
    mapping = _mapped_by_key(mappings)
    safe = lambda row: float(row["outcome"] == "SafeStop")
    for speed in SPEEDS:
        for budget in sorted({row["budget_ms"] for row in mappings}):
            for from_mode, to_mode in ADJACENT_MODE_PAIRS:
                left = grouped[speed, to_mode, mapping[speed, to_mode, budget]["ray_count"]]
                right = grouped[speed, from_mode, mapping[speed, from_mode, budget]["ray_count"]]
                estimate, low, high, n = paired_bootstrap_difference(left, right, safe)
                rows.append({
                    "comparison_kind": "within_speed_acceleration", "speed_mps": speed,
                    "lower_speed_mps": None, "upper_speed_mps": None,
                    "budget_ms": budget, "transition": f"{from_mode}_to_{to_mode}",
                    "left_mode": to_mode, "left_ray_count": mapping[speed, to_mode, budget]["ray_count"],
                    "right_mode": from_mode, "right_ray_count": mapping[speed, from_mode, budget]["ray_count"],
                    "metric": "safe_stop_rate", "difference": estimate,
                    "ci_low": low, "ci_high": high, "n_paired": n,
                })
    for lower, upper in zip(SPEEDS, SPEEDS[1:]):
        for budget in sorted({row["budget_ms"] for row in mappings}):
            for mode in shared.EXPECTED_MODES:
                left = grouped[upper, mode, mapping[upper, mode, budget]["ray_count"]]
                right = grouped[lower, mode, mapping[lower, mode, budget]["ray_count"]]
                estimate, low, high, n = paired_bootstrap_difference(left, right, safe)
                rows.append({
                    "comparison_kind": "adjacent_speed", "speed_mps": None,
                    "lower_speed_mps": lower, "upper_speed_mps": upper,
                    "budget_ms": budget, "transition": "",
                    "left_mode": mode, "left_ray_count": mapping[upper, mode, budget]["ray_count"],
                    "right_mode": mode, "right_ray_count": mapping[lower, mode, budget]["ray_count"],
                    "metric": "safe_stop_rate", "difference": estimate,
                    "ci_low": low, "ci_high": high, "n_paired": n,
                })
        for ray_count in shared.EXPECTED_RAY_COUNTS:
            for mode in shared.EXPECTED_MODES:
                left = grouped[upper, mode, ray_count]
                right = grouped[lower, mode, ray_count]
                estimate, low, high, n = paired_bootstrap_difference(left, right, safe)
                rows.append({
                    "comparison_kind": "adjacent_speed_fixed_ray", "speed_mps": None,
                    "lower_speed_mps": lower, "upper_speed_mps": upper,
                    "budget_ms": None, "transition": "",
                    "left_mode": mode, "left_ray_count": ray_count,
                    "right_mode": mode, "right_ray_count": ray_count,
                    "metric": "safe_stop_rate", "difference": estimate,
                    "ci_low": low, "ci_high": high, "n_paired": n,
                })
    return rows


def budget_attribution(mappings, differences):
    mapping = _mapped_by_key(mappings)
    difference = {
        (row["speed_mps"], row["budget_ms"], row["transition"]): row
        for row in differences if row["comparison_kind"] == "within_speed_acceleration"
    }
    rows = []
    for speed in SPEEDS:
        for budget in sorted({row["budget_ms"] for row in mappings}):
            for from_mode, to_mode in ADJACENT_MODE_PAIRS:
                baseline, accelerated = (mapping[speed, from_mode, budget],
                                         mapping[speed, to_mode, budget])
                paired = difference[speed, budget, f"{from_mode}_to_{to_mode}"]
                rows.append({
                    "speed_mps": speed, "timing_complexity": "1x",
                    "object_count": 100, "triangles_per_mesh": 120,
                    "budget_ms": budget, "budget_kind": baseline["budget_kind"],
                    "transition": f"{from_mode}_to_{to_mode}",
                    "from_mode": from_mode, "to_mode": to_mode,
                    "from_ray_count": baseline["ray_count"],
                    "to_ray_count": accelerated["ray_count"],
                    "ray_count_gain": accelerated["ray_count"] - baseline["ray_count"],
                    "from_safe_stop_rate": baseline["safe_stop_rate"],
                    "to_safe_stop_rate": accelerated["safe_stop_rate"],
                    "safe_stop_rate_gain": paired["difference"],
                    "ci_low": paired["ci_low"], "ci_high": paired["ci_high"],
                    "n_paired": paired["n_paired"],
                })
    return rows


def _within_status(row):
    if row["ray_count_gain"] < 0 or row["safe_stop_rate_gain"] < -EPSILON:
        return "reverses"
    if row["ray_count_gain"] == 0 and abs(row["safe_stop_rate_gain"]) <= EPSILON:
        return "disappears"
    if abs(row["safe_stop_rate_gain"]) <= EPSILON:
        return "narrows"
    return "persists"


def acceleration_conclusions(attribution):
    """Classify observed gains; never turn an adverse measurement into a benefit."""
    rows = []
    indexed = {(row["speed_mps"], row["budget_ms"], row["transition"]): row
               for row in attribution}
    for row in attribution:
        rows.append({
            "scope": "within_speed", "speed_mps": row["speed_mps"],
            "lower_speed_mps": None, "upper_speed_mps": None,
            "budget_ms": row["budget_ms"], "transition": row["transition"],
            "status": _within_status(row), "previous_safe_stop_rate_gain": None,
            "safe_stop_rate_gain": row["safe_stop_rate_gain"],
            "ray_count_gain": row["ray_count_gain"],
            "adverse": row["safe_stop_rate_gain"] < -EPSILON or row["ray_count_gain"] < 0,
            "classification_rule": (
                "reverses=negative ray or safety gain; disappears=both gains zero; "
                "narrows=zero safety gain with positive rays; persists=otherwise"),
        })
    for lower, upper in zip(SPEEDS, SPEEDS[1:]):
        for budget in sorted({row["budget_ms"] for row in attribution}):
            for from_mode, to_mode in ADJACENT_MODE_PAIRS:
                transition = f"{from_mode}_to_{to_mode}"
                before, after = indexed[lower, budget, transition], indexed[upper, budget, transition]
                previous, current = before["safe_stop_rate_gain"], after["safe_stop_rate_gain"]
                if current < -EPSILON:
                    status = "reverses"
                elif abs(current) <= EPSILON:
                    status = "disappears"
                elif previous > EPSILON and current < previous - EPSILON:
                    status = "narrows"
                else:
                    status = "persists"
                rows.append({
                    "scope": "across_adjacent_speeds", "speed_mps": None,
                    "lower_speed_mps": lower, "upper_speed_mps": upper,
                    "budget_ms": budget, "transition": transition, "status": status,
                    "previous_safe_stop_rate_gain": previous,
                    "safe_stop_rate_gain": current,
                    "ray_count_gain": after["ray_count_gain"],
                    "adverse": current < -EPSILON or after["ray_count_gain"] < 0,
                    "classification_rule": (
                        "reverses=current negative safety gain; disappears=current zero; "
                        "narrows=positive current gain below prior; persists=otherwise"),
                })
    return rows


def adverse_outcomes(summary, attribution, conclusions):
    """Retain observed fixed-ray collision outcomes alongside adverse acceleration."""
    rows = [
        {**row, "source": "budget_attribution", "status": _within_status(row)}
        for row in attribution
        if row["safe_stop_rate_gain"] < -EPSILON or row["ray_count_gain"] < 0
    ]
    rows.extend({
        **row, "source": "acceleration_conclusion"
    } for row in conclusions if row["status"] == "reverses")
    rows.extend({
        "source": "fixed_ray_summary",
        "status": f"observed_{row['metric']}",
        "speed_mps": row["speed_mps"],
        "mode": row["mode"],
        "ray_count": row["ray_count"],
        "metric": row["metric"],
        "estimate": row["estimate"],
        "n_scenarios": row["n_scenarios"],
        "n_measured": row["n_measured"],
        "adverse": (
            row["metric"] == "collision_speed" and
            row["estimate"] is not None and row["estimate"] > EPSILON
        ) or (
            row["metric"] == "undetected_collision_rate" and
            row["estimate"] > 0
        ),
    } for row in summary if row["metric"] in (
        "collision_speed", "undetected_collision_rate"))
    return rows


def _gate(name, status, detail):
    return {"gate": name, "status": status, "detail": detail}


def acceptance_gates(rows, grouped, curves, mappings):
    gates = [
        _gate("trial_row_count", "PASS" if len(rows) == 86400 else "FAIL",
              f"{len(rows)}/86400 rows"),
        _gate("scenario_coverage_and_common_pairing", "PASS",
              "2880 global scenarios; 960 tuples per speed with common pairing"),
        _gate("excluded_4mps_2m_control", "PASS",
              "2 m clearance is outside the predeclared common domain; "
              "hazard_speed_trial proves 4 m/s reaches contact at the "
              "first-frame braking endpoint"),
        _gate("tracer_parity", "PASS", "all hit flags, object IDs, and ranges match"),
        _gate("fixed_ray_mode_independence", "PASS",
              f"{len(curves)} speed/ray mode-independent curve points"),
    ]
    for speed in SPEEDS:
        for mode in shared.EXPECTED_MODES:
            zero = grouped[speed, mode, 0]
            zero_ok = all(row["outcome"] == "Collision" for row in zero)
            gates.append(_gate(
                f"zero_ray_collisions_{speed:g}_{mode}", "PASS" if zero_ok else "FAIL",
                f"{sum(row['outcome'] == 'Collision' for row in zero)}/{len(zero)} collided"))
            full = grouped[speed, mode, 361]
            full_rate = sum(row["outcome"] == "SafeStop" for row in full) / len(full)
            gates.append(_gate(
                f"full_resolution_safe_stop_{speed:g}_{mode}",
                "PASS" if full_rate >= .99 else "FAIL", f"{full_rate:.3%}"))
            control = [row for row in rows if row["speed_mps"] == speed and
                       row["mode"] == mode and row["control"] == "first_frame_braking"]
            rate = sum(row["outcome"] == "SafeStop" for row in control) / len(control)
            gates.append(_gate(
                f"first_frame_safe_stop_{speed:g}_{mode}",
                "PASS" if rate >= .99 else "FAIL", f"{rate:.3%}"))
            failures = []
            for metric, _, function in METRICS[:3]:
                if metric not in ("safe_stop_rate", "unbraked_ttc_all_scenarios"):
                    continue
                values = [(ray, _mean(grouped[speed, mode, ray], function)[0])
                          for ray in shared.EXPECTED_RAY_COUNTS]
                failures.extend(
                    f"{metric} {previous_ray}->{ray}"
                    for (previous_ray, previous), (ray, current) in zip(values, values[1:])
                    if current < previous - EPSILON)
            gates.append(_gate(
                f"monotonic_safety_and_ttc_{speed:g}_{mode}",
                "FAIL" if failures else "PASS",
                "; ".join(failures) or "safe-stop rate and all-scenario TTC nondecreasing"))
    mapping_fields = (
        "mode", "budget_ms", "budget_kind", "ray_count", "selected_p95_ms",
        "timing_complexity", "object_count", "triangles_per_mesh",
        "convergence_source",
    )
    by_base = {
        tuple(row[field] for field in mapping_fields)
        for row in mappings if row["speed_mps"] == SPEEDS[0]
    }
    shared_mapping = all(
        {tuple(row[field] for field in mapping_fields)
         for row in mappings if row["speed_mps"] == speed} == by_base
        for speed in SPEEDS)
    gates.append(_gate("shared_timing_mapping", "PASS" if shared_mapping else "FAIL",
                       "identical budget kind, selected p95, ray, and 1x/100/120 "
                       "timing metadata cross-applied to every speed"))
    gates.append(_gate("full_reporting", "PASS",
                       "summary, curves, pairs, timing, attribution, adverse, conclusions, gates"))
    return gates


def _write_csv(path, rows):
    rows = list(rows)
    if not rows:
        # The adverse table is intentionally permitted to be empty but still exact.
        fields = ("source", "status")
    else:
        fields = tuple(rows[0])
        if any(tuple(row) != fields for row in rows):
            fields = tuple(dict.fromkeys(field for row in rows for field in row))
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file, fieldnames=fields, extrasaction="raise",
            lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def render_plots(mappings, curves, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise shared.ValidationError(
            "plotting requires matplotlib; install the repository's existing "
            "plotting dependency or pass --no-plots") from exc

    figure, axes = plt.subplots(1, len(SPEEDS), figsize=(15, 4), sharey=True)
    for axis, speed in zip(axes, SPEEDS):
        for mode in shared.EXPECTED_MODES:
            points = [row for row in mappings if row["speed_mps"] == speed and
                      row["mode"] == mode]
            points.sort(key=lambda row: row["budget_ms"])
            axis.plot([row["budget_ms"] for row in points],
                      [row["safe_stop_rate"] for row in points],
                      marker=shared.STYLE[mode]["marker"], label=mode)
        axis.set_title(f"{speed:g} m/s")
        axis.set_xlabel("Budget (ms)")
        axis.set_xscale("symlog", linthresh=.5)
    axes[0].set_ylabel("Actual safe-stop rate")
    axes[-1].legend()
    figure.tight_layout()
    figure.savefig(os.path.join(out_dir, PLOT_FILENAMES["budget"]), dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(1, len(SPEEDS), figsize=(15, 4), sharey=True)
    for axis, speed in zip(axes, SPEEDS):
        points = [row for row in curves if row["speed_mps"] == speed]
        axis.plot([row["ray_count"] for row in points],
                  [row["safe_stop_rate"] for row in points], marker="o")
        axis.set_title(f"{speed:g} m/s")
        axis.set_xlabel("Fixed ray count")
    axes[0].set_ylabel("Mode-independent safe-stop rate")
    figure.tight_layout()
    figure.savefig(os.path.join(out_dir, PLOT_FILENAMES["fixed_ray"]), dpi=160)
    plt.close(figure)


def analyze(trials_path, timing_path, out_dir, make_plots=True):
    rows = read_trials(trials_path)
    validate_trials(rows)
    timing = read_base_timing(timing_path)
    base_mapping = base_timing_mapping(timing)
    summary, grouped = summarize(rows)
    curves = fixed_ray_curve(grouped)
    mappings = budget_mapping(base_mapping, grouped)
    differences = paired_differences(grouped, mappings)
    attribution = budget_attribution(mappings, differences)
    conclusions = acceleration_conclusions(attribution)
    adverse = adverse_outcomes(summary, attribution, conclusions)
    gates = acceptance_gates(rows, grouped, curves, mappings)
    os.makedirs(out_dir, exist_ok=True)
    for key, value in {
            "summary": summary, "fixed_ray_curve": curves,
            "paired_differences": differences, "base_timing_mapping": base_mapping,
            "budget_mapping": mappings, "budget_attribution": attribution,
            "adverse_outcomes": adverse, "acceleration_conclusions": conclusions,
            "acceptance_gates": gates}.items():
        _write_csv(os.path.join(out_dir, TABLE_FILENAMES[key]), value)
    if make_plots:
        render_plots(mappings, curves, out_dir)
    return {
        "summary": summary, "fixed_ray_curve": curves, "paired_differences": differences,
        "base_timing_mapping": base_mapping, "mappings": mappings,
        "budget_attribution": attribution, "adverse_outcomes": adverse,
        "acceleration_conclusions": conclusions, "gates": gates,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", required=True)
    parser.add_argument("--timing", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    try:
        result = analyze(args.trials, args.timing, args.out_dir,
                         make_plots=not args.no_plots)
    except (OSError, shared.ValidationError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    failed = [row for row in result["gates"] if row["status"] == "FAIL"]
    if failed:
        print("acceptance gates failed: " + ", ".join(row["gate"] for row in failed),
              file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
