#!/usr/bin/env python3
"""Aggregate and plot fixed-ray hazard benchmark CSVs.

Usage:
  python3 tools/analyze_hazard.py --trials plots/hazard_trials.csv \
      --timing plots/hazard_timing.csv --out-dir plots/hazard_analysis

Required trial columns are ``mode``, ``scenario_id``, ``ray_count``, ``outcome``,
``detected``, ``detection_range``, ``unbraked_ttc``, ``stopping_margin``, and
``collision_speed``. Required timing columns are ``mode``, ``ray_count``,
``median_ms``, and ``p95_ms``. Empty outcome-specific measurements are allowed.
Optional ``control`` (or ``trial_kind``) rows identify first-frame-braking
controls; optional parity booleans are checked when emitted.
"""
import argparse
import csv
import math
import os
import random
import sys
from collections import defaultdict


STANDARD_BUDGETS_MS = (0.5, 1.0, 2.0, 5.0, 8.0)
METRICS = (
    ("safe_stop_rate", "Safe-stop rate", lambda row: float(row["outcome"] == "SafeStop")),
    ("detection_range", "Detection range", lambda row: row["detection_range"]),
    ("unbraked_ttc", "Unbraked TTC", lambda row: row["unbraked_ttc"]),
    ("stopping_margin", "Stopping margin", lambda row: row["stopping_margin"]),
    ("collision_speed", "Collision speed", lambda row: row["collision_speed"]),
    ("undetected_collision_rate", "Undetected-collision rate",
     lambda row: float(row["outcome"] == "Collision" and not row["detected"])),
)
TRIAL_ALIASES = {
    "mode": ("mode",),
    "scenario_id": ("scenario_id",),
    "ray_count": ("ray_count", "rays"),
    "outcome": ("outcome",),
    "detected": ("detected",),
    "detection_range": ("detection_range",),
    "unbraked_ttc": ("unbraked_ttc",),
    "stopping_margin": ("stopping_margin",),
    "collision_speed": ("collision_speed",),
}
TIMING_ALIASES = {
    "mode": ("mode",),
    "ray_count": ("ray_count", "rays"),
    "median_ms": ("median_ms",),
    "p95_ms": ("p95_ms", "p95_scan_ms"),
}
PARITY_ALIASES = {
    "hit_flags_equal": ("hit_flags_equal",),
    "object_ids_equal": ("object_ids_equal",),
    "ranges_equal": ("ranges_equal", "range_within_tolerance"),
}
STYLE = {
    "true-brute": dict(label="true brute", color="#c1272d", marker="o"),
    "scene-bvh": dict(label="scene BVH", color="#0071bc", marker="D"),
}
PLOT_FILENAMES = {
    "budget": "hazard_budget_safe_stop_and_rays.png",
    "fixed_ray": "hazard_safety_by_rays.png",
    "undetected": "hazard_undetected_collisions.png",
}


class ValidationError(ValueError):
    """A benchmark CSV does not describe a complete, comparable experiment."""


def _columns(fieldnames, aliases, path):
    if not fieldnames:
        raise ValidationError(f"{path}: CSV has no header")
    mapping = {}
    for canonical, names in aliases.items():
        found = next((name for name in names if name in fieldnames), None)
        if found is None:
            expected = " or ".join(names)
            raise ValidationError(f"{path}: missing required column {expected!r}")
        mapping[canonical] = found
    return mapping


def _number(value, path, line, column, allow_empty=False):
    if value is None or value.strip() == "":
        if allow_empty:
            return None
        raise ValidationError(f"{path}:{line}: {column} must be a number")
    try:
        number = float(value)
    except ValueError as exc:
        raise ValidationError(f"{path}:{line}: {column} must be a number") from exc
    if not math.isfinite(number):
        raise ValidationError(f"{path}:{line}: {column} must be finite")
    return number


def _boolean(value, path, line, column):
    normalized = (value or "").strip().lower()
    if normalized in ("1", "true", "yes"):
        return True
    if normalized in ("0", "false", "no"):
        return False
    raise ValidationError(f"{path}:{line}: {column} must be true/false or 1/0")


def read_trials(path):
    """Read and validate one trial-level safety CSV."""
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = _columns(reader.fieldnames, TRIAL_ALIASES, path)
        parity_columns = {
            canonical: next((name for name in names if name in reader.fieldnames), None)
            for canonical, names in PARITY_ALIASES.items()
        }
        control_column = next((name for name in ("control", "trial_kind", "condition")
                               if name in reader.fieldnames), None)
        rows = []
        keys = set()
        for line, raw in enumerate(reader, start=2):
            mode = (raw[columns["mode"]] or "").strip()
            scenario_id = (raw[columns["scenario_id"]] or "").strip()
            outcome = (raw[columns["outcome"]] or "").strip()
            if not mode or not scenario_id:
                raise ValidationError(f"{path}:{line}: mode and scenario_id must be non-empty")
            if outcome not in ("SafeStop", "Collision"):
                raise ValidationError(
                    f"{path}:{line}: outcome must be 'SafeStop' or 'Collision'")
            ray_count = _number(raw[columns["ray_count"]], path, line, "ray_count")
            if ray_count < 0 or ray_count != int(ray_count):
                raise ValidationError(f"{path}:{line}: ray_count must be a non-negative integer")
            control = (raw[control_column] or "").strip() if control_column else ""
            key = (mode, int(ray_count), scenario_id, control)
            if key in keys:
                raise ValidationError(f"{path}:{line}: duplicate trial {key}")
            keys.add(key)
            row = {
                "mode": mode,
                "scenario_id": scenario_id,
                "ray_count": int(ray_count),
                "outcome": outcome,
                "detected": _boolean(raw[columns["detected"]], path, line, "detected"),
            }
            for metric in ("detection_range", "unbraked_ttc", "stopping_margin",
                           "collision_speed"):
                row[metric] = _number(raw[columns[metric]], path, line, metric,
                                      allow_empty=True)
            if not row["detected"]:
                # HazardResult uses -1 as its C++ sentinel before first detection.
                # Those quantities do not exist for an undetected trial, including
                # an undetected collision, so retain the trial but omit only its
                # undefined detection-time measurements.
                for metric in ("detection_range", "unbraked_ttc", "stopping_margin"):
                    row[metric] = None
            if row["detected"] and any(row[name] is None for name in (
                    "detection_range", "unbraked_ttc", "stopping_margin")):
                raise ValidationError(
                    f"{path}:{line}: detected trials need detection_range, "
                    "unbraked_ttc, and stopping_margin")
            if outcome == "Collision" and row["collision_speed"] is None:
                raise ValidationError(
                    f"{path}:{line}: collision trials need collision_speed")
            if control_column:
                row["control"] = control
            for canonical, column in parity_columns.items():
                if column is not None:
                    row[canonical] = _boolean(raw[column], path, line, canonical)
            rows.append(row)
    if not rows:
        raise ValidationError(f"{path}: CSV has no trial rows")
    return rows


def read_timing(path):
    """Read and validate complete-scan timing rows."""
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = _columns(reader.fieldnames, TIMING_ALIASES, path)
        rows = []
        keys = set()
        for line, raw in enumerate(reader, start=2):
            mode = (raw[columns["mode"]] or "").strip()
            if not mode:
                raise ValidationError(f"{path}:{line}: mode must be non-empty")
            ray_count = _number(raw[columns["ray_count"]], path, line, "ray_count")
            median_ms = _number(raw[columns["median_ms"]], path, line, "median_ms")
            p95_ms = _number(raw[columns["p95_ms"]], path, line, "p95_ms")
            if ray_count < 0 or ray_count != int(ray_count):
                raise ValidationError(f"{path}:{line}: ray_count must be a non-negative integer")
            if median_ms < 0 or p95_ms < 0:
                raise ValidationError(f"{path}:{line}: timing values must be non-negative")
            key = (mode, int(ray_count))
            if key in keys:
                raise ValidationError(f"{path}:{line}: duplicate timing row {key}")
            keys.add(key)
            rows.append({"mode": mode, "ray_count": int(ray_count),
                         "median_ms": median_ms, "p95_ms": p95_ms})
    if not rows:
        raise ValidationError(f"{path}: CSV has no timing rows")
    return rows


def validate_pairing(trials, timing):
    """Require every mode/ray safety series to contain the same scenario IDs."""
    trial_modes = {row["mode"] for row in trials}
    timing_modes = {row["mode"] for row in timing}
    missing_timing = trial_modes - timing_modes
    if missing_timing:
        raise ValidationError("timing CSV has no rows for trial modes: " +
                              ", ".join(sorted(missing_timing)))
    trial_rays = defaultdict(set)
    timing_rays = defaultdict(set)
    for row in trials:
        trial_rays[row["mode"]].add(row["ray_count"])
    for row in timing:
        timing_rays[row["mode"]].add(row["ray_count"])
    for mode in sorted(trial_modes):
        if trial_rays[mode] != timing_rays[mode]:
            raise ValidationError(
                f"timing ray counts for {mode} do not match safety trial ray counts")

    scenario_sets = defaultdict(set)
    for row in trials:
        scenario_sets[(row["mode"], row["ray_count"])].add(row["scenario_id"])
    reference_key, reference = next(iter(sorted(scenario_sets.items())))
    for key, scenario_ids in sorted(scenario_sets.items()):
        if scenario_ids != reference:
            missing = sorted(reference - scenario_ids)
            extra = sorted(scenario_ids - reference)
            details = []
            if missing:
                details.append("missing " + ",".join(missing[:3]))
            if extra:
                details.append("extra " + ",".join(extra[:3]))
            raise ValidationError(
                f"unpaired scenarios for {key}; expected IDs from {reference_key} "
                f"({'; '.join(details)})")


def paired_bootstrap_difference(left, right, metric, samples=2000, seed=20260819):
    """Return mean(left-right) and a deterministic percentile 95% paired interval."""
    left_by_id = {row["scenario_id"]: metric(row) for row in left}
    right_by_id = {row["scenario_id"]: metric(row) for row in right}
    if left_by_id.keys() != right_by_id.keys():
        raise ValidationError("paired bootstrap received different scenario IDs")
    pairs = [(left_by_id[scenario_id], right_by_id[scenario_id])
             for scenario_id in sorted(left_by_id)]
    pairs = [(a, b) for a, b in pairs if a is not None and b is not None]
    if not pairs:
        return None, None, None, 0
    differences = [a - b for a, b in pairs]
    estimate = sum(differences) / len(differences)
    rng = random.Random(seed)
    n = len(differences)
    bootstrap = []
    for _ in range(samples):
        bootstrap.append(sum(differences[rng.randrange(n)] for _ in range(n)) / n)
    bootstrap.sort()
    return estimate, _quantile(bootstrap, 0.025), _quantile(bootstrap, 0.975), n


def _quantile(values, fraction):
    index = (len(values) - 1) * fraction
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    return values[lower] + (values[upper] - values[lower]) * (index - lower)


def _mean_metric(rows, metric):
    values = [metric(row) for row in rows]
    values = [value for value in values if value is not None]
    return (sum(values) / len(values), len(values)) if values else (None, 0)


def summarize_trials(trials):
    grouped = defaultdict(list)
    for row in trials:
        grouped[(row["mode"], row["ray_count"])].append(row)
    summary = []
    for (mode, ray_count), rows in sorted(grouped.items()):
        for metric_name, metric_label, metric in METRICS:
            mean, n_measured = _mean_metric(rows, metric)
            summary.append({
                "mode": mode,
                "ray_count": ray_count,
                "metric": metric_name,
                "metric_label": metric_label,
                "estimate": mean,
                "n_scenarios": len(rows),
                "n_measured": n_measured,
            })
    return summary, grouped


def pair_differences(grouped, comparisons):
    rows = []
    for context, left_key, right_key in comparisons:
        left, right = grouped[left_key], grouped[right_key]
        for metric_name, metric_label, metric in METRICS:
            estimate, ci_low, ci_high, n_paired = paired_bootstrap_difference(
                left, right, metric)
            rows.append({
                "comparison": context,
                "left_mode": left_key[0],
                "left_ray_count": left_key[1],
                "right_mode": right_key[0],
                "right_ray_count": right_key[1],
                "metric": metric_name,
                "metric_label": metric_label,
                "difference": estimate,
                "ci_low": ci_low,
                "ci_high": ci_high,
                "n_paired": n_paired,
            })
    return rows


def map_budgets(timing, budgets):
    by_mode = defaultdict(list)
    for row in timing:
        by_mode[row["mode"]].append(row)
    mappings = []
    for mode, rows in sorted(by_mode.items()):
        for budget in budgets:
            fits = [row for row in rows if row["p95_ms"] <= budget]
            selected = max(fits, key=lambda row: row["ray_count"]) if fits else None
            mappings.append({
                "mode": mode,
                "budget_ms": budget,
                "ray_count": selected["ray_count"] if selected else 0,
                "selected_p95_ms": selected["p95_ms"] if selected else None,
            })
    return mappings


def prepare_budget_plot_data(summary, mappings, budget_differences):
    """Join mapped rays to actual rates and validate the reported paired intervals."""
    rates = {
        (row["mode"], row["ray_count"]): row["estimate"]
        for row in summary if row["metric"] == "safe_stop_rate"
    }
    mapped_rates = []
    for mapping in mappings:
        key = (mapping["mode"], mapping["ray_count"])
        if key not in rates:
            raise ValidationError(
                f"no safe-stop summary for mapped {mapping['mode']} {mapping['ray_count']} rays")
        mapped_rates.append({**mapping, "safe_stop_rate": rates[key]})

    points_by_budget = defaultdict(dict)
    comparison_budgets = {}
    for point in mapped_rates:
        points_by_budget[point["budget_ms"]][point["mode"]] = point
        comparison_budgets[f"budget_{point['budget_ms']:g}_ms"] = point["budget_ms"]

    safe_differences = []
    for row in budget_differences:
        if row["metric"] != "safe_stop_rate" or row["difference"] is None:
            continue
        budget = comparison_budgets.get(row["comparison"])
        if budget is None:
            raise ValidationError(
                f"safe-stop difference {row['comparison']} has no budget mapping")
        points = points_by_budget[budget]
        try:
            actual = (points[row["left_mode"]]["safe_stop_rate"] -
                      points[row["right_mode"]]["safe_stop_rate"])
        except KeyError as exc:
            raise ValidationError(
                f"safe-stop difference {row['comparison']} has no mapped {exc.args[0]} rate"
            ) from exc
        if not math.isclose(actual, row["difference"], rel_tol=0.0, abs_tol=1e-12):
            raise ValidationError(
                f"safe-stop difference {row['comparison']} does not match mapped rates")
        safe_differences.append({**row, "plot_budget_ms": budget})
    return mapped_rates, sorted(safe_differences, key=lambda row: row["plot_budget_ms"])


def prepare_fixed_ray_safety(summary, grouped):
    """Return one curve after verifying paired equal-ray outcomes across modes."""
    by_mode = defaultdict(dict)
    for row in summary:
        if row["metric"] == "safe_stop_rate":
            by_mode[row["mode"]][row["ray_count"]] = row["estimate"]
    if not by_mode:
        raise ValidationError("no safe-stop summary rows to plot")

    reference_mode = sorted(by_mode)[0]
    reference = by_mode[reference_mode]
    outcomes_by_key = {}
    for mode, values in sorted(by_mode.items()):
        if values.keys() != reference.keys():
            raise ValidationError(
                f"fixed-ray safe-stop counts differ between {reference_mode} and {mode}")
        for ray_count in values:
            rows = grouped[(mode, ray_count)]
            outcomes = {
                row["scenario_id"]: row["outcome"]
                for row in rows
            }
            if len(outcomes) != len(rows):
                raise ValidationError(
                    f"fixed-ray {mode} {ray_count}-ray rows contain duplicate scenario IDs")
            outcomes_by_key[(mode, ray_count)] = outcomes

    for mode, values in sorted(by_mode.items()):
        for ray_count in values:
            reference_outcomes = outcomes_by_key[(reference_mode, ray_count)]
            outcomes = outcomes_by_key[(mode, ray_count)]
            if reference_outcomes.keys() != outcomes.keys():
                raise ValidationError(
                    f"fixed-ray scenarios differ at {ray_count} rays "
                    f"between {reference_mode} and {mode}")
            mismatches = sum(
                reference_outcomes[scenario_id] != outcomes[scenario_id]
                for scenario_id in reference_outcomes)
            if mismatches:
                raise ValidationError(
                    f"fixed-ray outcomes differ for {mismatches} paired scenarios "
                    f"at {ray_count} rays between {reference_mode} and {mode}")
    return [
        {"ray_count": ray_count, "safe_stop_rate": reference[ray_count]}
        for ray_count in sorted(reference)
    ]


def _convergence_budget(timing, supplied):
    if supplied is not None:
        if supplied < 0:
            raise ValidationError("convergence budget must be non-negative")
        return supplied, "supplied"
    p95_at_361 = {}
    for row in timing:
        if row["ray_count"] == 361:
            p95_at_361[row["mode"]] = row["p95_ms"]
    if set(p95_at_361) != {row["mode"] for row in timing}:
        raise ValidationError(
            "cannot measure convergence budget: every timing mode needs a 361-ray row")
    return 1.1 * max(p95_at_361.values()), "measured_110pct_slower_361_p95"


def _gate(name, status, detail):
    return {"gate": name, "status": status, "detail": detail}


def evaluate_gates(trials, grouped, mappings, budget_differences,
                   convergence_budget, control_rows=()):
    gates = []
    by_ray = defaultdict(list)
    for row in trials:
        by_ray[row["ray_count"]].append(row)
    zero_rows = by_ray.get(0, [])
    if zero_rows:
        zero_collisions = all(row["outcome"] == "Collision" for row in zero_rows)
        gates.append(_gate("zero_ray_collisions", "PASS" if zero_collisions else "FAIL",
                           f"{sum(row['outcome'] == 'Collision' for row in zero_rows)}/"
                           f"{len(zero_rows)} zero-ray trials collided"))
    else:
        gates.append(_gate("zero_ray_collisions", "NOT_APPLICABLE",
                           "no zero-ray trials represented"))

    full_rows = by_ray.get(361, [])
    if full_rows:
        rate = sum(row["outcome"] == "SafeStop" for row in full_rows) / len(full_rows)
        gates.append(_gate("full_resolution_safe_stop", "PASS" if rate >= .99 else "FAIL",
                           f"361-ray safe-stop rate {rate:.3%}"))
    else:
        gates.append(_gate("full_resolution_safe_stop", "NOT_APPLICABLE",
                           "no 361-ray trials represented"))

    first_frame_rows = [row for row in control_rows if row.get("control", "").lower() in
                        ("first_frame_braking", "first-frame-braking",
                         "first_frame", "first-frame")]
    if first_frame_rows:
        rate = sum(row["outcome"] == "SafeStop" for row in first_frame_rows) / len(first_frame_rows)
        gates.append(_gate("first_frame_braking_safe_stop",
                           "PASS" if rate >= .99 else "FAIL",
                           f"first-frame-braking safe-stop rate {rate:.3%}"))
    else:
        gates.append(_gate("first_frame_braking_safe_stop", "NOT_APPLICABLE",
                           "no first-frame-braking control rows represented"))

    monotonic_failures = []
    for mode in sorted({row["mode"] for row in trials}):
        series = sorted((ray_count, grouped[(mode, ray_count)])
                        for current_mode, ray_count in grouped if current_mode == mode)
        for metric_name in ("safe_stop_rate", "unbraked_ttc"):
            metric = next(item[2] for item in METRICS if item[0] == metric_name)
            if metric_name == "unbraked_ttc":
                values = [
                    (ray_count, sum(metric(row) or 0.0 for row in rows) / len(rows))
                    for ray_count, rows in series
                ]
            else:
                values = [
                    (ray_count, _mean_metric(rows, metric)[0])
                    for ray_count, rows in series
                ]
            comparable = [(ray_count, value) for ray_count, value in values if value is not None]
            if len(comparable) >= 2:
                for (previous_ray, previous), (current_ray, current) in zip(
                        comparable, comparable[1:]):
                    if current < previous - 1e-12:
                        monotonic_failures.append(
                            f"{mode} {metric_name} fell at {previous_ray}->{current_ray}")
    gates.append(_gate("nested_ray_monotonicity",
                       "FAIL" if monotonic_failures else "PASS",
                       "; ".join(monotonic_failures) or
                       "safe-stop rate and all-scenario mean unbraked TTC do not decrease"))

    present_parity = {name for name in PARITY_ALIASES
                      if any(name in row for row in trials)}
    if present_parity:
        if present_parity != set(PARITY_ALIASES):
            gates.append(_gate("tracer_parity", "FAIL",
                               "partial parity data: " + ", ".join(sorted(present_parity))))
        else:
            mismatches = sum(not all(row[name] for name in PARITY_ALIASES) for row in trials)
            gates.append(_gate("tracer_parity", "PASS" if mismatches == 0 else "FAIL",
                               f"{mismatches}/{len(trials)} parity rows mismatched"))
    else:
        gates.append(_gate("tracer_parity", "NOT_APPLICABLE",
                           "no tracer parity columns represented"))

    modes = {row["mode"] for row in trials}
    mapping_by_budget = defaultdict(dict)
    for row in mappings:
        mapping_by_budget[row["budget_ms"]][row["mode"]] = row["ray_count"]
    comparable_budgets = [
        budget for budget, rays in mapping_by_budget.items()
        if "true-brute" in rays and "scene-bvh" in rays
    ]
    advantage = [
        budget for budget in comparable_budgets
        if mapping_by_budget[budget]["true-brute"] > 0 and
        mapping_by_budget[budget]["scene-bvh"] >=
        2 * mapping_by_budget[budget]["true-brute"]
    ]
    gates.append(_gate("ray_advantage", "PASS" if advantage else "FAIL",
                       ("scene-BVH has >=2x rays at " +
                        ", ".join(f"{budget:g} ms" for budget in advantage))
                       if advantage else "no tested budget has >=2x scene-BVH rays"))

    separation = []
    for row in budget_differences:
        if row["metric"] != "safe_stop_rate" or row["difference"] is None:
            continue
        excludes_zero = row["ci_low"] > 0 or row["ci_high"] < 0
        if abs(row["difference"]) >= .10 and excludes_zero:
            separation.append(row["budget_ms"])
    gates.append(_gate("safety_separation", "PASS" if separation else "FAIL",
                       ("mapped safe-stop rates differ by >=10 pp with a paired "
                        "95% CI excluding zero at " +
                        ", ".join(f"{budget:g} ms" for budget in separation))
                       if separation else "no mapped budget meets the safety separation gate"))

    convergence = mapping_by_budget.get(convergence_budget, {})
    if {"true-brute", "scene-bvh"} <= modes and convergence:
        both_361 = (convergence.get("true-brute") == 361 and
                    convergence.get("scene-bvh") == 361)
        outcomes_match = False
        if both_361 and ("true-brute", 361) in grouped and ("scene-bvh", 361) in grouped:
            left = {row["scenario_id"]: row["outcome"] for row in grouped[("scene-bvh", 361)]}
            right = {row["scenario_id"]: row["outcome"] for row in grouped[("true-brute", 361)]}
            outcomes_match = left == right
        gates.append(_gate("convergence", "PASS" if both_361 and outcomes_match else "FAIL",
                           f"{convergence_budget:g} ms maps to "
                           f"{convergence.get('true-brute', 0)} and "
                           f"{convergence.get('scene-bvh', 0)} rays; "
                           f"361-ray outcomes {'match' if outcomes_match else 'do not match'}"))
    else:
        gates.append(_gate("convergence", "NOT_APPLICABLE",
                           "both true-brute and scene-bvh trial modes are required"))
    return gates


def _write_csv(path, rows, fields):
    with open(path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def plot(summary, grouped, mappings, budget_differences, convergence_budget, out_dir):
    """Render the direct budget result and secondary fixed-ray diagnostics."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.ticker import PercentFormatter
    except ImportError as exc:
        raise ValidationError(
            "plotting requires matplotlib; install the repository's existing plotting "
            "dependency or pass --no-plots") from exc

    def mode_style(mode):
        return STYLE.get(mode, dict(label=mode, color="gray", marker="x"))

    fixed_ray_safety = prepare_fixed_ray_safety(summary, grouped)
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot([row["ray_count"] for row in fixed_ray_safety],
            [row["safe_stop_rate"] for row in fixed_ray_safety],
            marker="o", color="#4d4d4d", label="both tracers (identical outcomes)")
    ax.set_title("Diagnostic: safe-stop outcome by fixed ray count")
    ax.set_xlabel("fixed rays per frame (compute-independent)")
    ax.set_ylabel("safe-stop rate")
    ax.yaxis.set_major_formatter(PercentFormatter(1.0))
    ax.set_ylim(0, 1.04)
    ax.grid(True, alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, PLOT_FILENAMES["fixed_ray"]), dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    by_mode = defaultdict(list)
    for row in summary:
        if row["metric"] == "undetected_collision_rate":
            by_mode[row["mode"]].append(row)
    for mode, rows in by_mode.items():
        rows.sort(key=lambda row: row["ray_count"])
        style = mode_style(mode)
        ax.plot([row["ray_count"] for row in rows],
                [row["estimate"] for row in rows], marker=style["marker"],
                color=style["color"], label=style["label"])
    ax.set_xlabel("rays per frame")
    ax.set_ylabel("undetected-collision rate")
    ax.set_ylim(0, 1)
    ax.grid(True, alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, PLOT_FILENAMES["undetected"]), dpi=140)
    plt.close(fig)

    mapped_rates, safe_differences = prepare_budget_plot_data(
        summary, mappings, budget_differences)
    budgets = sorted({row["budget_ms"] for row in mapped_rates})
    budget_positions = {budget: position for position, budget in enumerate(budgets)}
    fig, (safety_ax, rays_ax) = plt.subplots(1, 2, figsize=(13, 5.8))
    by_mode = defaultdict(list)
    for row in mapped_rates:
        by_mode[row["mode"]].append(row)
    for mode, rows in sorted(by_mode.items()):
        rows.sort(key=lambda row: row["budget_ms"])
        style = mode_style(mode)
        xs = [budget_positions[row["budget_ms"]] for row in rows]
        safety_ax.plot(xs, [row["safe_stop_rate"] for row in rows],
                       marker=style["marker"], color=style["color"],
                       label=style["label"])
        rays_ax.plot(xs, [row["ray_count"] for row in rows],
                     marker=style["marker"], color=style["color"],
                     label=style["label"])
        for x, row in zip(xs, rows):
            horizontal = -10 if mode == "true-brute" else 10
            vertical = -16 if mode == "true-brute" else (
                -29 if row["safe_stop_rate"] > .93 else 8)
            safety_ax.annotate(
                f"{row['safe_stop_rate']:.1%}", (x, row["safe_stop_rate"]),
                xytext=(horizontal, vertical), textcoords="offset points",
                ha="right" if horizontal < 0 else "left", fontsize=8,
                color=style["color"])
            rays_ax.annotate(
                f"{row['ray_count']} rays", (x, row["ray_count"]),
                xytext=(horizontal, -15 if mode == "true-brute" else 7),
                textcoords="offset points",
                ha="right" if horizontal < 0 else "left", fontsize=8,
                color=style["color"])

    tick_labels = [f"{budget:g}" for budget in budgets]
    for ax in (safety_ax, rays_ax):
        ax.set_xticks(range(len(budgets)), tick_labels)
        ax.set_xlabel("discrete p95 scan budget (ms; this machine)")
        ax.grid(True, alpha=.3)
        ax.legend(loc="lower right")
    safety_ax.set_title("(a) Actual safe-stop rate at mapped rays")
    safety_ax.set_ylabel("safe-stop rate")
    safety_ax.yaxis.set_major_formatter(PercentFormatter(1.0))
    safety_ax.set_ylim(0, 1.08)
    rays_ax.set_title("(b) Fixed rays afforded by the same budget")
    rays_ax.set_ylabel("largest tested ray count whose p95 fits")
    rays_ax.set_ylim(bottom=0)

    convergence_x = next(
        (budget_positions[budget] for budget in budgets
         if math.isclose(budget, convergence_budget, rel_tol=0.0, abs_tol=1e-12)),
        None)
    if convergence_x is not None:
        for ax in (safety_ax, rays_ax):
            ax.axvline(convergence_x, color="#555555", linestyle="--",
                       linewidth=1.0, alpha=.7)
        rays_ax.text(convergence_x, rays_ax.get_ylim()[1] * .72,
                     "convergence budget\nboth modes: 361 rays",
                     ha="center", va="center", fontsize=8,
                     bbox=dict(facecolor="white", edgecolor="#777777", alpha=.85))

    difference_by_budget = {
        row["plot_budget_ms"]: row for row in safe_differences
    }
    difference_cells = []
    for budget in budgets:
        row = difference_by_budget[budget]
        difference_cells.append(
            f"{row['difference'] * 100:+.1f}\n"
            f"[{row['ci_low'] * 100:+.1f}, {row['ci_high'] * 100:+.1f}]")
    difference_table = safety_ax.table(
        cellText=[difference_cells],
        rowLabels=["scene-BVH − true-brute (pp)\npaired 95% CI"],
        cellLoc="center", rowLoc="center", bbox=[0.0, -.42, 1.0, .22])
    difference_table.auto_set_font_size(False)
    difference_table.set_fontsize(7)

    fig.suptitle(
        "Machine-specific p95 budget mapping: budget → afforded rays → safe-stop outcome",
        fontsize=12)
    fig.text(
        .5, .015,
        "Discrete estimates from complete fixed-ray scan p95 timings on this machine; "
        "not hard real-time deadline guarantees.",
        ha="center", fontsize=9)
    fig.subplots_adjust(bottom=.29, top=.84, wspace=.28)
    fig.savefig(os.path.join(out_dir, PLOT_FILENAMES["budget"]), dpi=140)
    plt.close(fig)


def analyze(trials_path, timing_path, out_dir, convergence_budget_ms=None, make_plots=True):
    """Run the complete analysis and return output rows for programmatic tests."""
    all_trials = read_trials(trials_path)
    first_frame_controls = {
        "first_frame_braking", "first-frame-braking", "first_frame", "first-frame"
    }
    control_rows = [
        row for row in all_trials
        if row.get("control", "").lower() in first_frame_controls
    ]
    trials = [
        row for row in all_trials
        if row.get("control", "").lower() not in first_frame_controls
    ]
    if not trials:
        raise ValidationError("trial CSV has no fixed-scan experiment rows")
    timing = read_timing(timing_path)
    validate_pairing(trials, timing)
    summary, grouped = summarize_trials(trials)
    convergence_budget, convergence_source = _convergence_budget(
        timing, convergence_budget_ms)
    budgets = tuple(dict.fromkeys((*STANDARD_BUDGETS_MS, convergence_budget)))
    mappings = map_budgets(timing, budgets)

    same_ray = []
    modes = sorted({row["mode"] for row in trials})
    if "true-brute" in modes and "scene-bvh" in modes:
        ray_counts = sorted(set(ray for mode, ray in grouped if mode == "true-brute") &
                            set(ray for mode, ray in grouped if mode == "scene-bvh"))
        same_ray = [(f"same_ray_{ray}", ("scene-bvh", ray), ("true-brute", ray))
                    for ray in ray_counts]
    paired = pair_differences(grouped, same_ray)

    mapping_by_budget = defaultdict(dict)
    for mapping in mappings:
        mapping_by_budget[mapping["budget_ms"]][mapping["mode"]] = mapping["ray_count"]
    budget_comparisons = []
    for budget, mapped in sorted(mapping_by_budget.items()):
        left_key = ("scene-bvh", mapped.get("scene-bvh", 0))
        right_key = ("true-brute", mapped.get("true-brute", 0))
        if left_key in grouped and right_key in grouped:
            budget_comparisons.append((f"budget_{budget:g}_ms", left_key, right_key))
    budget_differences = pair_differences(grouped, budget_comparisons)
    for row in budget_differences:
        row["budget_ms"] = float(row["comparison"].split("_")[1])

    gates = evaluate_gates(
        trials, grouped, mappings, budget_differences, convergence_budget,
        control_rows)
    os.makedirs(out_dir, exist_ok=True)
    _write_csv(os.path.join(out_dir, "hazard_summary.csv"), summary,
               ("mode", "ray_count", "metric", "metric_label", "estimate",
                "n_scenarios", "n_measured"))
    _write_csv(os.path.join(out_dir, "hazard_paired_differences.csv"), paired,
               ("comparison", "left_mode", "left_ray_count", "right_mode",
                "right_ray_count", "metric", "metric_label", "difference",
                "ci_low", "ci_high", "n_paired"))
    _write_csv(os.path.join(out_dir, "hazard_budget_mapping.csv"), mappings,
               ("mode", "budget_ms", "ray_count", "selected_p95_ms"))
    _write_csv(os.path.join(out_dir, "hazard_budget_differences.csv"), budget_differences,
               ("budget_ms", "comparison", "left_mode", "left_ray_count",
                "right_mode", "right_ray_count", "metric", "metric_label",
                "difference", "ci_low", "ci_high", "n_paired"))
    gates.insert(0, _gate("convergence_budget_source", "INFO",
                          f"{convergence_budget:g} ms ({convergence_source})"))
    _write_csv(os.path.join(out_dir, "hazard_acceptance_gates.csv"), gates,
               ("gate", "status", "detail"))
    if make_plots:
        plot(summary, grouped, mappings, budget_differences, convergence_budget, out_dir)
    return {"summary": summary, "mappings": mappings, "paired": paired,
            "budget_differences": budget_differences, "gates": gates}


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--trials", required=True, help="trial-level hazard CSV")
    parser.add_argument("--timing", required=True, help="complete-scan timing CSV")
    parser.add_argument("--out-dir", default="plots/hazard_analysis",
                        help="directory for tables and plots")
    parser.add_argument("--convergence-budget-ms", type=float,
                        help="override the measured 110%% slower-361-ray p95 budget")
    parser.add_argument("--no-plots", action="store_true",
                        help="write tables and gates without importing matplotlib")
    args = parser.parse_args()
    try:
        result = analyze(args.trials, args.timing, args.out_dir,
                         args.convergence_budget_ms, not args.no_plots)
    except (OSError, ValidationError) as exc:
        parser.exit(2, f"error: {exc}\n")
    failures = [gate for gate in result["gates"] if gate["status"] == "FAIL"]
    print(f"[analysis] wrote tables to {args.out_dir}; "
          f"{len(failures)} acceptance gate(s) failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
