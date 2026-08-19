#!/usr/bin/env python3
"""Analyze the fixed-complexity hazard timing sweep over object count."""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import analyze_hazard as shared


EXPECTED_OBJECT_COUNTS = (25, 50, 100, 200, 400)
EXPECTED_TRIANGLES_PER_MESH = 120
FROM_MODE = "mesh-bvh"
TO_MODE = "scene-bvh"
TIMING_ALIASES = {
    "object_count": ("object_count",),
    "triangles_per_mesh": ("triangles_per_mesh",),
    "mode": ("mode",),
    "ray_count": ("ray_count", "rays"),
    "median_ms": ("median_ms",),
    "p95_ms": ("p95_ms", "p95_scan_ms"),
}


def read_timing(path):
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        columns = shared._columns(reader.fieldnames, TIMING_ALIASES, path)
        rows = []
        keys = set()
        for line, raw in enumerate(reader, start=2):
            mode = (raw[columns["mode"]] or "").strip()
            object_count = shared._number(
                raw[columns["object_count"]], path, line, "object_count")
            triangles = shared._number(
                raw[columns["triangles_per_mesh"]], path, line,
                "triangles_per_mesh")
            ray_count = shared._number(
                raw[columns["ray_count"]], path, line, "ray_count")
            median_ms = shared._number(
                raw[columns["median_ms"]], path, line, "median_ms")
            p95_ms = shared._number(
                raw[columns["p95_ms"]], path, line, "p95_ms")
            if not mode:
                raise shared.ValidationError(
                    f"{path}:{line}: mode must be non-empty")
            for name, value in (
                    ("object_count", object_count),
                    ("triangles_per_mesh", triangles),
                    ("ray_count", ray_count)):
                if value < 0 or value != int(value):
                    raise shared.ValidationError(
                        f"{path}:{line}: {name} must be a non-negative integer")
            if object_count == 0 or triangles == 0:
                raise shared.ValidationError(
                    f"{path}:{line}: object and triangle counts must be positive")
            if median_ms < 0 or p95_ms < 0:
                raise shared.ValidationError(
                    f"{path}:{line}: timing values must be non-negative")
            key = (int(object_count), mode, int(ray_count))
            if key in keys:
                raise shared.ValidationError(
                    f"{path}:{line}: duplicate timing row {key}")
            keys.add(key)
            rows.append({
                "object_count": int(object_count),
                "triangles_per_mesh": int(triangles),
                "mode": mode,
                "ray_count": int(ray_count),
                "median_ms": median_ms,
                "p95_ms": p95_ms,
            })
    if not rows:
        raise shared.ValidationError(f"{path}: CSV has no timing rows")
    return rows


def validate_coverage(trials, timing):
    if {row["mode"] for row in trials} != set(shared.EXPECTED_MODES):
        raise shared.ValidationError(
            "trial modes must be exactly " + ", ".join(shared.EXPECTED_MODES))
    if {row["mode"] for row in timing} != set(shared.EXPECTED_MODES):
        raise shared.ValidationError(
            "timing modes must be exactly " + ", ".join(shared.EXPECTED_MODES))
    if {row["object_count"] for row in timing} != set(EXPECTED_OBJECT_COUNTS):
        raise shared.ValidationError(
            "timing object counts must be exactly " +
            ", ".join(map(str, EXPECTED_OBJECT_COUNTS)))

    expected_rays = set(shared.EXPECTED_RAY_COUNTS)
    for mode in shared.EXPECTED_MODES:
        trial_rays = {
            row["ray_count"] for row in trials if row["mode"] == mode
        }
        if trial_rays != expected_rays:
            raise shared.ValidationError(
                f"trial ray counts for {mode} must be exactly " +
                ", ".join(map(str, shared.EXPECTED_RAY_COUNTS)))
        for object_count in EXPECTED_OBJECT_COUNTS:
            rows = [
                row for row in timing
                if row["object_count"] == object_count and row["mode"] == mode
            ]
            if {row["ray_count"] for row in rows} != expected_rays:
                raise shared.ValidationError(
                    f"timing ray counts for {object_count} objects and {mode} "
                    "must be exactly " +
                    ", ".join(map(str, shared.EXPECTED_RAY_COUNTS)))
            if {row["triangles_per_mesh"] for row in rows} != {
                    EXPECTED_TRIANGLES_PER_MESH}:
                raise shared.ValidationError(
                    f"{object_count}-object timing rows must have exactly "
                    f"{EXPECTED_TRIANGLES_PER_MESH} triangles per mesh")
    expected_rows = (
        len(EXPECTED_OBJECT_COUNTS) *
        len(shared.EXPECTED_MODES) *
        len(shared.EXPECTED_RAY_COUNTS))
    if len(timing) != expected_rows:
        raise shared.ValidationError(
            f"timing CSV must contain exactly {expected_rows} unique rows")


def convergence_budgets(timing, supplied=None):
    if supplied is not None and supplied < 0:
        raise shared.ValidationError(
            "convergence budget must be non-negative")
    at_361 = defaultdict(dict)
    for row in timing:
        if row["ray_count"] == 361:
            at_361[row["object_count"]][row["mode"]] = row["p95_ms"]
    budgets = {}
    for object_count in EXPECTED_OBJECT_COUNTS:
        if set(at_361[object_count]) != set(shared.EXPECTED_MODES):
            raise shared.ValidationError(
                f"cannot measure {object_count}-object convergence: all modes "
                "need a 361-ray row")
        budgets[object_count] = (
            supplied if supplied is not None
            else 1.1 * max(at_361[object_count].values()),
            "supplied" if supplied is not None
            else "measured_110pct_slower_361_p95",
        )
    return budgets


def map_budgets(timing, budgets_by_count, safe_curve):
    by_key = defaultdict(list)
    for row in timing:
        by_key[row["object_count"], row["mode"]].append(row)
    safe_rate = {
        row["ray_count"]: row["safe_stop_rate"] for row in safe_curve
    }
    mappings = []
    for object_count in EXPECTED_OBJECT_COUNTS:
        budgets = tuple(dict.fromkeys((
            *shared.STANDARD_BUDGETS_MS,
            budgets_by_count[object_count][0],
        )))
        for mode in shared.EXPECTED_MODES:
            rows = by_key[object_count, mode]
            for budget in budgets:
                fits = [row for row in rows if row["p95_ms"] <= budget]
                selected = (
                    max(fits, key=lambda row: row["ray_count"])
                    if fits else None)
                ray_count = selected["ray_count"] if selected else 0
                mappings.append({
                    "object_count": object_count,
                    "triangles_per_mesh": EXPECTED_TRIANGLES_PER_MESH,
                    "mode": mode,
                    "budget_ms": budget,
                    "budget_kind": (
                        "standard"
                        if budget in shared.STANDARD_BUDGETS_MS
                        else "convergence"),
                    "ray_count": ray_count,
                    "selected_p95_ms": (
                        selected["p95_ms"] if selected else None),
                    "safe_stop_rate": safe_rate[ray_count],
                })
    return mappings


def timing_attribution(timing):
    by_key = {
        (row["object_count"], row["mode"], row["ray_count"]): row
        for row in timing
    }
    rows = []
    for object_count in EXPECTED_OBJECT_COUNTS:
        for ray_count in shared.EXPECTED_RAY_COUNTS:
            baseline = by_key[object_count, FROM_MODE, ray_count]
            accelerated = by_key[object_count, TO_MODE, ray_count]
            rows.append({
                "object_count": object_count,
                "triangles_per_mesh": EXPECTED_TRIANGLES_PER_MESH,
                "transition": f"{FROM_MODE}_to_{TO_MODE}",
                "ray_count": ray_count,
                "mesh_bvh_median_ms": baseline["median_ms"],
                "scene_bvh_median_ms": accelerated["median_ms"],
                "median_reduction_ms": (
                    baseline["median_ms"] - accelerated["median_ms"]),
                "median_speedup": shared._speedup(
                    baseline["median_ms"], accelerated["median_ms"]),
                "mesh_bvh_p95_ms": baseline["p95_ms"],
                "scene_bvh_p95_ms": accelerated["p95_ms"],
                "p95_reduction_ms": (
                    baseline["p95_ms"] - accelerated["p95_ms"]),
                "p95_speedup": shared._speedup(
                    baseline["p95_ms"], accelerated["p95_ms"]),
            })
    return rows


def timing_inversions(timing):
    rows = []
    by_count_mode = defaultdict(list)
    by_mode_ray = defaultdict(list)
    for row in timing:
        by_count_mode[row["object_count"], row["mode"]].append(row)
        by_mode_ray[row["mode"], row["ray_count"]].append(row)

    for (object_count, mode), series in sorted(by_count_mode.items()):
        series.sort(key=lambda row: row["ray_count"])
        for previous, current in zip(series, series[1:]):
            if current["p95_ms"] < previous["p95_ms"]:
                rows.append({
                    "axis": "ray_count",
                    "object_count": object_count,
                    "mode": mode,
                    "ray_count": "",
                    "lower_coordinate": previous["ray_count"],
                    "lower_p95_ms": previous["p95_ms"],
                    "higher_coordinate": current["ray_count"],
                    "higher_p95_ms": current["p95_ms"],
                    "p95_decrease_ms": (
                        previous["p95_ms"] - current["p95_ms"]),
                })
    for (mode, ray_count), series in sorted(by_mode_ray.items()):
        series.sort(key=lambda row: row["object_count"])
        for previous, current in zip(series, series[1:]):
            if current["p95_ms"] < previous["p95_ms"]:
                rows.append({
                    "axis": "object_count",
                    "object_count": "",
                    "mode": mode,
                    "ray_count": ray_count,
                    "lower_coordinate": previous["object_count"],
                    "lower_p95_ms": previous["p95_ms"],
                    "higher_coordinate": current["object_count"],
                    "higher_p95_ms": current["p95_ms"],
                    "p95_decrease_ms": (
                        previous["p95_ms"] - current["p95_ms"]),
                })
    return rows


def budget_attribution(mappings, grouped):
    by_key = {
        (row["object_count"], row["mode"], row["budget_ms"]): row
        for row in mappings
    }
    contexts = sorted({
        (row["object_count"], row["budget_ms"], row["budget_kind"])
        for row in mappings
    })
    rows = []
    metric = lambda row: float(row["outcome"] == "SafeStop")
    for object_count, budget, budget_kind in contexts:
        baseline = by_key[object_count, FROM_MODE, budget]
        accelerated = by_key[object_count, TO_MODE, budget]
        estimate, ci_low, ci_high, n_paired = (
            shared.paired_bootstrap_difference(
                grouped[TO_MODE, accelerated["ray_count"]],
                grouped[FROM_MODE, baseline["ray_count"]],
                metric))
        actual = (
            accelerated["safe_stop_rate"] - baseline["safe_stop_rate"])
        if not math.isclose(
                estimate, actual, rel_tol=0.0, abs_tol=1e-12):
            raise shared.ValidationError(
                f"{object_count}-object {budget:g} ms safety attribution "
                "does not match the fixed-ray curve")
        rows.append({
            "object_count": object_count,
            "triangles_per_mesh": EXPECTED_TRIANGLES_PER_MESH,
            "budget_ms": budget,
            "budget_kind": budget_kind,
            "transition": f"{FROM_MODE}_to_{TO_MODE}",
            "mesh_bvh_selected_p95_ms": baseline["selected_p95_ms"],
            "scene_bvh_selected_p95_ms": accelerated["selected_p95_ms"],
            "mesh_bvh_ray_count": baseline["ray_count"],
            "scene_bvh_ray_count": accelerated["ray_count"],
            "ray_count_gain": (
                accelerated["ray_count"] - baseline["ray_count"]),
            "mesh_bvh_safe_stop_rate": baseline["safe_stop_rate"],
            "scene_bvh_safe_stop_rate": accelerated["safe_stop_rate"],
            "safe_stop_rate_gain": actual,
            "ci_low": ci_low,
            "ci_high": ci_high,
            "n_paired": n_paired,
        })
    return rows


def robustness_summary(timing_layers, budget_layers):
    timing_by_key = {
        (row["object_count"], row["ray_count"]): row
        for row in timing_layers
    }
    rows = []
    exceptions = []
    for object_count in EXPECTED_OBJECT_COUNTS:
        nonzero = [
            timing_by_key[object_count, ray_count]
            for ray_count in shared.EXPECTED_RAY_COUNTS
            if ray_count != 0
        ]
        for row in nonzero:
            if row["scene_bvh_p95_ms"] >= row["mesh_bvh_p95_ms"]:
                exceptions.append({
                    "object_count": object_count,
                    "triangles_per_mesh": EXPECTED_TRIANGLES_PER_MESH,
                    "ray_count": row["ray_count"],
                    "mesh_bvh_p95_ms": row["mesh_bvh_p95_ms"],
                    "scene_bvh_p95_ms": row["scene_bvh_p95_ms"],
                    "scene_minus_mesh_p95_ms": (
                        row["scene_bvh_p95_ms"] -
                        row["mesh_bvh_p95_ms"]),
                })
        standard = [
            row for row in budget_layers
            if row["object_count"] == object_count and
            row["budget_kind"] == "standard"
        ]
        at_361 = timing_by_key[object_count, 361]
        improved = all(row["p95_speedup"] > 1.0 for row in nonzero)
        rows.append({
            "object_count": object_count,
            "triangles_per_mesh": EXPECTED_TRIANGLES_PER_MESH,
            "mesh_bvh_p95_361_ms": at_361["mesh_bvh_p95_ms"],
            "scene_bvh_p95_361_ms": at_361["scene_bvh_p95_ms"],
            "p95_reduction_361_ms": at_361["p95_reduction_ms"],
            "p95_speedup_361": at_361["p95_speedup"],
            "min_p95_speedup_nonzero": min(
                row["p95_speedup"] for row in nonzero),
            "max_p95_speedup_nonzero": max(
                row["p95_speedup"] for row in nonzero),
            "min_ray_gain_standard_budget": min(
                row["ray_count_gain"] for row in standard),
            "max_ray_gain_standard_budget": max(
                row["ray_count_gain"] for row in standard),
            "min_safe_stop_gain_standard_budget": min(
                row["safe_stop_rate_gain"] for row in standard),
            "max_safe_stop_gain_standard_budget": max(
                row["safe_stop_rate_gain"] for row in standard),
            "scene_bvh_faster_at_all_nonzero_ray_counts": improved,
            "object_count_result": (
                "IMPROVED_AT_ALL_NONZERO_RAYS"
                if improved else "MIXED_OR_REGRESSED"),
        })
    return rows, exceptions


def evaluate_gates(timing, mappings, robustness, exceptions,
                   inversions, convergence):
    gates = [
        shared._gate(
            "timing_completeness", "PASS",
            f"{len(timing)} rows = 5 object counts x 3 modes x 9 ray counts"),
        shared._gate(
            "fixed_ray_safety_curve", "PASS",
            "all object counts map onto one paired mode-independent safety curve"),
    ]
    mapping_by_key = {
        (row["object_count"], row["mode"], row["budget_ms"]): row
        for row in mappings
    }
    for object_count in EXPECTED_OBJECT_COUNTS:
        result = next(
            row for row in robustness
            if row["object_count"] == object_count)
        count_exceptions = [
            row for row in exceptions
            if row["object_count"] == object_count
        ]
        gates.append(shared._gate(
            f"scene_bvh_incremental_result_{object_count}", "INFO",
            f"{result['object_count_result']}; "
            f"{len(count_exceptions)} nonzero-ray exception(s)"))
        budget = convergence[object_count][0]
        rows = [
            mapping_by_key[object_count, mode, budget]
            for mode in shared.EXPECTED_MODES
        ]
        converged = all(row["ray_count"] == 361 for row in rows)
        gates.append(shared._gate(
            f"convergence_{object_count}",
            "PASS" if converged else "FAIL",
            f"{budget:g} ms maps to " +
            ", ".join(
                f"{row['mode']}={row['ray_count']}" for row in rows) +
            " rays"))
    gates.append(shared._gate(
        "timing_inversions", "INFO",
        f"{sum(row['axis'] == 'ray_count' for row in inversions)} ray-axis "
        f"and {sum(row['axis'] == 'object_count' for row in inversions)} "
        "object-axis p95 decrease(s) reported without smoothing"))
    return gates


def plot(timing_layers, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import Normalize, TwoSlopeNorm
    except ImportError as exc:
        raise shared.ValidationError(
            "plotting requires matplotlib; pass --no-plots if unavailable"
        ) from exc

    fig, (latency_ax, heatmap_ax) = plt.subplots(
        1, 2, figsize=(14, 5.5), gridspec_kw={"width_ratios": (1, 1.5)})
    for mode, field in (
            (FROM_MODE, "mesh_bvh_p95_ms"),
            (TO_MODE, "scene_bvh_p95_ms")):
        rows = [
            row for row in timing_layers if row["ray_count"] == 361
        ]
        rows.sort(key=lambda row: row["object_count"])
        style = shared.STYLE[mode]
        latency_ax.plot(
            [row["object_count"] for row in rows],
            [row[field] for row in rows],
            marker=style["marker"], color=style["color"],
            label=style["label"])
    latency_ax.set_xscale("log", base=2)
    latency_ax.set_yscale("log")
    latency_ax.set_xticks(EXPECTED_OBJECT_COUNTS, EXPECTED_OBJECT_COUNTS)
    latency_ax.set_xlabel("objects in the fixed scene domain")
    latency_ax.set_ylabel("p95 complete-scan latency (ms)")
    latency_ax.set_title("(a) Same-ray p95 at 361 rays")
    latency_ax.grid(True, alpha=.3)
    latency_ax.legend()

    ray_counts = [
        ray_count for ray_count in shared.EXPECTED_RAY_COUNTS
        if ray_count != 0
    ]
    by_key = {
        (row["object_count"], row["ray_count"]): row
        for row in timing_layers
    }
    values = [
        [by_key[object_count, ray_count]["p95_speedup"]
         for ray_count in ray_counts]
        for object_count in EXPECTED_OBJECT_COUNTS
    ]
    color_values = [
        [math.log2(value) for value in row]
        for row in values
    ]
    flat_colors = [value for row in color_values for value in row]
    if min(flat_colors) < 0 < max(flat_colors):
        norm = TwoSlopeNorm(
            vmin=min(flat_colors), vcenter=0.0, vmax=max(flat_colors))
    else:
        norm = Normalize(vmin=min(flat_colors), vmax=max(flat_colors))
    image = heatmap_ax.imshow(
        color_values, aspect="auto", cmap="RdYlGn", norm=norm)
    for row_index, row in enumerate(values):
        for column_index, value in enumerate(row):
            heatmap_ax.text(
                column_index, row_index, f"{value:.2f}x",
                ha="center", va="center", fontsize=8)
    heatmap_ax.set_xticks(range(len(ray_counts)), ray_counts)
    heatmap_ax.set_yticks(
        range(len(EXPECTED_OBJECT_COUNTS)), EXPECTED_OBJECT_COUNTS)
    heatmap_ax.set_xlabel("fixed rays per scan")
    heatmap_ax.set_ylabel("object count")
    heatmap_ax.set_title("(b) mesh-BVH / scene-BVH p95 speedup")
    fig.colorbar(
        image, ax=heatmap_ax,
        label="log2 incremental p95 speedup (annotations show x)")
    fig.suptitle(
        "Top-level scene-BVH value as object count grows at 120 triangles/object")
    fig.tight_layout()
    fig.savefig(
        os.path.join(out_dir, "hazard_object_count_robustness.png"),
        dpi=140)
    plt.close(fig)


def analyze(trials_path, timing_path, out_dir,
            convergence_budget_ms=None, make_plots=True):
    all_trials = shared.read_trials(trials_path)
    first_frame_controls = {
        "first_frame_braking", "first-frame-braking",
        "first_frame", "first-frame",
    }
    trials = [
        row for row in all_trials
        if row.get("control", "").lower() not in first_frame_controls
    ]
    if not trials:
        raise shared.ValidationError(
            "trial CSV has no fixed-scan experiment rows")
    timing = read_timing(timing_path)
    validate_coverage(trials, timing)
    summary, grouped = shared.summarize_trials(trials)
    safe_curve = shared.prepare_fixed_ray_safety(summary, grouped)
    convergence = convergence_budgets(timing, convergence_budget_ms)
    mappings = map_budgets(timing, convergence, safe_curve)
    timing_layers = timing_attribution(timing)
    inversions = timing_inversions(timing)
    budget_layers = budget_attribution(mappings, grouped)
    robustness, exceptions = robustness_summary(
        timing_layers, budget_layers)
    gates = evaluate_gates(
        timing, mappings, robustness, exceptions, inversions, convergence)

    os.makedirs(out_dir, exist_ok=True)
    shared._write_csv(
        os.path.join(
            out_dir, "hazard_object_count_timing_attribution.csv"),
        timing_layers,
        ("object_count", "triangles_per_mesh", "transition", "ray_count",
         "mesh_bvh_median_ms", "scene_bvh_median_ms",
         "median_reduction_ms", "median_speedup",
         "mesh_bvh_p95_ms", "scene_bvh_p95_ms",
         "p95_reduction_ms", "p95_speedup"))
    shared._write_csv(
        os.path.join(out_dir, "hazard_object_count_budget_mapping.csv"),
        mappings,
        ("object_count", "triangles_per_mesh", "mode", "budget_ms",
         "budget_kind", "ray_count", "selected_p95_ms", "safe_stop_rate"))
    shared._write_csv(
        os.path.join(
            out_dir, "hazard_object_count_budget_attribution.csv"),
        budget_layers,
        ("object_count", "triangles_per_mesh", "budget_ms", "budget_kind",
         "transition", "mesh_bvh_selected_p95_ms",
         "scene_bvh_selected_p95_ms", "mesh_bvh_ray_count",
         "scene_bvh_ray_count", "ray_count_gain",
         "mesh_bvh_safe_stop_rate", "scene_bvh_safe_stop_rate",
         "safe_stop_rate_gain", "ci_low", "ci_high", "n_paired"))
    shared._write_csv(
        os.path.join(out_dir, "hazard_object_count_robustness.csv"),
        robustness,
        ("object_count", "triangles_per_mesh",
         "mesh_bvh_p95_361_ms", "scene_bvh_p95_361_ms",
         "p95_reduction_361_ms", "p95_speedup_361",
         "min_p95_speedup_nonzero", "max_p95_speedup_nonzero",
         "min_ray_gain_standard_budget", "max_ray_gain_standard_budget",
         "min_safe_stop_gain_standard_budget",
         "max_safe_stop_gain_standard_budget",
         "scene_bvh_faster_at_all_nonzero_ray_counts",
         "object_count_result"))
    shared._write_csv(
        os.path.join(
            out_dir, "hazard_object_count_robustness_exceptions.csv"),
        exceptions,
        ("object_count", "triangles_per_mesh", "ray_count",
         "mesh_bvh_p95_ms", "scene_bvh_p95_ms",
         "scene_minus_mesh_p95_ms"))
    shared._write_csv(
        os.path.join(
            out_dir, "hazard_object_count_timing_inversions.csv"),
        inversions,
        ("axis", "object_count", "mode", "ray_count",
         "lower_coordinate", "lower_p95_ms", "higher_coordinate",
         "higher_p95_ms", "p95_decrease_ms"))
    shared._write_csv(
        os.path.join(
            out_dir, "hazard_object_count_acceptance_gates.csv"),
        gates, ("gate", "status", "detail"))
    if make_plots:
        plot(timing_layers, out_dir)
    return {
        "timing": timing,
        "mappings": mappings,
        "timing_attribution": timing_layers,
        "budget_attribution": budget_layers,
        "robustness": robustness,
        "robustness_exceptions": exceptions,
        "timing_inversions": inversions,
        "gates": gates,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trials", required=True)
    parser.add_argument("--timing", required=True)
    parser.add_argument(
        "--out-dir", default="plots/hazard_object_count_analysis")
    parser.add_argument("--convergence-budget-ms", type=float)
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    try:
        result = analyze(
            args.trials, args.timing, args.out_dir,
            args.convergence_budget_ms, not args.no_plots)
    except (OSError, shared.ValidationError) as exc:
        parser.exit(2, f"error: {exc}\n")
    failures = [
        gate for gate in result["gates"] if gate["status"] == "FAIL"
    ]
    print(
        f"[object-count-analysis] wrote tables to {args.out_dir}; "
        f"{len(failures)} acceptance gate(s) failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
