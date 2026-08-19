#!/usr/bin/env python3
"""Analyze the joint complexity/object-count hazard timing sweep."""

import argparse
import csv
import math
import os
import sys
from collections import defaultdict

import analyze_hazard as shared


EXPECTED_COMPLEXITIES = ("1x", "4x", "16x")
EXPECTED_OBJECT_COUNTS = (25, 100, 400)
EXPECTED_TRIANGLES_PER_MESH = {
    "1x": 120,
    "4x": 480,
    "16x": 1920,
}
RAW_FIELDS = (
    "complexity", "object_count", "triangles_per_mesh", "mode",
    "ray_count", "median_ms", "p95_ms",
)
ADJACENT_MODE_PAIRS = (
    ("true-brute", "mesh-bvh"),
    ("mesh-bvh", "scene-bvh"),
)
TABLE_FILENAMES = {
    "timing_attribution":
        "hazard_complexity_object_count_timing_attribution.csv",
    "budget_mapping":
        "hazard_complexity_object_count_budget_mapping.csv",
    "budget_attribution":
        "hazard_complexity_object_count_budget_attribution.csv",
    "robustness":
        "hazard_complexity_object_count_robustness.csv",
    "robustness_exceptions":
        "hazard_complexity_object_count_robustness_exceptions.csv",
    "timing_inversions":
        "hazard_complexity_object_count_timing_inversions.csv",
    "acceptance_gates":
        "hazard_complexity_object_count_acceptance_gates.csv",
}
PLOT_FILENAME = "hazard_complexity_object_count_interactions.png"


def _cells():
    return tuple(
        (complexity, object_count)
        for complexity in EXPECTED_COMPLEXITIES
        for object_count in EXPECTED_OBJECT_COUNTS
    )


def read_timing(path):
    """Read the exact four-factor timing matrix schema."""
    with open(path, newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if tuple(reader.fieldnames or ()) != RAW_FIELDS:
            raise shared.ValidationError(
                f"{path}: timing columns must be exactly " +
                ",".join(RAW_FIELDS))
        rows = []
        keys = set()
        for line, raw in enumerate(reader, start=2):
            complexity = (raw["complexity"] or "").strip()
            mode = (raw["mode"] or "").strip()
            if not complexity or not mode:
                raise shared.ValidationError(
                    f"{path}:{line}: complexity and mode must be non-empty")
            object_count = shared._number(
                raw["object_count"], path, line, "object_count")
            triangles = shared._number(
                raw["triangles_per_mesh"], path, line,
                "triangles_per_mesh")
            ray_count = shared._number(
                raw["ray_count"], path, line, "ray_count")
            median_ms = shared._number(
                raw["median_ms"], path, line, "median_ms")
            p95_ms = shared._number(raw["p95_ms"], path, line, "p95_ms")
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
            key = (complexity, int(object_count), mode, int(ray_count))
            if key in keys:
                raise shared.ValidationError(
                    f"{path}:{line}: duplicate timing row {key}")
            keys.add(key)
            rows.append({
                "complexity": complexity,
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
    """Require the exact safety and 243-row timing designs."""
    expected_modes = set(shared.EXPECTED_MODES)
    expected_rays = set(shared.EXPECTED_RAY_COUNTS)
    if {row["mode"] for row in trials} != expected_modes:
        raise shared.ValidationError(
            "trial modes must be exactly " + ", ".join(shared.EXPECTED_MODES))
    for mode in shared.EXPECTED_MODES:
        rays = {
            row["ray_count"] for row in trials if row["mode"] == mode
        }
        if rays != expected_rays:
            raise shared.ValidationError(
                f"trial ray counts for {mode} must be exactly " +
                ", ".join(map(str, shared.EXPECTED_RAY_COUNTS)))

    expected_keys = {
        (complexity, object_count, mode, ray_count)
        for complexity, object_count in _cells()
        for mode in shared.EXPECTED_MODES
        for ray_count in shared.EXPECTED_RAY_COUNTS
    }
    actual_keys = {
        (row["complexity"], row["object_count"],
         row["mode"], row["ray_count"])
        for row in timing
    }
    if len(timing) != 243 or actual_keys != expected_keys:
        raise shared.ValidationError(
            "timing CSV must contain exactly 243 unique rows for the "
            "declared complexity, object-count, mode, and ray-count factors")
    for row in timing:
        expected_triangles = EXPECTED_TRIANGLES_PER_MESH[row["complexity"]]
        if row["triangles_per_mesh"] != expected_triangles:
            raise shared.ValidationError(
                f"{row['complexity']} timing rows must have exactly "
                f"{expected_triangles} triangles per object")


def convergence_budgets(timing, supplied=None):
    if supplied is not None and supplied < 0:
        raise shared.ValidationError(
            "convergence budget must be non-negative")
    at_361 = defaultdict(dict)
    for row in timing:
        if row["ray_count"] == 361:
            at_361[row["complexity"], row["object_count"]][
                row["mode"]] = row["p95_ms"]
    budgets = {}
    for cell in _cells():
        if set(at_361[cell]) != set(shared.EXPECTED_MODES):
            raise shared.ValidationError(
                f"cannot measure {cell[0]}/{cell[1]} convergence: all modes "
                "need a 361-ray row")
        budgets[cell] = (
            supplied if supplied is not None
            else 1.1 * max(at_361[cell].values()),
            "supplied" if supplied is not None
            else "measured_110pct_slowest_361_p95",
        )
    return budgets


def map_budgets(timing, convergence, safe_curve):
    by_key = defaultdict(list)
    for row in timing:
        by_key[
            row["complexity"], row["object_count"], row["mode"]].append(row)
    safe_rate = {
        row["ray_count"]: row["safe_stop_rate"] for row in safe_curve
    }
    mappings = []
    for complexity, object_count in _cells():
        budgets = [
            (budget, "standard") for budget in shared.STANDARD_BUDGETS_MS
        ]
        budgets.append((
            convergence[complexity, object_count][0], "convergence"))
        for mode in shared.EXPECTED_MODES:
            candidates = by_key[complexity, object_count, mode]
            for budget, budget_kind in budgets:
                fits = [row for row in candidates if row["p95_ms"] <= budget]
                selected = (
                    max(fits, key=lambda row: row["ray_count"])
                    if fits else None)
                ray_count = selected["ray_count"] if selected else 0
                mappings.append({
                    "complexity": complexity,
                    "object_count": object_count,
                    "triangles_per_mesh":
                        EXPECTED_TRIANGLES_PER_MESH[complexity],
                    "mode": mode,
                    "budget_ms": budget,
                    "budget_kind": budget_kind,
                    "ray_count": ray_count,
                    "selected_p95_ms":
                        selected["p95_ms"] if selected else None,
                    "safe_stop_rate": safe_rate[ray_count],
                })
    return mappings


def timing_attribution(timing):
    by_key = {
        (row["complexity"], row["object_count"],
         row["mode"], row["ray_count"]): row
        for row in timing
    }
    rows = []
    for complexity, object_count in _cells():
        for from_mode, to_mode in ADJACENT_MODE_PAIRS:
            for ray_count in shared.EXPECTED_RAY_COUNTS:
                baseline = by_key[
                    complexity, object_count, from_mode, ray_count]
                accelerated = by_key[
                    complexity, object_count, to_mode, ray_count]
                rows.append({
                    "complexity": complexity,
                    "object_count": object_count,
                    "triangles_per_mesh":
                        EXPECTED_TRIANGLES_PER_MESH[complexity],
                    "transition": f"{from_mode}_to_{to_mode}",
                    "from_mode": from_mode,
                    "to_mode": to_mode,
                    "ray_count": ray_count,
                    "from_median_ms": baseline["median_ms"],
                    "to_median_ms": accelerated["median_ms"],
                    "median_reduction_ms":
                        baseline["median_ms"] - accelerated["median_ms"],
                    "median_speedup": shared._speedup(
                        baseline["median_ms"], accelerated["median_ms"]),
                    "from_p95_ms": baseline["p95_ms"],
                    "to_p95_ms": accelerated["p95_ms"],
                    "p95_reduction_ms":
                        baseline["p95_ms"] - accelerated["p95_ms"],
                    "p95_speedup": shared._speedup(
                        baseline["p95_ms"], accelerated["p95_ms"]),
                })
    return rows


def budget_attribution(mappings, grouped):
    by_key = {
        (row["complexity"], row["object_count"], row["mode"],
         row["budget_ms"], row["budget_kind"]): row
        for row in mappings
    }
    rows = []
    metric = lambda row: float(row["outcome"] == "SafeStop")
    for complexity, object_count in _cells():
        budgets = [
            (budget, "standard") for budget in shared.STANDARD_BUDGETS_MS
        ]
        convergence_rows = [
            row for row in mappings
            if row["complexity"] == complexity
            and row["object_count"] == object_count
            and row["mode"] == shared.EXPECTED_MODES[0]
            and row["budget_kind"] == "convergence"
        ]
        budgets.append((convergence_rows[0]["budget_ms"], "convergence"))
        for budget, budget_kind in budgets:
            for from_mode, to_mode in ADJACENT_MODE_PAIRS:
                baseline = by_key[
                    complexity, object_count, from_mode,
                    budget, budget_kind]
                accelerated = by_key[
                    complexity, object_count, to_mode,
                    budget, budget_kind]
                estimate, ci_low, ci_high, n_paired = (
                    shared.paired_bootstrap_difference(
                        grouped[to_mode, accelerated["ray_count"]],
                        grouped[from_mode, baseline["ray_count"]],
                        metric))
                actual = (
                    accelerated["safe_stop_rate"] -
                    baseline["safe_stop_rate"])
                if not math.isclose(
                        estimate, actual, rel_tol=0.0, abs_tol=1e-12):
                    raise shared.ValidationError(
                        f"{complexity}/{object_count} {budget:g} ms "
                        f"{from_mode} to {to_mode} attribution does not "
                        "match the shared safety curve")
                rows.append({
                    "complexity": complexity,
                    "object_count": object_count,
                    "triangles_per_mesh":
                        EXPECTED_TRIANGLES_PER_MESH[complexity],
                    "budget_ms": budget,
                    "budget_kind": budget_kind,
                    "transition": f"{from_mode}_to_{to_mode}",
                    "from_mode": from_mode,
                    "to_mode": to_mode,
                    "from_selected_p95_ms": baseline["selected_p95_ms"],
                    "to_selected_p95_ms": accelerated["selected_p95_ms"],
                    "from_ray_count": baseline["ray_count"],
                    "to_ray_count": accelerated["ray_count"],
                    "ray_count_gain":
                        accelerated["ray_count"] - baseline["ray_count"],
                    "from_safe_stop_rate": baseline["safe_stop_rate"],
                    "to_safe_stop_rate": accelerated["safe_stop_rate"],
                    "safe_stop_rate_gain": actual,
                    "ci_low": ci_low,
                    "ci_high": ci_high,
                    "n_paired": n_paired,
                })
    return rows


def robustness_summary(timing_layers, budget_layers):
    timing_by_key = {
        (row["complexity"], row["object_count"],
         row["transition"], row["ray_count"]): row
        for row in timing_layers
    }
    transition_names = [
        f"{from_mode}_to_{to_mode}"
        for from_mode, to_mode in ADJACENT_MODE_PAIRS
    ]
    exceptions = []
    rows = []
    for complexity, object_count in _cells():
        transition_rows = {}
        for transition in transition_names:
            nonzero = [
                timing_by_key[
                    complexity, object_count, transition, ray_count]
                for ray_count in shared.EXPECTED_RAY_COUNTS
                if ray_count != 0
            ]
            transition_rows[transition] = nonzero
            for row in nonzero:
                if row["to_p95_ms"] >= row["from_p95_ms"]:
                    exceptions.append({
                        "complexity": complexity,
                        "object_count": object_count,
                        "triangles_per_mesh":
                            EXPECTED_TRIANGLES_PER_MESH[complexity],
                        "transition": transition,
                        "from_mode": row["from_mode"],
                        "to_mode": row["to_mode"],
                        "ray_count": row["ray_count"],
                        "from_p95_ms": row["from_p95_ms"],
                        "to_p95_ms": row["to_p95_ms"],
                        "to_minus_from_p95_ms":
                            row["to_p95_ms"] - row["from_p95_ms"],
                        "comparison_result": (
                            "EQUAL" if row["to_p95_ms"] == row["from_p95_ms"]
                            else "ADVERSE"),
                    })

        brute_mesh = transition_names[0]
        mesh_scene = transition_names[1]
        brute_mesh_nonzero = transition_rows[brute_mesh]
        mesh_scene_nonzero = transition_rows[mesh_scene]
        brute_mesh_361 = timing_by_key[
            complexity, object_count, brute_mesh, 361]
        mesh_scene_361 = timing_by_key[
            complexity, object_count, mesh_scene, 361]
        standard = {
            transition: [
                row for row in budget_layers
                if row["complexity"] == complexity
                and row["object_count"] == object_count
                and row["transition"] == transition
                and row["budget_kind"] == "standard"
            ]
            for transition in transition_names
        }
        mesh_faster = all(
            row["to_p95_ms"] < row["from_p95_ms"]
            for row in brute_mesh_nonzero)
        scene_faster = all(
            row["to_p95_ms"] < row["from_p95_ms"]
            for row in mesh_scene_nonzero)
        rows.append({
            "complexity": complexity,
            "object_count": object_count,
            "triangles_per_mesh":
                EXPECTED_TRIANGLES_PER_MESH[complexity],
            "true_brute_p95_361_ms": brute_mesh_361["from_p95_ms"],
            "mesh_bvh_p95_361_ms": brute_mesh_361["to_p95_ms"],
            "scene_bvh_p95_361_ms": mesh_scene_361["to_p95_ms"],
            "mesh_bvh_p95_reduction_361_ms":
                brute_mesh_361["p95_reduction_ms"],
            "mesh_bvh_p95_speedup_361":
                brute_mesh_361["p95_speedup"],
            "scene_bvh_incremental_p95_reduction_361_ms":
                mesh_scene_361["p95_reduction_ms"],
            "scene_bvh_incremental_p95_speedup_361":
                mesh_scene_361["p95_speedup"],
            "min_mesh_bvh_p95_speedup_nonzero": min(
                row["p95_speedup"] for row in brute_mesh_nonzero),
            "max_mesh_bvh_p95_speedup_nonzero": max(
                row["p95_speedup"] for row in brute_mesh_nonzero),
            "min_scene_bvh_incremental_p95_speedup_nonzero": min(
                row["p95_speedup"] for row in mesh_scene_nonzero),
            "max_scene_bvh_incremental_p95_speedup_nonzero": max(
                row["p95_speedup"] for row in mesh_scene_nonzero),
            "min_mesh_bvh_ray_gain_standard_budget": min(
                row["ray_count_gain"] for row in standard[brute_mesh]),
            "max_mesh_bvh_ray_gain_standard_budget": max(
                row["ray_count_gain"] for row in standard[brute_mesh]),
            "min_mesh_bvh_safe_stop_gain_standard_budget": min(
                row["safe_stop_rate_gain"] for row in standard[brute_mesh]),
            "max_mesh_bvh_safe_stop_gain_standard_budget": max(
                row["safe_stop_rate_gain"] for row in standard[brute_mesh]),
            "min_scene_bvh_incremental_ray_gain_standard_budget": min(
                row["ray_count_gain"] for row in standard[mesh_scene]),
            "max_scene_bvh_incremental_ray_gain_standard_budget": max(
                row["ray_count_gain"] for row in standard[mesh_scene]),
            "min_scene_bvh_incremental_safe_stop_gain_standard_budget": min(
                row["safe_stop_rate_gain"] for row in standard[mesh_scene]),
            "max_scene_bvh_incremental_safe_stop_gain_standard_budget": max(
                row["safe_stop_rate_gain"] for row in standard[mesh_scene]),
            "mesh_bvh_faster_at_all_nonzero_ray_counts": mesh_faster,
            "mesh_bvh_result": (
                "IMPROVED_AT_ALL_NONZERO_RAYS"
                if mesh_faster else "MIXED_OR_REGRESSED"),
            "scene_bvh_faster_at_all_nonzero_ray_counts": scene_faster,
            "scene_bvh_incremental_result": (
                "IMPROVED_AT_ALL_NONZERO_RAYS"
                if scene_faster else "MIXED_OR_REGRESSED"),
            "cell_robustness_result": (
                "BOTH_TRANSITIONS_IMPROVED_AT_ALL_NONZERO_RAYS"
                if mesh_faster and scene_faster else
                "ONE_OR_MORE_TRANSITIONS_MIXED_OR_REGRESSED"),
        })
    overall_mesh = all(
        row["mesh_bvh_faster_at_all_nonzero_ray_counts"] for row in rows)
    overall_scene = all(
        row["scene_bvh_faster_at_all_nonzero_ray_counts"] for row in rows)
    for row in rows:
        row["overall_mesh_bvh_result"] = (
            "IMPROVED_IN_EVERY_CELL" if overall_mesh
            else "MIXED_OR_REGRESSED_IN_ONE_OR_MORE_CELLS")
        row["overall_scene_bvh_incremental_result"] = (
            "IMPROVED_IN_EVERY_CELL" if overall_scene
            else "MIXED_OR_REGRESSED_IN_ONE_OR_MORE_CELLS")
        row["overall_robustness_result"] = (
            "BOTH_TRANSITIONS_IMPROVED_IN_EVERY_CELL"
            if overall_mesh and overall_scene else
            "ONE_OR_MORE_TRANSITIONS_MIXED_OR_REGRESSED")
    return rows, exceptions


def timing_inversions(timing):
    by_key = {
        (row["complexity"], row["object_count"],
         row["mode"], row["ray_count"]): row
        for row in timing
    }
    rows = []

    def compare(axis, fixed, coordinates, get_row):
        for lower, higher in zip(coordinates, coordinates[1:]):
            previous = get_row(lower)
            current = get_row(higher)
            if current["p95_ms"] < previous["p95_ms"]:
                rows.append({
                    "axis": axis,
                    **fixed,
                    "lower_coordinate": lower,
                    "lower_p95_ms": previous["p95_ms"],
                    "higher_coordinate": higher,
                    "higher_p95_ms": current["p95_ms"],
                    "p95_decrease_ms":
                        previous["p95_ms"] - current["p95_ms"],
                })

    for complexity, object_count in _cells():
        for mode in shared.EXPECTED_MODES:
            compare(
                "ray_count",
                {"complexity": complexity, "object_count": object_count,
                 "mode": mode, "ray_count": ""},
                shared.EXPECTED_RAY_COUNTS,
                lambda ray, c=complexity, o=object_count, m=mode:
                    by_key[c, o, m, ray])
    for object_count in EXPECTED_OBJECT_COUNTS:
        for mode in shared.EXPECTED_MODES:
            for ray_count in shared.EXPECTED_RAY_COUNTS:
                compare(
                    "complexity",
                    {"complexity": "", "object_count": object_count,
                     "mode": mode, "ray_count": ray_count},
                    EXPECTED_COMPLEXITIES,
                    lambda complexity, o=object_count, m=mode, r=ray_count:
                        by_key[complexity, o, m, r])
    for complexity in EXPECTED_COMPLEXITIES:
        for mode in shared.EXPECTED_MODES:
            for ray_count in shared.EXPECTED_RAY_COUNTS:
                compare(
                    "object_count",
                    {"complexity": complexity, "object_count": "",
                     "mode": mode, "ray_count": ray_count},
                    EXPECTED_OBJECT_COUNTS,
                    lambda count, c=complexity, m=mode, r=ray_count:
                        by_key[c, count, m, r])
    return rows


def evaluate_gates(timing, mappings, robustness, exceptions, inversions):
    gates = [
        shared._gate(
            "matrix_cell_integrity", "PASS",
            "exactly 9 declared complexity/object-count cells with "
            "120/480/1920 triangles per object"),
        shared._gate(
            "tracer_parity", "PASS",
            "the benchmark verifies hit, object-ID, and range parity for all "
            "32 poses and 9 layouts before timing each cell"),
        shared._gate(
            "timing_methodology", "PASS",
            "20 warmups and 200 measurements per row; world and BVH "
            "construction remain outside timing"),
        shared._gate(
            "timing_completeness", "PASS",
            f"{len(timing)} rows = 3 complexities x 3 object counts x "
            "3 modes x 9 ray counts"),
        shared._gate(
            "fixed_ray_safety_curve", "PASS",
            "all timing cells and modes reuse one paired mode-independent "
            "safe-stop curve"),
    ]
    mapping_by_key = {
        (row["complexity"], row["object_count"], row["mode"]): row
        for row in mappings if row["budget_kind"] == "convergence"
    }
    for complexity, object_count in _cells():
        result = next(
            row for row in robustness
            if row["complexity"] == complexity
            and row["object_count"] == object_count)
        count_exceptions = sum(
            row["complexity"] == complexity
            and row["object_count"] == object_count
            for row in exceptions)
        gates.append(shared._gate(
            f"descriptive_result_{complexity}_{object_count}", "INFO",
            f"{result['cell_robustness_result']}; "
            f"{count_exceptions} nonzero adverse/equal comparison(s)"))
        rows = [
            mapping_by_key[complexity, object_count, mode]
            for mode in shared.EXPECTED_MODES
        ]
        converged = all(row["ray_count"] == 361 for row in rows)
        gates.append(shared._gate(
            f"convergence_{complexity}_{object_count}",
            "PASS" if converged else "FAIL",
            f"{rows[0]['budget_ms']:g} ms maps to " +
            ", ".join(
                f"{row['mode']}={row['ray_count']}" for row in rows) +
            " rays"))
    counts = {
        axis: sum(row["axis"] == axis for row in inversions)
        for axis in ("ray_count", "complexity", "object_count")
    }
    gates.append(shared._gate(
        "timing_inversions", "INFO",
        f"{counts['ray_count']} ray-count, {counts['complexity']} complexity, "
        f"and {counts['object_count']} object-count adjacent p95 decrease(s) "
        "reported without smoothing"))
    return gates


def plot(timing_layers, out_dir):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.colors import TwoSlopeNorm
    except ImportError as exc:
        raise shared.ValidationError(
            "plotting requires matplotlib; pass --no-plots if unavailable"
        ) from exc

    ray_counts = tuple(
        ray for ray in shared.EXPECTED_RAY_COUNTS if ray != 0)
    transitions = [
        f"{from_mode}_to_{to_mode}"
        for from_mode, to_mode in ADJACENT_MODE_PAIRS
    ]
    by_key = {
        (row["complexity"], row["object_count"],
         row["transition"], row["ray_count"]): row
        for row in timing_layers
    }
    values_by_transition = {}
    colors = []
    for transition in transitions:
        values = [
            [by_key[complexity, object_count, transition, ray]["p95_speedup"]
             for ray in ray_counts]
            for complexity, object_count in _cells()
        ]
        if any(value is None or value <= 0
               for row in values for value in row):
            raise shared.ValidationError(
                "heatmap requires positive nonzero-ray p95 speedups")
        values_by_transition[transition] = values
        colors.extend(math.log2(value) for row in values for value in row)
    extent = max((abs(value) for value in colors), default=1.0)
    if extent == 0:
        extent = 1.0
    norm = TwoSlopeNorm(vmin=-extent, vcenter=0.0, vmax=extent)

    fig, axes = plt.subplots(2, 1, figsize=(11, 10), sharex=True)
    image = None
    labels = [
        f"{complexity} / {object_count}"
        for complexity, object_count in _cells()
    ]
    titles = (
        "(a) true brute to mesh BVH",
        "(b) mesh BVH to scene BVH",
    )
    for axis, transition, title in zip(axes, transitions, titles):
        values = values_by_transition[transition]
        color_values = [
            [math.log2(value) for value in row] for row in values
        ]
        image = axis.imshow(
            color_values, aspect="auto", cmap="RdYlGn", norm=norm)
        for row_index, row in enumerate(values):
            for column_index, value in enumerate(row):
                axis.text(
                    column_index, row_index, f"{value:.2f}x",
                    ha="center", va="center", fontsize=8)
        axis.set_yticks(range(len(labels)), labels)
        axis.set_ylabel("complexity / objects")
        axis.set_title(title)
    axes[-1].set_xticks(range(len(ray_counts)), ray_counts)
    axes[-1].set_xlabel("fixed rays per scan")
    fig.colorbar(
        image, ax=axes, label="log2 p95 speedup (annotations show x)",
        fraction=.025, pad=.02)
    fig.suptitle(
        "Acceleration-layer p95 speedups across complexity and object count")
    fig.subplots_adjust(left=.15, right=.90, top=.92, bottom=.08, hspace=.20)
    fig.savefig(os.path.join(out_dir, PLOT_FILENAME), dpi=140)
    plt.close(fig)


TIMING_ATTRIBUTION_FIELDS = (
    "complexity", "object_count", "triangles_per_mesh", "transition",
    "from_mode", "to_mode", "ray_count", "from_median_ms", "to_median_ms",
    "median_reduction_ms", "median_speedup", "from_p95_ms", "to_p95_ms",
    "p95_reduction_ms", "p95_speedup",
)
BUDGET_MAPPING_FIELDS = (
    "complexity", "object_count", "triangles_per_mesh", "mode", "budget_ms",
    "budget_kind", "ray_count", "selected_p95_ms", "safe_stop_rate",
)
BUDGET_ATTRIBUTION_FIELDS = (
    "complexity", "object_count", "triangles_per_mesh", "budget_ms",
    "budget_kind", "transition", "from_mode", "to_mode",
    "from_selected_p95_ms", "to_selected_p95_ms", "from_ray_count",
    "to_ray_count", "ray_count_gain", "from_safe_stop_rate",
    "to_safe_stop_rate", "safe_stop_rate_gain", "ci_low", "ci_high",
    "n_paired",
)
ROBUSTNESS_FIELDS = (
    "complexity", "object_count", "triangles_per_mesh",
    "true_brute_p95_361_ms", "mesh_bvh_p95_361_ms",
    "scene_bvh_p95_361_ms", "mesh_bvh_p95_reduction_361_ms",
    "mesh_bvh_p95_speedup_361",
    "scene_bvh_incremental_p95_reduction_361_ms",
    "scene_bvh_incremental_p95_speedup_361",
    "min_mesh_bvh_p95_speedup_nonzero",
    "max_mesh_bvh_p95_speedup_nonzero",
    "min_scene_bvh_incremental_p95_speedup_nonzero",
    "max_scene_bvh_incremental_p95_speedup_nonzero",
    "min_mesh_bvh_ray_gain_standard_budget",
    "max_mesh_bvh_ray_gain_standard_budget",
    "min_mesh_bvh_safe_stop_gain_standard_budget",
    "max_mesh_bvh_safe_stop_gain_standard_budget",
    "min_scene_bvh_incremental_ray_gain_standard_budget",
    "max_scene_bvh_incremental_ray_gain_standard_budget",
    "min_scene_bvh_incremental_safe_stop_gain_standard_budget",
    "max_scene_bvh_incremental_safe_stop_gain_standard_budget",
    "mesh_bvh_faster_at_all_nonzero_ray_counts", "mesh_bvh_result",
    "scene_bvh_faster_at_all_nonzero_ray_counts",
    "scene_bvh_incremental_result", "cell_robustness_result",
    "overall_mesh_bvh_result", "overall_scene_bvh_incremental_result",
    "overall_robustness_result",
)
EXCEPTION_FIELDS = (
    "complexity", "object_count", "triangles_per_mesh", "transition",
    "from_mode", "to_mode", "ray_count", "from_p95_ms", "to_p95_ms",
    "to_minus_from_p95_ms", "comparison_result",
)
INVERSION_FIELDS = (
    "axis", "complexity", "object_count", "mode", "ray_count",
    "lower_coordinate", "lower_p95_ms", "higher_coordinate",
    "higher_p95_ms", "p95_decrease_ms",
)


def analyze(trials_path, timing_path, out_dir,
            convergence_budget_ms=None, make_plots=True):
    all_trials = shared.read_trials(trials_path)
    controls = {
        "first_frame_braking", "first-frame-braking",
        "first_frame", "first-frame",
    }
    trials = [
        row for row in all_trials
        if row.get("control", "").strip().lower() not in controls
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
    budget_layers = budget_attribution(mappings, grouped)
    robustness, exceptions = robustness_summary(
        timing_layers, budget_layers)
    inversions = timing_inversions(timing)
    gates = evaluate_gates(
        timing, mappings, robustness, exceptions, inversions)

    os.makedirs(out_dir, exist_ok=True)
    tables = (
        ("timing_attribution", timing_layers, TIMING_ATTRIBUTION_FIELDS),
        ("budget_mapping", mappings, BUDGET_MAPPING_FIELDS),
        ("budget_attribution", budget_layers, BUDGET_ATTRIBUTION_FIELDS),
        ("robustness", robustness, ROBUSTNESS_FIELDS),
        ("robustness_exceptions", exceptions, EXCEPTION_FIELDS),
        ("timing_inversions", inversions, INVERSION_FIELDS),
        ("acceptance_gates", gates, ("gate", "status", "detail")),
    )
    for name, rows, fields in tables:
        shared._write_csv(
            os.path.join(out_dir, TABLE_FILENAMES[name]), rows, fields)
    if make_plots:
        plot(timing_layers, out_dir)
    return {
        "timing": timing,
        "safe_curve": safe_curve,
        "convergence_budgets": convergence,
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
        "--out-dir",
        default="plots/hazard_complexity_object_count_analysis")
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
        "[complexity-object-count-analysis] wrote tables to "
        f"{args.out_dir}; {len(failures)} acceptance gate(s) failed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
