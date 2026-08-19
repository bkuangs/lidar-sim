#!/usr/bin/env python3
"""Render the three headline plots for the LiDAR BVH benchmark.

The benchmark binaries are headless and emit CSV; this script is the only place
that turns those numbers into pictures. It runs both binaries, saves their CSVs,
and renders:

  1. layer1_scaling.png  - ns/ray vs obstacle count N
       brute force O(N) scan climbs; scene-BVH O(log N) grows slowly.
  2. layer2_safety.png   - collision-free completion vs compute budget
       the primary all-seed safety outcome with bootstrap confidence intervals.
  3. layer2_rays.png     - rays actually cast (avg_K) vs budget
       the mechanism: the BVH affords far more perception per frame.

CSV is the source of truth. Use --no-run to plot already-saved CSVs without
re-running the benchmarks.

Usage:
  ./.venv/bin/python plot_results.py [--build-dir build] [--seeds 32]
                                     [--out-dir plots] [--with-mesh-bvh]
                                     [--no-run]
"""
import argparse
import csv
import os
import subprocess
import sys
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Stable per-mode styling shared across figures so a mode looks the same
# everywhere. Keys are the raw mode strings the binaries print.
STYLE = {
    "linear-scan": dict(label="brute force  O(N) scan", color="#c1272d", marker="o"),
    "true-brute": dict(label="brute force  O(N.tris)", color="#c1272d", marker="o"),
    "mesh-bvh": dict(label="mesh-BVH", color="#f2a900", marker="s"),
    "scene-bvh": dict(label="scene-BVH  O(log N)", color="#0071bc", marker="D"),
}


def style(mode):
    return STYLE.get(mode, dict(label=mode, color="gray", marker="x"))


def find_binary(build_dir, name):
    path = os.path.join(build_dir, name)
    if not os.path.isfile(path):
        sys.exit(
            f"error: '{path}' not found.\n"
            f"build first:  cmake -S . -B {build_dir} && cmake --build {build_dir}\n"
            f"or plot existing CSVs with --no-run."
        )
    return path


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def run_benchmarks(args, scaling_csv, layer2_csv, trials_csv):
    scaling_bin = find_binary(args.build_dir, "lidar_scaling")
    layer2_bin = find_binary(args.build_dir, "layer2_benchmark")

    print(f"[run] {scaling_bin} --csv {scaling_csv}")
    subprocess.run([scaling_bin, "--csv", scaling_csv], check=True,
                   stdout=subprocess.DEVNULL)

    cmd = [layer2_bin, "--seeds", str(args.seeds), "--csv",
           "--trials-csv", trials_csv]
    if args.with_mesh_bvh:
        cmd.append("--with-mesh-bvh")
    print(f"[run] {' '.join(cmd)} > {layer2_csv}")
    # With --csv the binary prints pure CSV to stdout (verification chatter is
    # suppressed; a verification failure goes to stderr and exits non-zero).
    with open(layer2_csv, "w") as out:
        subprocess.run(cmd, check=True, stdout=out)


def plot_layer1(rows, out_path):
    by_mode = defaultdict(list)
    for r in rows:
        by_mode[r["mode"]].append((float(r["N"]), float(r["ns_per_ray"])))

    fig, ax = plt.subplots(figsize=(7, 4.5))
    for mode, pts in by_mode.items():
        pts.sort()
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        st = style(mode)
        ax.plot(xs, ys, marker=st["marker"], color=st["color"], label=st["label"])

    ax.set_xscale("log")
    ax.set_xlabel("obstacles in corridor  (N, log scale)")
    ax.set_ylabel("query cost  (ns / ray)")
    ax.set_title("Layer 1 - per-ray cost vs scene size\nO(N) scan grows rapidly; scene-BVH grows slowly")
    ax.grid(True, which="both", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[plot] wrote {out_path}")


def _layer2_series(rows, ykey, need_scored):
    """Group Layer 2 rows into {mode: (budgets, ys, stds)} sorted by budget."""
    by_mode = defaultdict(list)
    for r in rows:
        if need_scored and int(r["n_scored"]) == 0:
            continue
        by_mode[r["mode"]].append(
            (float(r["budget_ms"]), float(r[ykey]),
             float(r.get("coll_per_100m_std", 0.0)))
        )
    for mode in by_mode:
        by_mode[mode].sort()
    return by_mode


def plot_layer2_safety(rows, out_path):
    by_mode = defaultdict(list)
    for r in rows:
        by_mode[r["mode"]].append(
            (float(r["budget_ms"]),
             float(r["success_pct"]),
             float(r["success_ci_low_pct"]),
             float(r["success_ci_high_pct"]))
        )
    for mode in by_mode:
        by_mode[mode].sort()

    fig, ax = plt.subplots(figsize=(7, 4.5))
    for mode, pts in by_mode.items():
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        lo = [p[2] for p in pts]
        hi = [p[3] for p in pts]
        st = style(mode)
        ax.plot(xs, ys, marker=st["marker"], color=st["color"], label=st["label"])
        ax.fill_between(xs, lo, hi, color=st["color"], alpha=0.15)

    ax.set_xlabel("per-frame perception budget  (ms)")
    ax.set_ylabel("collision-free completion  (%)")
    ax.set_title("Layer 2 - all-seed safety vs compute budget\npaired courses; shaded 95% bootstrap CI")
    ax.set_ylim(0, 100)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[plot] wrote {out_path}")


def plot_layer2_rays(rows, out_path):
    by_mode = _layer2_series(rows, "avg_k", need_scored=False)

    fig, ax = plt.subplots(figsize=(7, 4.5))
    for mode, pts in by_mode.items():
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        st = style(mode)
        ax.plot(xs, ys, marker=st["marker"], color=st["color"], label=st["label"])

    ax.set_xlabel("per-frame perception budget  (ms)")
    ax.set_ylabel("rays cast per frame  (avg K)")
    ax.set_title("Layer 2 - perception afforded vs budget\nwhy safety differs: the BVH buys more rays per ms")
    ax.set_ylim(bottom=0)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=140)
    plt.close(fig)
    print(f"[plot] wrote {out_path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build-dir", default="build",
                    help="directory holding the compiled binaries (default: build)")
    ap.add_argument("--seeds", type=int, default=32,
                    help="seeds for the Layer 2 sweep (default: 32)")
    ap.add_argument("--out-dir", default="plots",
                    help="output directory for CSVs and PNGs (default: plots)")
    ap.add_argument("--with-mesh-bvh", action="store_true",
                    help="include the mesh-BVH ablation row")
    ap.add_argument("--no-run", action="store_true",
                    help="skip running the binaries; plot existing CSVs in --out-dir")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    scaling_csv = os.path.join(args.out_dir, "scaling.csv")
    layer2_csv = os.path.join(args.out_dir, "layer2.csv")
    trials_csv = os.path.join(args.out_dir, "layer2_trials.csv")

    if not args.no_run:
        run_benchmarks(args, scaling_csv, layer2_csv, trials_csv)
    else:
        for p in (scaling_csv, layer2_csv):
            if not os.path.isfile(p):
                sys.exit(f"error: --no-run but '{p}' is missing; run once without --no-run.")

    plot_layer1(read_csv(scaling_csv), os.path.join(args.out_dir, "layer1_scaling.png"))
    layer2_rows = read_csv(layer2_csv)
    plot_layer2_safety(layer2_rows, os.path.join(args.out_dir, "layer2_safety.png"))
    plot_layer2_rays(layer2_rows, os.path.join(args.out_dir, "layer2_rays.png"))


if __name__ == "__main__":
    main()
