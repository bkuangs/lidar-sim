#!/usr/bin/env python3
"""Render the Layer 1 scaling plot for the LiDAR BVH benchmark.

The benchmark binaries are headless and emit CSV; this script is the only place
that turns Layer 1 scaling numbers into a picture. It runs the scaling binary,
saves its CSV, and renders:

  1. layer1_scaling.png  - ns/ray vs obstacle count N
       brute force O(N) scan climbs; scene-BVH O(log N) grows slowly.

CSV is the source of truth. Use --no-run to plot already-saved CSVs without
re-running the benchmarks.

Usage:
  ./.venv/bin/python tools/plot_results.py [--build-dir build] [--out-dir plots]
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


def run_benchmark(args, scaling_csv):
    scaling_bin = find_binary(args.build_dir, "lidar_scaling")

    print(f"[run] {scaling_bin} --csv {scaling_csv}")
    subprocess.run([scaling_bin, "--csv", scaling_csv], check=True,
                   stdout=subprocess.DEVNULL)


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


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--build-dir", default="build",
                    help="directory holding the compiled binaries (default: build)")
    ap.add_argument("--out-dir", default="plots",
                    help="output directory for CSVs and PNGs (default: plots)")
    ap.add_argument("--no-run", action="store_true",
                    help="skip running the binaries; plot existing CSVs in --out-dir")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    scaling_csv = os.path.join(args.out_dir, "scaling.csv")

    if not args.no_run:
        run_benchmark(args, scaling_csv)
    elif not os.path.isfile(scaling_csv):
        sys.exit(f"error: --no-run but '{scaling_csv}' is missing; run once without --no-run.")

    plot_layer1(read_csv(scaling_csv), os.path.join(args.out_dir, "layer1_scaling.png"))


if __name__ == "__main__":
    main()
