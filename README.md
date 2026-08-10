# lidar_3d — BVH-accelerated LiDAR vs brute force

A **headless** C++/Eigen benchmark that answers one question: *does a BVH
acceleration structure make an autonomous vehicle measurably safer, not just
faster?* A follow-the-gap vehicle drives a randomized pillar corridor, scanning
with a simulated 3D LiDAR. The same (correct) ray hits feed every mode — what
differs is how many rays each mode can **afford** under a fixed per-frame
compute budget.

No GPU, no viewer, no external deps beyond Eigen. Results come out as CSV and a
few plots.

## The two layers

**Layer 1 — latency vs scene size** (`lidar_scaling`). Trace a fixed ray set
against a corridor of `N` obstacles and measure ns/ray as `N` grows. A brute
`O(N)` scene walk climbs linearly; the scene-BVH `O(log N)` stays flat. Every
BVH result is correctness-gated against the linear scan (0 mismatches).

**Layer 2 — autonomy under a frame budget** (`layer2_benchmark`). Give each
frame a fixed compute budget (ms). A calibrated cost model converts that budget
into `K` = how many azimuth rays the mode can cast. Starve brute force and it
aliases thin pillars and crashes more; the BVH always affords full angular
resolution. The headline metric is **collisions per 100 m** vs budget. All modes
trace identical, verified hits — only `K` differs.

The link between them is a **cost model** fitted on the real pillar geometry
(`calibrate`, R² reported), baked into `include/layer2.hpp`.

## Layout

```
include/      header-only core: ray tracing, BVH, scenes, LiDAR, vehicle, planner, Layer 2
benchmarks/   the driver executables (scaling, calibrate, layer2_benchmark)
tests/        correctness + determinism gates
tools/        plot_results.py — runs the binaries, saves CSVs, renders the plots
docs/         DESIGN.md (the single source of truth) + the original project outline
```

## Build & run

Requires CMake ≥ 3.18, a C++17 compiler, and Eigen3.

```bash
cmake -S . -B build          # add -DCMAKE_PREFIX_PATH=/opt/homebrew on macOS/brew
cmake --build build
```

Then:

```bash
./build/lidar_tests                    # all correctness + determinism gates
./build/lidar_scaling                  # Layer 1 table + scaling.csv
./build/calibrate                      # refit the cost model (prints R²)
./build/layer2_benchmark               # Layer 2 sweep (budget x mode x seeds)
```

Useful flags: `layer2_benchmark --seeds 32 --speed 3.5 --csv`,
`--with-mesh-bvh` (opt-in ablation), `--no-verify` (skip the scene gate);
`lidar_scaling --with-buckets` (opt-in Layer 1 intermediate mode).

## Plots

The C++ stays headless; a small Python script turns the CSVs into figures.

```bash
python3 -m venv .venv
./.venv/bin/pip install matplotlib
./.venv/bin/python tools/plot_results.py            # default 32 seeds -> plots/
./.venv/bin/python tools/plot_results.py --no-run   # replot saved CSVs
```

Outputs (in `plots/`, gitignored):

- `layer1_scaling.png` — ns/ray vs N: the `O(N)` scan blows up, the BVH stays flat.
- `layer2_safety.png` — collisions/100 m vs budget (±std): the accelerator crashes far less when compute is scarce.
- `layer2_rays.png` — rays cast vs budget: *why* — the BVH buys far more perception per ms.

## More

See [`docs/DESIGN.md`](docs/DESIGN.md) for the full rationale, the cost-model
calibration, locked parameters, results, and the running progress log.
