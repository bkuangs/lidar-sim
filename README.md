# 3D LiDAR Simulator

This project compares triangle-soup, per-mesh BVH, and scene-BVH ray tracing for
simulated LiDAR.
It publishes two controlled benchmarks:

1. **Layer 1 scaling:** fixed-ray latency as obstacle count grows.
2. **Hazard benchmark:** a straight-road vehicle detects one cylindrical hazard
   and brakes; fixed, nested ray counts measure detection and safe-stop outcomes.

The hazard experiment is deliberately narrow: it does not establish behavior for
general road scenes, planners, or hard real-time systems.

## Build, Test, and Run

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/lidar_tests
```

Run Layer 1 with:

```bash
./build/lidar_scaling
./.venv/bin/python tools/plot_results.py --build-dir build --out-dir plots
```

Generate and analyze the hazard results with:

```bash
./build/hazard_benchmark safety --csv plots/hazard_trials.csv
./build/hazard_benchmark timing --csv plots/hazard_timing.csv
./.venv/bin/python tools/analyze_hazard.py \
  --trials plots/hazard_trials.csv --timing plots/hazard_timing.csv \
  --out-dir plots/hazard_analysis
```

`hazard_trials.csv` has every fixed-ray scenario outcome; `hazard_timing.csv`
has median and p95 scan timing. The analyzer writes summary, paired-difference,
timing-attribution, budget-mapping, budget-attribution, and acceptance-gate
tables plus plots under `plots/hazard_analysis/`. Use `python3` in place of
`.venv/bin/python` when no project virtual environment is available.

## Published Results

### Layer 1 latency
<img src="plots/layer1_scaling.png" alt="Ray latency versus scene size" width="70%" />

At 3,200 obstacles, scene-BVH is roughly **224x cheaper per ray** and grows much
more slowly with scene size than the linear scan.

### Hazard safety and budget mapping
<img src="plots/hazard_analysis/hazard_budget_safe_stop_and_rays.png" alt="Actual hazard safe-stop rates and afforded ray counts at each discrete p95 budget" width="100%" />

The published controlled experiment contains 1,440 deterministic scenarios per
mode and ray count. At 361 rays, per-mesh BVH acceleration reduces p95 scan
latency from **21.284 ms** for true-brute to **0.524 ms** (**40.6x**); adding the
scene BVH reduces it further to **0.197 ms** (another **2.67x**).

The primary figure maps each discrete budget to the largest tested fixed ray
count whose measured p95 fits, then maps that ray count to its observed safe-stop
rate. At 0.5 ms, the three layers map to **9 -> 257 -> 361 rays** and **34.1% ->
100.0% -> 100.0%** safe stops. Per-mesh BVH therefore contributes the measured
**+65.9 percentage-point** safety gain at that budget; scene BVH contributes
another **104 rays** but no additional safe stops because the safety curve has
already converged. The adjacent-layer confidence intervals and every declared
budget are reported in
[`hazard_budget_attribution.csv`](plots/hazard_analysis/hazard_budget_attribution.csv).

The dashed convergence budget is 110% of the slowest mode's measured 361-ray
p95. There, all three modes afford 361 rays and produce the same **100.0%**
safe-stop rate. These are machine-specific p95 estimates from complete fixed-ray
scans, not hard real-time deadline guarantees. All acceptance gates pass; see
[`hazard_acceptance_gates.csv`](plots/hazard_analysis/hazard_acceptance_gates.csv).

#### Fixed-ray safety diagnostic
<img src="plots/hazard_analysis/hazard_safety_by_rays.png" alt="Mode-independent hazard safe-stop rate by fixed ray count" width="70%" />

This secondary curve isolates the safety effect of ray count from compute cost.
Only one curve is shown because all three tracers produce identical outcomes
when given the same fixed rays.
