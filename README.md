# 3D LiDAR Simulator

After learning about ray tracing in computer graphics, particularly acceleration data structures (BVHs),
I was curious whether applying the same optimization to LiDAR sensors could make an 
autonomous vehicle measurably safer under time constraint, given their underlying similarities.  

**Scenario:** A car running basic follow-the-gap obstacle avoidance drives down a corridor 
with randomized pillar spawns, scanning with a simulated, ray-traced 3D LiDAR. We will measure 
how many rays each mode (brute force vs. acceleration) can afford under a fixed, per-frame compute budget.

## Comparison

We compare across 2 verticals:

**1. Latency vs. scene size:** Trace a fixed ray set
against a corridor of `N` obstacles and measure nanosec/ray as `N` grows. A brute force
`O(N)` scene walk climbs linearly; the scene-BVH `O(log N)` stays flat. Every
BVH result is correctness-gated against the linear scan (0 mismatches).

**2. Autonomy at a budget:**. Give each
frame a fixed compute budget (e.g. 0.5 ms). A calibrated cost model converts that budget
into `K` = how many azimuth rays the mode can cast. The hypothesis is that under severe constraint, 
brute force cannot generate a clear picture fast enough and crashes more, while BVH always affords full angular
resolution. We will use collisions per 100 m as our metric.  

### Cost Model

We use a **cost model** measured in nanoseconds per ray (ns/ray) to compare performance. The key idea is that for each ray, its job is to walk and find the nearest surface it hits. The nearest hit is what the LiDAR reports as the range for that direction. Brute force finds it by testing the ray against every triangle in the scene; BVH uses bounding boxes to take shortcuts and not have to check large areas that are empty. Thus, we expect its 
ns/ray is to be much shorter.

## Build & Run

```bash
cmake -S . -B build
cmake --build build
```

Then:

```bash
./build/lidar_tests
./build/lidar_scaling         # Latency/scaling comparison
./build/calibrate             # refit the cost model (prints R²)
./build/layer2_benchmark
```

## Plots

A small Python script to generate plots:

```bash
python3 -m venv .venv
./.venv/bin/pip install matplotlib
./.venv/bin/python tools/plot_results.py            # default 32 seeds -> plots/
./.venv/bin/python tools/plot_results.py --no-run   # replot saved CSVs
```  

### Crash Safety 
<img src="plots/layer2_safety.png" alt="Collisions per 100m" width="70%" />  

We observe roughly **~2.5x less** collisions when given only 0.5 ms.  

### Ray Latency  
<img src="plots/layer1_scaling.png" alt="Collisions per 100m" width="70%" />  

BVH becomes **exponentially** cheaper (less time per ray) as the scene grows. 