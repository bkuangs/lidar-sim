# Controlled Hazard Benchmark Plan

## Status

The first-pass MVP is implemented and its published outputs pass every
acceptance gate. The hazard benchmark has replaced the old Layer 2 benchmark and
its tracked artifacts; Layer 1 remains. This document is retained as the
experiment specification for the controlled straight-road hazard study, not as a
claim about broader driving, planner, or real-time behavior.

## Goal

Test one causal chain:

```text
BVH -> more LiDAR rays within a measured budget -> earlier hazard detection
    -> more braking distance -> fewer collisions
```

Keep the existing Layer 1 scaling benchmark. The completed hazard benchmark is
the replacement for Layer 2.

## Keep: First-Pass MVP

### Experiment

Each scenario uses a straight road, one stationary cylindrical hazard, and no
planner or steering. The vehicle:

1. starts at 3 m/s;
2. scans at 30 Hz;
3. continues at 3 m/s until a completed ray hits the hazard object ID;
4. applies 4 m/s² braking from that frame onward;
5. ends in either a safe stop or collision.

Use these scenario values:

| Variable | Values |
|---|---|
| Initial front-bumper clearance | 2, 4, 8 m |
| Cylinder diameter | 0.10, 0.25, 0.50 m |
| Lateral offset from vehicle centerline | -0.30, -0.15, 0.00, 0.15, 0.30 m |
| Cylinder height | 2.0 m |
| LiDAR height | 1.2 m |
| Horizontal FOV | [-90°, +90°) |
| Maximum range | 20 m |
| Frame rate | 30 Hz |
| Braking | 4 m/s² |

Run 32 deterministic azimuth phases for every clearance, diameter, and offset
combination. A scenario ID identifies the complete parameter tuple, including
phase.

A valid scenario:

- starts without overlap;
- places the hazard within the LiDAR range and FOV;
- produces a collision if the vehicle never brakes;
- permits a safe stop if braking begins on the first frame.

Generate only valid scenarios. Validation failures are benchmark errors, not
trials that may be filtered from the results.

### Ray layout

Create one 361-bin base grid over the horizontal FOV. For phase `p` in `[0, 1)`:

```text
azimuth(i) = -90° + (i + p) * 180° / 361
```

Use `progressiveAzimuthOrder(361)` and take nested prefixes of:

```text
0, 5, 9, 17, 33, 65, 129, 257, 361 rays/frame
```

Every denser layout therefore contains every ray from the preceding layout.
This makes earlier detection non-regressive by construction and avoids
left-to-right scan bias. Derive `p` deterministically from the scenario ID.

Fixed-ray scans are instantaneous in simulated time. They measure the safety
effect of ray count, not wall-clock deadline behavior.

### Frame and collision semantics

For each frame:

1. scan from the current pose;
2. latch detection if any scan point has the hazard object ID;
3. select constant speed or maximum braking;
4. advance the vehicle by `1/30` second;
5. test the full swept motion for hazard contact.

Collision takes precedence if contact occurs during the motion step. A safe stop
requires zero speed without contact. A run that exceeds its conservative frame
limit is a benchmark error, not a physical outcome.

Initial clearance and stopping margin use the vehicle's front bumper and the
hazard's circular collision footprint, not LiDAR range. Collision footprints
remain independent of mesh tessellation.

### Outcomes and metrics

```cpp
enum class HazardOutcome { SafeStop, Collision };

struct HazardScenario {
    unsigned scenario_id;
    double speed;
    double initial_clearance;
    double lateral_offset;
    double azimuth_phase;
    HazardSpec hazard;
};

struct HazardResult {
    HazardOutcome outcome;
    bool detected;
    double detection_range;
    double unbraked_ttc;
    double stopping_margin;
    double collision_speed;
};
```

At first detection:

```text
unbraked_ttc = remaining straight-line distance to contact / current speed
stopping_margin = remaining distance to contact - speed² / (2 * braking)
```

Primary metric:

```text
safe-stop rate = safe stops / all generated scenarios
```

Also report detection range, unbraked TTC, stopping margin, collision speed, and
undetected collisions. Use paired scenario IDs and paired bootstrap confidence
intervals. Do not filter collisions or create a composite safety score.

### Object identity and geometry

Propagate the existing `Hit::objId` into `ScanPoint`:

```cpp
struct ScanPoint {
    double azimuth;
    double elevation;
    double range;
    int object_id;
    bool hit;
};
```

Use `-1` for clear rays. Brake only on the hazard object ID. Other hits must not
trigger braking.

For the MVP, reuse the existing pillar mesh as a 2 m cylinder with fixed `28x8`
slices/stacks. Its collision footprint is the intended circle, not the polygonal
mesh boundary.

### Timing and budget mapping

Time complete fixed-ray scans for:

```text
true-brute
scene-bvh
```

Use one deterministic timing scene with 100 medium-complexity background
objects. Timing backgrounds are not present in safety trials, so they cannot
occlude the hazard or alter collision outcomes.

For every mode and tested ray count:

- exclude world construction, BVH construction, CSV output, and aggregation;
- include ray construction and intersection queries;
- run at least 20 warmup scans and 200 timed scans over deterministic poses;
- report median and p95 full-scan latency.

For budgets `0.5, 1, 2, 5, 8 ms`, select the largest tested ray count whose p95
fits. If none fits, map the budget to zero rays. Also include a convergence
budget equal to 110% of the slower mode's measured p95 at 361 rays.

The fixed-ray safety curve is compute-independent. The budget-to-ray mapping is
machine-specific. Combining them produces a p95-based estimate, not a claim
that every live scan meets a hard deadline.

### Output

Add one executable:

```bash
./build/hazard_benchmark safety --csv plots/hazard_trials.csv
./build/hazard_benchmark timing --csv plots/hazard_timing.csv
```

Emit trial-level CSV from C++. Aggregate, bootstrap, and plot in Python.

### Required tests

- fixed scans emit the exact requested ray count;
- ray layouts are nested, deterministic, and contain no duplicates;
- scan points preserve hit object IDs;
- identical layouts produce identical hits across both tracers;
- scenario generation is deterministic and produces only valid scenarios;
- zero rays cause an undetected collision in every scenario;
- first-frame detection produces a safe stop in every scenario;
- an explicit late-detection fixture produces a collision;
- swept collision cannot skip the smallest hazard;
- mesh and collision footprints preserve the intended cylinder dimensions;
- non-hazard object IDs do not trigger braking.

### Acceptance gates

The completed replacement satisfies:

1. Zero-ray trials collide in 100% of generated scenarios.
2. First-frame braking and 361-ray trials safely stop in at least 99%.
3. Safe-stop rate and mean unbraked TTC are non-decreasing across nested ray
   counts.
4. Both tracers return identical hit flags and object IDs, with ranges equal
   within the existing geometry tolerance.
5. At some tested budget, scene-BVH maps to at least twice as many rays as
   true-brute.
6. At some tested budget, the mapped ray counts differ by at least 10 percentage
   points in safe-stop rate, with a paired 95% confidence interval excluding
   zero.
7. At the convergence budget, both modes map to 361 rays and therefore the same
   safety result.
8. A clean build, tests, CSV generation, and plots pass end to end.

If future results fail gates 5 or 6, report only acceleration and
detection-latency results. Do not introduce another safety metric to force
separation.

### Completed implementation order

1. Propagate object IDs into scan points.
2. Define and test the phased, nested horizontal layouts.
3. Implement deterministic scenarios and the straight-line braking trial.
4. Add fixed-ray safety tests and trial CSV output.
5. Validate zero-ray, first-frame, and full-resolution controls.
6. Add direct timing and p95 budget mapping.
7. Add aggregation, paired intervals, and plots.
8. Run all acceptance gates.

## Scope held for future experiments

These were intentionally not included in the completed first pass:

- mesh-BVH as a third benchmark mode;
- additional speeds, background-count sweeps, and mesh-complexity sweeps;
- live deadline-limited closed-loop trials;
- vertical layouts and progressive 2D ordering;
- ground, debris, boxes, cones, panels, and barriers;
- oriented-box collision and generalized mesh factories;
- replacing `ActiveObstacle` with `SceneObject`;
- simplifying `vehicle.hpp`;

Future extensions should be added one at a time and evaluated as separate
experiments.
