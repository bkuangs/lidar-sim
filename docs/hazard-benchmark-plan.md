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
mesh-bvh
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
- identical layouts produce identical hits across all three tracers;
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
4. All three tracers return identical hit flags and object IDs, with ranges equal
   within the existing geometry tolerance.
5. At some tested budget, scene-BVH maps to at least twice as many rays as
   true-brute.
6. At some tested budget, the mapped ray counts differ by at least 10 percentage
   points in safe-stop rate, with a paired 95% confidence interval excluding
   zero.
7. At the convergence budget, all modes map to 361 rays and therefore the same
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

## Geometry-preserving mesh-complexity extension

This extension asks one additional question: does the measured per-mesh BVH
benefit remain robust as triangle count increases while physical surface
geometry and all other controls remain fixed?

### Locked controls

Only timing-mesh triangle count changes. The extension retains:

- the same 100 timing objects and object IDs;
- the same pillar positions, radii, heights, `12x4` slices/stacks, silhouettes,
  AABBs, and collision footprints;
- the same 32 poses, nine nested ray layouts, and three tracer modes;
- 20 warmup and 200 measured complete scans per timing row;
- the existing safety scenarios, braking model, budgets, and metrics.

The safety CSV has no complexity dimension and is not regenerated for
compute-only levels. Each `(complexity, mode)` timing series maps onto the same
validated mode-independent fixed-ray safety curve.

### Predeclared complexity levels

Each subdivision pass replaces every existing triangle with four coplanar
triangles formed from edge midpoints. It does not increase cylinder
slices/stacks or alter the represented piecewise surface.

| complexity | subdivision passes | triangles per mesh |
| --- | ---: | ---: |
| `1x` | 0 | 120 |
| `4x` | 1 | 480 |
| `16x` | 2 | 1,920 |

The timing CSV contains exactly
`3 complexities x 3 modes x 9 ray counts = 81` unique rows. Each complexity has
its own convergence budget equal to 110% of that level's slowest measured
361-ray p95.

### Robustness result

Do not assume or require monotonic latency or speedup across complexity. Report
**yes** only if mesh-BVH p95 is lower than true-brute p95 at every nonzero
tested ray count for all three predeclared levels; otherwise report **no** and
identify every exception. This result is descriptive, not a gate that may be
tuned. Report any adjacent-ray p95 inversion explicitly and retain the
predeclared discrete mapping rule rather than smoothing or rerunning to force
monotonicity. If the verdict is `NO`, emit one exception row per failing
complexity/ray pair with both measured p95 values.

The published run reports **yes**. At 361 rays, true-brute, mesh-BVH, and
scene-BVH p95 values are:

| complexity | true-brute (ms) | mesh-BVH (ms) | scene-BVH (ms) | per-mesh speedup |
| --- | ---: | ---: | ---: | ---: |
| `1x` | 10.828 | 0.394 | 0.154 | 27.5x |
| `4x` | 44.019 | 0.427 | 0.168 | 103.2x |
| `16x` | 185.844 | 0.508 | 0.200 | 365.8x |

At 0.5 ms, the mapped true-brute/mesh-BVH/scene-BVH ray counts are
`9/361/361`, `0/361/361`, and `0/257/361` for `1x`, `4x`, and `16x`.
Those map to safe-stop rates of `34.1%/100%/100%`, `0%/100%/100%`, and
`0%/100%/100%` on the unchanged fixed-ray curve.

The published timing has one adjacent-ray inversion: `1x` true-brute p95 is
5.358 ms at 65 rays and 4.823 ms at 129 rays. Consequently, the 5 ms mapping
uses 129 rays under the predeclared largest-fitting-measurement rule. This is
reported as timing noise, not interpreted as monotonic scaling.

### Extension acceptance gates

1. Subdivision produces exact `1x`/`4x`/`16x` counts while preserving object
   IDs, AABBs, and deterministic intersections within the existing tolerance.
2. All tracers retain complete hit/object/range parity at every complexity,
   pose, and ray layout.
3. Timing output has exactly 81 unique rows and retains all timing controls.
4. `plots/hazard_trials.csv` and fixed-ray safety outcomes remain byte-for-byte
   unchanged, with no safety duplication by complexity.
5. Analysis emits explicit complexity-keyed latency, budget-ray, mapped
   safe-stop, robustness, and per-level convergence results.
6. Existing scientific controls remain passing where applicable, regardless of
   robustness direction.
7. Clean build, tests, analyzer, generated artifacts, and reproducibility pass
   without another sweep or unrelated refactor.

## Fixed-complexity object-count extension

This extension asks one additional question: how does the incremental value of
the top-level scene BVH change as object count grows while mesh complexity and
all timing controls remain fixed?

### Locked controls and nested scenes

Object count is exactly `25, 50, 100, 200, 400`. Every object uses the `1x`
pillar mesh with zero subdivision passes, `12x4` slices/stacks, and exactly 120
triangles. The extension retains the same 32 poses, nine nested ray layouts,
three tracer definitions, 20 warmups, 200 measured complete scans, timing
boundaries, budgets, and safety curve.

The 25- and 50-object levels are spatially balanced nested subsets of PR 2's
exact 100-object set. Membership selection is separate from construction order:
selected PR 2 objects are always instantiated in original row-major order, and
deterministic low-discrepancy additions inside that set's bounds use one stable
canonical order for the 200- and 400-object levels. Object IDs, poses, radii, and
geometry do not change after an object first appears. All levels therefore occupy
one fixed domain rather than expanding scene bounds or adding objects outside
measured rays.

The safety experiment is not rerun or duplicated. Every count/mode timing series
maps onto `plots/hazard_trials.csv` and its validated mode-independent fixed-ray
safety curve.

### Output and measured result

`plots/hazard_object_count_timing.csv` contains exactly
`5 counts x 3 modes x 9 ray counts = 135` unique rows. The separate
`plots/hazard_object_count_analysis/` slice reports mesh-BVH -> scene-BVH
same-ray latency, budget-mapped ray and safe-stop gains, a per-count convergence
budget, explicit robustness outcomes, every adverse comparison, and timing
inversions along both ray-count and object-count axes.

At 361 rays, the measured incremental results are:

| objects | mesh-BVH p95 (ms) | scene-BVH p95 (ms) | speedup |
| ---: | ---: | ---: | ---: |
| 25 | 0.393 | 0.408 | 0.96x |
| 50 | 1.102 | 0.439 | 2.51x |
| 100 | 1.503 | 0.512 | 2.94x |
| 200 | 1.126 | 0.350 | 3.22x |
| 400 | 3.179 | 0.404 | 7.86x |

The result is mixed and non-monotonic at low counts. Scene-BVH p95 is not lower
at every nonzero ray count for 25 or 50 objects (3 and 1 exceptions), but it is
lower at all eight nonzero ray counts for 100, 200, and 400 objects. At the
0.5 ms budget, scene-BVH adds 0, 232, 192, 232, and 296 mapped rays over
mesh-BVH at 25, 50, 100, 200, and 400 objects. Every mapped pair is already on
the fixed curve's 100% safe-stop plateau, so safe-stop gain is zero at every
standard budget and count.

The measured run has 4 adjacent-ray and 24 adjacent-count p95 decreases. They
remain in the published inversion table and the discrete mapping still selects
the largest individually measured ray count whose p95 fits. No placement,
metric, smoothing, or rerun was used to force a trend.

### Extension acceptance gates

1. All five scenes have exact unique counts, are deterministic nested subsets,
   preserve the exact PR 2 100-object construction sequence and one fixed spatial
   domain, and use exactly 120 triangles per object.
2. True-brute, mesh-BVH, and scene-BVH have identical hit flags, object IDs, and
   ranges within `1e-6` at every count, pose, and layout.
3. Timing output has exactly 135 unique rows with unchanged timing methodology
   and construction exclusions.
4. Safety trials, fixed-ray safety outputs, and PR 2 mesh-complexity artifacts
   remain byte-for-byte unchanged.
5. Every count reports same-ray p95 reduction/speedup, budget ray gain, mapped
   safe-stop gain, and convergence, including explicit mixed/regressed results.
6. Every scaling claim is backed by reported data, with exceptions and inversions
   visible.
7. Existing scientific controls, build, tests, analyzers, and artifact
   reproducibility pass without a mesh-by-object matrix or speed sweep.

## Matched mesh-complexity by object-count interaction matrix

This extension asks one additional question: do the separate acceleration
conclusions remain valid across a small matched matrix of mesh complexity and
object count, or do joint interactions change them?

### Locked matrix and timing controls

The matrix is exactly:

| factor | levels |
| --- | --- |
| mesh complexity | `1x`, `4x`, `16x` |
| triangles per object | 120, 480, 1,920 |
| object count | 25, 100, 400 |

The mesh levels use the same exact coplanar subdivision as the complexity
extension. The count levels use the validated deterministic nested subsets and
canonical fixed-domain construction from the object-count extension. Each cell
therefore changes only the two declared factors. Object positions, IDs, radii,
heights, physical surfaces, domain, 32 poses/layout phases, nine ray counts, and
the three tracer definitions remain fixed.

Every timing row retains 20 warmup and 200 measured complete scans. World and BVH
construction, CSV output, and aggregation stay outside timing. Hit flags, object
IDs, and ranges must match across all three tracers at every cell, pose, and
layout before that cell is timed.

The matrix is generated as one fresh coherent batch. It does not join timing
values from either prior sweep. The safety experiment remains byte-identical and
is reused once for budget mapping rather than duplicated by matrix cell.

### Output and descriptive analysis

`plots/hazard_complexity_object_count_timing.csv` contains exactly
`3 complexities x 3 counts x 3 modes x 9 ray counts = 243` unique rows. The
separate `plots/hazard_complexity_object_count_analysis/` publication reports:

- same-ray median and p95 reductions and speedups for true-brute to mesh-BVH and
  mesh-BVH to scene-BVH in every cell;
- standard-budget and per-cell convergence mappings to rays and the unchanged
  fixed-ray safe-stop curve;
- whether each acceleration transition wins at every nonzero ray in each cell,
  with every adverse or equal comparison retained;
- every adjacent p95 decrease along the ray-count, mesh-complexity, and
  object-count axes; and
- simple tables and faceted heatmaps without a fitted interaction model.

No monotonicity, significance threshold, or favorable result is required. The
publication states the measured conclusion even if either separate-sweep result
fails in a corner.

### Measured result

The per-mesh conclusion remains valid across the full matrix: mesh-BVH p95 is
lower than true-brute at every nonzero ray count in all nine cells. The
scene-level conclusion is conditional. Scene-BVH p95 is lower than mesh-BVH at
every nonzero ray count in five cells, but six adverse comparisons occur in four
cells:

| complexity / objects | adverse mesh-BVH to scene-BVH ray counts |
| --- | --- |
| `1x / 25` | 33 |
| `4x / 25` | 129 |
| `16x / 25` | 9, 33, 65 |
| `16x / 100` | 257 |

All `1x/100`, `1x/400`, `4x/100`, `4x/400`, and `16x/400` comparisons improve
at every nonzero ray count. At 361 rays, both acceleration transitions improve
in every cell; the true-brute/mesh-BVH/scene-BVH p95 values are:

| complexity | objects | true-brute (ms) | mesh-BVH (ms) | scene-BVH (ms) |
| --- | ---: | ---: | ---: | ---: |
| `1x` | 25 | 5.997 | 0.210 | 0.210 |
| `1x` | 100 | 24.235 | 0.980 | 0.386 |
| `1x` | 400 | 186.572 | 4.595 | 0.372 |
| `4x` | 25 | 30.627 | 0.240 | 0.230 |
| `4x` | 100 | 104.073 | 1.902 | 0.450 |
| `4x` | 400 | 628.554 | 5.594 | 0.483 |
| `16x` | 25 | 124.166 | 0.343 | 0.313 |
| `16x` | 100 | 443.368 | 1.111 | 0.359 |
| `16x` | 400 | 3436.750 | 3.603 | 0.373 |

The measured answer is therefore that joint interactions do not overturn the
per-mesh BVH result, but they do expose scene-BVH overhead at low object count
and one `16x/100` ray setting. The batch also reports 10 ray-axis, 40
complexity-axis, and 14 object-count-axis adjacent p95 decreases without
smoothing them or interpreting them as monotonic scaling.

### Extension acceptance gates

1. The batch has exactly nine deterministic cells with the declared triangle and
   object counts in one fixed domain; only the two factors vary.
2. True-brute, mesh-BVH, and scene-BVH have identical hit flags, object IDs, and
   ranges within `1e-6` at every cell, pose, and layout.
3. Timing output has exactly 243 unique rows with unchanged 20/200 timing
   methodology and construction exclusions.
4. Safety trials, mesh-sweep timing and analysis, and object-sweep timing and
   analysis remain byte-for-byte unchanged.
5. Every cell reports both same-ray transitions, budget-mapped rays/safe stops,
   and convergence.
6. Robustness checks and every adverse comparison are published without an
   unjustified statistical model or invented threshold.
7. Every ray-axis and factor-axis timing inversion is explicit, and the measured
   conclusion is stated regardless of direction.
8. Clean build, tests, analyzers, and deterministic analysis regeneration pass
   without new factor levels, a speed sweep, a safety metric, or unrelated
   refactoring.

## Scope held for future experiments

These were intentionally not included in the completed first pass:

- speed sweeps and a combined speed operating envelope;
- live deadline-limited closed-loop trials;
- vertical layouts and progressive 2D ordering;
- ground, debris, boxes, cones, panels, and barriers;
- oriented-box collision and generalized mesh factories;
- replacing `ActiveObstacle` with `SceneObject`;
- simplifying `vehicle.hpp`;

Future extensions should be added one at a time and evaluated as separate
experiments.
