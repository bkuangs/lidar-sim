# Project Design: BVH-Accelerated LiDAR — Benchmark & Autonomy Coupling

Status: **Design agreed, implementation in progress**
Last updated: 2026-08-09

---

## 1. Motivation & the core problem

The original goal — *"measure the effectiveness of LiDAR with BVH ray tracing vs brute force"* — has a hole in it as stated.

A scene-level acceleration structure (BVH, x-buckets) is a **pure latency optimization that returns bit-identical hits** to brute force. This is already asserted in code by `verifyAcceleratedQueries`. Because the control loop also uses a fixed `dt`, the planner sees identical ranges and the vehicle drives an identical path in every mode. Therefore:

> **Obstacles-avoided, damage-taken, clearance, and path are identical between BVH and brute force.** The accelerator and the autonomy outcome are *orthogonal* unless compute cost is fed back into behavior.

Comparing navigation quality across acceleration modes is meaningless. The accelerator only differs on **one axis: how much perception you can afford per unit time.**

This is confirmed empirically by the headless benchmark: all three modes drive identical distance and take identical collisions; only scan latency differs.

## 2. Core thesis

> **Faster ray tracing → more perception per unit time → measurably better autonomy.**

We make "BVH vs brute" a real, causal claim by coupling compute cost back into driving behavior through a **fixed real-time budget**.

## 3. Architecture: two layers joined by a cost model

```
Layer 1 (systems)                         Layer 2 (autonomy)
raytracing scaling benchmark   --fits-->  closed-loop sim under a fixed frame budget
brute vs buckets vs BVH        cost model  budget -> ray count -> resolution -> avoidance
latency/throughput vs N        (a,b,g)     collision-rate vs speed, per mode
```

Layer 1 **measures** the real cost curves; Layer 2 **consumes** them as a deterministic cost model. This keeps Layer 2 reproducible (not wall-clock/machine dependent) while staying grounded in real measurement.

### Layer 1 — Raytracing scaling benchmark (headless, no Open3D)

- **Sweep:** primitive count `N = {10, 50, 100, 500, 1k, 5k, ...}` (dense obstacle field; optionally high-poly meshes to also stress the per-mesh BVH).
- **Per mode (brute / buckets / BVH):** time `build_ms` and `query_ms` (full budgeted scan) **separately**, over many poses + warmup → mean / p50 / p95, rays/sec.
- **Correctness gate:** every mode must match brute exactly (full-scan extension of `verifyAcceleratedQueries`).
- **Output:** CSV → scaling chart (brute `O(N)` vs BVH `~O(log N)`, build-cost crossover) **and** the fitted cost model consumed by Layer 2.

### Layer 2 — Closed-loop autonomy under a fixed frame budget

- Each frame gets a fixed compute budget `B`. The accelerator determines **how many rays fit** → angular resolution → detection quality.
- **Coupling mechanism: resolution adaptation** (chosen over reaction-latency as more on-thesis for "LiDAR effectiveness"). Faster tracer → more rays → denser cloud → detects small obstacles the starved scan **aliases past**.
- **Scene isolates perception:** wide corridor so maneuvering is never the bottleneck → any collision difference is attributable to resolution (i.e., to the accelerator).
- **Headline metric:** **collision-rate vs speed, one curve per mode** — BVH sustains higher speed without crashing. Secondary: *smallest reliably-detected obstacle size* per mode.

## 4. The mechanism, quantified

### 4.1 Detection (aliasing)

An obstacle is detected once at least one ray strikes it, i.e. its angular width exceeds ray spacing. Smallest reliably-detected width at range `r`:

```
w_min ≈ r · Δθ          (Δθ = angular spacing in radians, K = FOV / Δθ)
```

| Resolution | Δθ (rad) | w_min @4m | @6m | @8m |
|---|---|---|---|---|
| 0.5°/ray | 0.0087 | 0.035 | 0.052 | 0.070 |
| 1°/ray   | 0.0175 | 0.070 | 0.105 | 0.140 |
| 2°/ray   | 0.0349 | 0.140 | 0.209 | 0.279 |
| 4°/ray   | 0.0698 | 0.279 | 0.419 | 0.559 |
| 8°/ray   | 0.1396 | 0.559 | 0.838 | 1.117 |

Over a 180° forward arc: `K = 180 / (deg per ray)` → 1° = 180 rays, 4° = 45 rays, 8° = 23 rays.

### 4.2 When a miss becomes a collision

An obstacle becomes visible within `r_detect = w / Δθ`. A collision occurs only if it is detected too late to steer around:

```
Collision  ⇔  r_detect(w, Δθ_mode) < r_req(v)
```

`r_req(v)` = longitudinal distance needed to clear the obstacle laterally. With the kinematic bicycle model the turn radius `R = wheelbase/tan(steer) ≈ 2.77 m` is **speed-independent**, so speed couples in via the **steering-rate limit** (90°/s → ~0.33 s to full lock):

```
r_req(v) ≈ L_maneuver + v · t_ramp ≈ 1.8–2.3 m + v · 0.33
```

Approx values: `v=2 → ~2.7m`, `v=4 → ~3.4m`, `v=6 → ~4.0m`, `v=8 → ~4.7m`.

`r_detect = w / Δθ`:

| Δθ | w=0.25 | w=0.5 | w=0.75 |
|---|---|---|---|
| **8°** (brute target) | 1.8 m | 3.6 m | 5.4 m |
| 4° | 3.6 m | 7.2 m | 10.7 m |
| **1°** (BVH target) | 14.3 m | 28.6 m | 43 m |

Read-off:
- **Brute @ 8°/ray:** 0.25 m obstacles collide even at v=2; 0.5 m collide at v≳6; 0.75 m safe → a smooth collision-rate rising with speed.
- **Brute @ 4°/ray:** only smallest obstacles collide, only at high speed → weak story.
- **BVH @ 1°/ray:** `r_detect` ≫ `r_req` for all sizes → essentially never collides.

## 5. Locked design parameters

| Param | Value | Rationale |
|---|---|---|
| Obstacle width | `U(0.25, 0.75) m` | straddles the collision threshold → smooth rate curve |
| Corridor half-width | 1.5–2.5 m | wide → maneuvering never the bottleneck (perception isolated) |
| Scan shape | thin elevation band × `K` azimuth, **budgeted** | pay only for perception used; denser cloud for BVH = good demo |
| Elevation bands | 1 (default; expandable to 3) | slice is ±3° anyway |
| Budget covers | scan only (build amortized, excluded) | build is O(N log N) over dozens of frames → negligible |
| Cost model | `K = ⌊B / cost_ray(mode, N)⌋` | deterministic, fed by Layer 1 fit |
| **Calibration target** | **brute ≈ 8°/ray, BVH ≤ 1°/ray at chosen (N, B)** | the condition for a measurable effect |
| Headline metric | collision-rate vs speed, per mode | + smallest reliably-detected obstacle |

### 5.1 Cost model forms

```
brute:   cost_ray = α + β·N              (tests all N obstacles)
buckets: cost_ray = α + β·ρ(N)           (ρ = obstacles along ray's x-span)
bvh:     cost_ray = α + γ·log2(N)
K(mode)  = ⌊ B_budget / cost_ray(mode, N) ⌋
```

Ratio `K_bvh / K_brute = (α + β·N) / (α + γ·log2 N)` grows ~`N/log N`. Choose `(N, B)` so brute lands at 4–8°/ray while BVH stays ≤1°/ray.

## 6. Success criteria

- **Layer 1:** correctness-gated timing across all modes; fitted `α, β, γ`; **confirm brute can be pushed to ~8°/ray at a feasible N.** (If not achievable, the coupling story weakens — this is the key risk to retire first.)
- **Layer 2:** reproducible (deterministic cost model, fixed pre-generated course); collision-rate-vs-speed curves that **separate by mode**; BVH sustains higher safe speed than brute.

## 7. Explicitly out of scope / reframed

- "LiDAR effectiveness: BVH vs brute" as a *navigation-quality* comparison — incoherent (identical perception).
- Reaction-latency coupling (considered, not chosen; may add later as a second axis).
- Full GPU ray tracing, SLAM/mapping, rich vehicle dynamics.

## 8. Repo impact (overhaul in place — do NOT start from scratch)

Core algorithms are strong and reused verbatim; the work is orchestration overhaul + additive benchmark pieces.

| File | Verdict | Notes |
|---|---|---|
| `geometry.hpp` | KEEP as-is | primitives, per-mesh BVH, Möller–Trumbore, SAH, AABB slab |
| `scene.hpp` | KEEP | the brute-force baseline mode |
| `accelerated_scene.hpp` | KEEP + extend | added `SceneBVH` to `QueryMode` ✅ |
| `scene_bvh.hpp` | KEEP | new top-level BVH ✅ |
| `planner.hpp` | KEEP | unseen bins = free → the aliasing behavior we want |
| `vehicle.hpp` | KEEP | bicycle model + steering-rate limit = speed coupling |
| `scan.hpp` | minor | honor parameterized ray count |
| `lidar.hpp` | OVERHAUL (small) | parameterize `(elevation_bands, K_azimuth)` |
| `hallway.hpp` | OVERHAUL (moderate) | new obstacle regime + fixed pre-generated course |
| `simulation.hpp` | OVERHAUL (largest) | decouple from real-time; budget→resolution; cost model; new metrics |
| `main.cpp` | KEEP (demo only) | not used by benchmarks |
| `harness.hpp` | NEW | headless runner ✅ |
| `benchmark.cpp` | NEW | Layer-1 seed driver ✅ |
| `tests.cpp` | KEEP + extend | SceneBVH equivalence (shared oracle) ✅ |
| `CMakeLists.txt` | ADD (gitignored/local) | headless targets, no Open3D |
| `visualize.cpp` | NUKE (pending) | dead standalone PLY viewer |

Rough split: ~65% kept verbatim, ~20% overhauled (orchestration), ~15% new (harness, benchmarks, cost model).

## 9. New components to add

- **Headless harness** — done (`harness.hpp`, `benchmark.cpp`).
- **Scaling-scene generator** — dense N sweep for Layer 1. *(next)*
- **Layer 1 benchmark exe** — CSV + correctness gate + cost-model fit.
- **Cost model** — `α, β, γ` consumed by Layer 2.
- **Layer 2 benchmark exe** — collision-rate-vs-speed sweep over `{mode × speed × seeds}`.
- **(optional) Plotting script** — Python, for the charts.

## 10. Build sequence

1. ✅ Headless harness (no viewer/sleep).
2. ✅ Wire in `SceneBVH` as a 3rd `QueryMode` + extend the correctness oracle.
3. Dense scene generator (scale N). *(next)*
4. Layer 1 benchmark exe → CSV + correctness gate.
5. Fit cost model (`α, β, γ`) from Layer 1.
6. Layer 2 resolution-adaptation coupling using the cost model.
7. New behavior metrics + terminal-crash handling.
8. Layer 2 benchmark exe → sweep → CSV.
9. (optional) Python plotting.

## 11. Open items / risks

- **Key risk:** can Layer 1 push brute to ~8°/ray at feasible N? Retire this first (steps 3–5).
- Machine-specific constants `α, β, γ` set final `(N, B)` — unknown until Layer 1 runs.
- Determinism across modes: pre-generate a fixed course so every mode faces identical obstacles (positions already seeded independently of vehicle path).
- Fairness of buckets vs BVH build accounting (currently excluded from budget).

## 12. Progress log

- **2026-08-09:** Headless harness (`harness.hpp` + `benchmark.cpp`, Eigen-only target) and `SceneBVH` (3rd `QueryMode`, correctness-gated) landed. Benchmark confirms identical behavior across modes with BVH lowest latency (seed 42, 300 frames: brute 1.35ms / buckets 1.22ms / BVH 0.94ms mean scan; all verified).
- **2026-08-09:** Layer 1 scaling benchmark (`scaling.cpp`, `lidar_scaling` target) landed. **Key risk retired.**
- **2026-08-09:** Renamed Layer 1's baseline mode `brute-force → linear-scan` (`bvh_vs_bf → bvh_vs_scan`) for honesty — see terminology note below.
- **2026-08-09:** Added a Layer 2 correctness gate (`verifyLayer2Scene` in `layer2.hpp`): samples rays across poses on the actual pillar course and asserts `SceneBVH` hits match a triangle-soup ground truth (`bruteForceIntersect`, no per-mesh BVH) — checks both scene-level traversal and per-mesh path. `layer2_benchmark` runs it fail-fast before the sweep (3640 rays/seed × 8 seeds, 0 mismatches). Verified the gate has teeth (cross-check against mismatched geometry flags 1525/3640).
- **2026-08-09:** Cost-model provenance retired the hand-transcribed single-point constants. New `calibrate.cpp` (`calibrate` target) times all three tracers on the **actual pillar geometry** over a grid of `N ∈ {25,50,100,200}` × `stacks ∈ {4,8,16}` (tris 280/504/952) and fits the assumed forms: **brute `ns = 3605 + 3.441·(N·tris)`, R²=0.998**; **mesh `ns = 45.7 + 10.32·N`, R²=0.983**; scene ~flat few-hundred ns (log₂N signal swamped by timing noise, R²=0.67, nonsensical negative intercept). Baked the fitted slopes into `CostModel`; scene modeled as a conservative flat constant (it always caps K within any realistic budget — its real O(log N) scaling is the Layer 1 latency result). Also switched the cost model to count the **total** obstacle set, not just those in view (**choice A**: brute/mesh have no spatial culling, so they test every obstacle on every ray). Net effect: brute cost ~2.4× higher, K constant per run, stronger and cleaner starvation curve.
- **2026-08-09:** Resolved loose end #3 (reach% variance / stuck-run contamination). `layer2_benchmark` now scores each seed's coll/100m **individually** and reports **mean ± sample std** over the **common finisher set** (seeds every mode completes at that budget → identical courses cross-mode; `n` = its size), and reports **reach%** as a separate first-class outcome. Non-finishing (stuck / frame-capped) runs are excluded from the safety metric instead of being blended into a distance-weighted pool. Default seeds bumped 8 → 32 for statistical power (common `n` ≈ 11 in the clean 1–3 ms band, ~66 s runtime). Surfaced that the follow-the-gap planner legitimately deadlocks on hard courses and that *better perception lowers reach* (coarse brute drives through phantom gaps) — which is precisely why reach% and safety must be reported separately.

> **Terminology — two different "brutes."** These are NOT the same baseline:
> - **Layer 1 `linear-scan`** = the O(N) scene walk (`Scene::intersect` tests the ray against *every* object; each object still uses its own per-mesh BVH / analytic form). Obstacles here are analytic spheres / 12-tri cubes, so per-object cost is trivial — this isolates **scene-level scaling (N vs log N)**. scene-BVH's win here is a pure **top-level-tree / latency** result.
> - **Layer 2 `true-brute`** = triangle-soup (`TriangleMeshGeometry::bruteForceIntersect`, no per-mesh BVH), O(N·tris). This is the expensive textbook baseline used to *starve the ray budget* in the autonomy loop.
>
> Consequence: `linear-scan` (a.k.a. the mesh-BVH tier) can only be beaten by scene-BVH in **latency**, and only diverges at high N — which is why the mesh-BVH-vs-scene-BVH separation can't be reproduced as a *driving* (collision) result (starving it needs undrivable density). The scene-BVH-vs-`linear-scan` win lives here in Layer 1.

### Layer 1 results (seed 42, 4344 rays/pass, 50m corridor, 12-tri cubes / analytic spheres)

| N | linear-scan ns/ray (O(N)) | buckets ns/ray | bvh ns/ray | bvh speedup |
|---|---|---|---|---|
| 10 | 126 | 174 | 50 | 2.5× |
| 50 | 259 | 279 | 69 | 3.7× |
| 100 | 597 | 334 | 54 | 11.0× |
| 200 | 973 | 515 | 52 | 18.6× |
| 400 | 2031 | 1090 | 61 | 33× |
| 800 | 4887 | 2380 | 70 | 70× |
| 1600 | 10509 | 7497 | 99 | 106× |
| 3200 | 23125 | 15866 | 98 | 237× |

Findings:
- **The linear scan is O(N); BVH is flat (~O(log N)); buckets grows ~linearly** (∝ local density) and degrades toward the linear scan in dense scenes → BVH is the decisive accelerator.
- All accelerated modes bit-verified against the linear-scan reference at every N.
- BVH build cost is negligible (0.006 ms @ N=10 → 1.3 ms @ N=3200), amortizable across a window.
- The scene-BVH vs linear-scan speedup reaches **~237–246× at N=3200** — the high-N scene-BVH win, expressed as latency.

### Open decision surfaced by Layer 1 — the Layer 2 operating point

Starving brute at a **realistic ms-scale budget** needs unrealistic N with trivial obstacles (~30k @ 5ms). **Resolved: high-poly obstacles + a true triangle-soup brute baseline.**

Key subtlety: `Scene::intersect` already uses each obstacle's per-mesh BVH, and most rays *miss* a small obstacle's AABB in one cheap test — independent of triangle count. So high-poly only starves brute if "brute" is the **true triangle-soup baseline** (`TriangleMeshGeometry::bruteForceIntersect`, no per-mesh BVH). That is the honest textbook brute-vs-BVH comparison.

Validated by `poly_probe.cpp` (16×16 UV spheres, 480 tris each):

| N | total tris | true-brute ns/ray | mesh-BVH ns/ray | scene-BVH ns/ray | K@1ms | K@5ms |
|---|---|---|---|---|---|---|
| 100 | 48k | 211,416 | 977 | 75 | 5 | **24** |
| 200 | 96k | 425,941 | 1834 | 75 | 2 | 12 |
| 400 | 192k | 884,187 | 3557 | 99 | 1 | 6 |

**Locked Layer 2 operating point:** ~100 high-poly obstacles in a 50 m corridor, **5 ms perception budget** → true-brute ≈ 8°/ray (24 rays, aliases ~half of 0.25–0.75 m obstacles); scene-BVH capped at full res (≤1°/ray). Realistic obstacle count, realistic budget, clean starvation.

Three modes worth showing: **true-brute** (triangle soup), **mesh-BVH** (per-mesh only, scene-level brute — still O(N)), **scene-BVH** (both levels). Both BVH tiers matter.

### Layer 2 results (`layer2.hpp` / `layer2_benchmark` target)

Closed loop lives in `layer2.hpp` (`makeLayer2World` + `runLayer2`); driver `layer2_benchmark.cpp`. High-poly pillars via `meshes.hpp` (`makePillar`, **504 tris @ 28×8** = 2·slices·(stacks+1)). Course: 120 m corridor, **half-width 5 m**, ~71 full-height pillars (widths U(0.25,0.75), random y, along-x spacing U(1.0,2.2)). Each frame: **calibrated** cost model over the **total** obstacle count → afford `K` azimuth rays → trace scene-BVH → follow-the-gap → bicycle model → collision check. All modes trace identical (correct) hits; only `K` differs by mode+budget.

**Metric fix that mattered:** absolute collisions and per-obstacle rates are confounded — a starved/blind mode drives *further per frame*, so it "passes" more obstacles and travels more. Normalise by distance: **collisions per 100 m**.

**Aggregation fix (loose end #3, resolved):** three problems were bundled together. (1) *No spread* — a single pooled ratio per cell gave no sense of seed-to-seed variance. (2) *Non-finishing runs* — every run ends by finishing (x ≥ 120 m), getting stuck (speed ≈ 0 for 90 frames), or hitting the 6000-frame cap; the old pool blended partial runs in. (3) *Distance-pooling skew* — pooling by total distance let a run that stalled at 40 m distort the ratio. Fixes: score each seed's coll/100m **individually** and report **mean ± sample std** (equal-weighted, no distance pool); score only over the **common finisher set** (seeds *every* mode completes at that budget → identical courses across modes, `n` = its size); and report **reach%** as its own first-class outcome so a stuck run can never masquerade as "safe." Key finding this exposed: the follow-the-gap planner *legitimately* deadlocks (correctly sees a fully-blocked ±90° arc → stops) on the harder courses, and **better perception → lower reach** (coarse brute misses pillars, finds a phantom gap, drives through colliding). That's exactly why reach% must be separated from safety. (Planner has no recovery behavior — creep/reverse is out of scope; the metric is now robust to it.)

**Headline (32 seeds, target 3.5 m/s, sweep budget; cost model = fitted constants, total-N; safety = mean ± std over the n common finishers):**

| budget (ms) | brute K | true-brute coll/100m | BVH coll/100m | n | brute reach% | BVH reach% |
|---|---|---|---|---|---|---|
| 1.00 | 8 | **6.99 ± 2.70** | 2.80 ± 1.45 | 11 | 100% | 34% |
| 1.50 | 12 | 5.99 ± 2.33 | 2.80 ± 1.45 | 11 | 100% | 34% |
| 2.00 | 16 | 4.70 ± 1.80 | 2.80 ± 1.45 | 11 | 100% | 34% |
| 3.00 | 24 | 3.34 ± 1.60 | 2.80 ± 1.45 | 11 | 94% | 34% |
| 5.00 | 40 | 2.55 ± 0.98 | 2.89 ± 1.53 | 7 | 66% | 34% |
| 8.00 | 64 | 2.43 ± 1.32 | 2.83 ± 1.53 | 10 | 53% | 34% |

- **Monotone starvation curve, now with error bars:** shrinking the budget starves brute (K 24→8), it aliases small pillars, collisions/100m climb 3.3 → 7.0. At 1 ms brute is **~2.5× the BVH floor**; with n=11 the standard error is std/√11 ≈ 0.8, so the two means (6.99 vs 2.80) are separated by ~5 SE — a real gap, not seed noise.
- **Both BVH tiers flat at ~2.80** regardless of budget — always afford full angular resolution. This floor is the *control-tracking* limit (weaving with a steering-rate-limited bicycle), independent of perception — the clean baseline the accelerator is measured against. The **excess above it is the pure aliasing penalty** bought back by the accelerator.
- **Convergence:** by 3–5 ms brute affords enough rays to resolve the pillars and meets the floor — the expected "enough budget → no penalty" behavior.
- **reach% is now honest and separate:** brute's reach *falls* (100% → 53%) as budget grows because better perception makes it correctly stop at genuinely blocked spots (safe, but a non-completion); BVH sits at a steady 34% (deadlock is course-driven, not perception-driven). The safety numbers above are scored only on courses **all** modes finished, so this asymmetry cannot leak into the coll/100m comparison. The 5/8 ms rows have smaller `n` (fewer common finishers) — hence their wider/looser figures — but the clean, high-`n` signal is the 1–3 ms band.
- **mesh-BVH == scene-BVH at this N** (~71 total, ≈30 in view): per-mesh AABB culling alone already affords full res, so both pin K=361. Honest finding — the *scene*-level BVH's separate win only emerges at much higher obstacle counts (where scene-level brute's O(N) starves too). The dominant, robust contrast here is **true-brute vs any BVH**.

**Speed axis** is secondary/noisy at this operating point (steering-rate coupling is weak vs the resolution effect); **budget is the clean primary axis** and tells the whole story: faster ray tracing → more rays afforded → fewer collisions.

