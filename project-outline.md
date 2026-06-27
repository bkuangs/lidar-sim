# Follow-The-Gap LiDAR Collision Avoidance Benchmark

## Problem Statement

How might we build a visually compelling 3D LiDAR simulation where an Ackermann-style vehicle avoids randomized hallway obstacles using follow-the-gap, while comparing reaction latency between brute-force and accelerated scene queries?

## Recommended Direction

Build a visual Open3D autonomy demo first, backed by a reusable benchmark loop.

The project should show a vehicle moving through the randomized hallway, scanning with 3D LiDAR, selecting a traversable gap, steering toward it with Ackermann constraints, and adjusting speed based on obstacle proximity. The viewer should make the autonomy obvious: point cloud, chosen gap/target direction, vehicle path, steering angle, speed, collision status, and latency numbers.

For the acceleration story, compare:

```text
brute-force Scene::intersect()
```

against:

```text
accelerated scene-level obstacle lookup
```

The best first accelerator is probably a hallway-aware spatial grid or x-axis bucket index, because the world is procedurally generated along `x`. A full scene BVH is more general and impressive, but the grid is simpler, easier to explain, and likely faster to implement correctly.

## Key Assumptions To Validate

- [ ] Follow-the-gap works well enough on the current 3D LiDAR data. Test by projecting LiDAR points into a 2D driving slice and verifying the selected gap avoids obstacles.
- [ ] Scene-level acceleration produces measurable latency improvement. Test with increasing obstacle counts and compare scan time per frame.
- [ ] Ackermann steering makes the demo more robotics-relevant without overcomplicating control. Test with a simple bicycle model before adding richer vehicle dynamics.
- [ ] Open3D can support the needed visual overlays cleanly. Test rendering point cloud, vehicle pose/path, and selected target direction in one loop.

## MVP Scope

MVP should include:

- Deterministic randomized hallway seed.
- Vehicle state: `x`, `y`, heading, speed, steering angle.
- Ackermann/bicycle-model update.
- 3D LiDAR scan each frame.
- 2D follow-the-gap planner from LiDAR ranges.
- Speed control: slow down near obstacles, accelerate in clear space.
- Brute-force scan mode.
- Accelerated scene-query mode.
- Live Open3D visualization.
- On-screen or console metrics: scan latency, planner latency, total loop latency, collisions, speed.

## Not Doing

- Full GPU ray tracing yet, because that is too much infrastructure before the autonomy story is proven.
- Complex SLAM or mapping, because this is reactive collision avoidance rather than localization.
- Perfect physical vehicle dynamics, because Ackermann-like behavior is enough for recruiting.
- General 3D navigation, because hallway driving keeps the scope sharp.
- Per-mesh BVH as the main benchmark, because the current obstacles are too simple for that to be meaningful.

## Open Questions

- Should acceleration start as x-axis buckets or a general scene BVH?
- Should follow-the-gap use only a horizontal LiDAR slice, or aggregate multiple elevation bands?
- Should benchmark output be CSV, terminal summary, or both?
- Should the visual demo include obstacle meshes too, or just point cloud plus path?

## Suggested Implementation Sequence

1. Refactor simulation into reusable update pieces.
2. Add vehicle dynamics.
3. Add follow-the-gap planner.
4. Add visual steering, path, and metrics.
5. Add brute-force vs accelerated scene mode.
6. Add benchmark logging.
