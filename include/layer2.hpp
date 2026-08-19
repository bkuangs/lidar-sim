#pragma once
#include "meshes.hpp"
#include "scene_bvh.hpp"
#include "obstacle.hpp"
#include "planner.hpp"
#include "scan.hpp"
#include "vehicle.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <random>
#include <vector>

/**
 * LAYER 2 — closed-loop autonomy under a fixed per-frame perception budget.
 *
 * Each mode traces with its real implementation until the per-frame deadline.
 * Only rays completed on time reach the planner. Fewer rays -> coarser angular
 * resolution -> small pillars are aliased past -> collisions.
 */

enum class Layer2Mode
{
    TrueBrute, // triangle soup, no acceleration
    MeshBVH,   // per-mesh BVH only, scene-level brute (still O(N))
    SceneBVH   // scene + per-mesh BVH
};

inline constexpr std::array<double, 7> layer2BudgetsMs = {
    0.5, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0};

inline const char *layer2ModeName(Layer2Mode m)
{
    switch (m)
    {
    case Layer2Mode::TrueBrute:
        return "true-brute";
    case Layer2Mode::MeshBVH:
        return "mesh-bvh";
    case Layer2Mode::SceneBVH:
        return "scene-bvh";
    }
    return "unknown";
}

// Cost per ray (ns). Fitted on the ACTUAL pillar geometry by calibrate.cpp
// (seed 42, macOS / clang -O2, 2026-08-09) and baked here (not measured live)
// for machine-independent reproducibility:
//   brute: ns = 3605 + 3.441 * (N*tris)   R2=0.998   (triangle soup, no culling)
//   mesh:  ns = 45.7  + 10.32 * N         R2=0.983   (O(N) object walk)
//   scene: ~a few-hundred ns, ~flat; the log2(N) signal is swamped by timing
//          noise (R2=0.67, nonsensical negative intercept) because scene-BVH is
//          so cheap. It always caps K within any realistic budget, so it is
//          modeled as a conservative constant; its true O(log N) scaling is the
//          Layer 1 (latency) result, not something this budget can resolve.
// The additive intercepts are dropped (negligible vs the slope term at Layer 2's
// operating point: brute's 3605 ns is ~3% of 128k ns/ray at N=72, tris=504).
struct CostModel
{
    double brute_ns_per_tri = 3.441;
    double mesh_ns_per_obstacle = 10.32;
    double bvh_base_ns = 550.0; // conservative flat estimate; scene always caps K
    int tris_per_obstacle = 504; // 2 * slices(28) * (stacks(8) + 1)

    double costRayNs(Layer2Mode m, int in_view) const
    {
        const int n = std::max(1, in_view);
        switch (m)
        {
        case Layer2Mode::TrueBrute:
            return brute_ns_per_tri * n * tris_per_obstacle;
        case Layer2Mode::MeshBVH:
            return mesh_ns_per_obstacle * n;
        case Layer2Mode::SceneBVH:
            // Modeled flat: the true O(log N) term is unresolvable here (see above)
            // and scene-BVH always caps K within budget, so the constant suffices.
            return bvh_base_ns;
        }
        return 0.0;
    }
};

struct Layer2Config
{
    unsigned int seed = 1;
    double course_length = 120.0;
    double spacing_min = 1.0;
    double spacing_max = 2.2;
    double corridor_half_width = 5.0;
    double height = 3.0;
    double min_size = 0.25;
    double max_size = 0.75;
    int pillar_slices = 28;
    int pillar_stacks = 8;

    double target_speed = 3.0;
    double budget_ms = 2.0;
    Layer2Mode mode = Layer2Mode::SceneBVH;

    int k_max = 361; // full-res cap over the 180-degree arc
    double max_range = 50.0;
    double view_behind = 5.0;

    double dt = 1.0 / 30.0;
    int max_observation_age_frames = 3;
    double collision_motion_tolerance = 0.01;
    int max_frames = 6000;
    bool stop_on_crash = false;
};

struct Layer2Result
{
    Layer2Mode mode = Layer2Mode::SceneBVH;
    double target_speed = 0.0;
    double budget_ms = 0.0;
    int obstacles_total = 0;
    int collisions = 0;
    double collisions_per_100m = 0.0;
    double distance = 0.0;
    double distance_to_first_collision = -1.0;
    double progress = 0.0;
    double avg_k = 0.0;
    double predicted_k = 0.0;
    double avg_in_view = 0.0;
    double cost_ns_per_ray = 0.0;
    double avg_scan_ms = 0.0;
    double p95_scan_ms = 0.0;
    double mean_overrun_ms = 0.0;
    double max_overrun_ms = 0.0;
    int min_k = 0;
    int max_k = 0;
    int frames = 0;
    int deadline_cutoffs = 0;
    int execution_overruns = 0;
    bool wall_contact = false;
    bool collision_free_completion = false;
    bool reached_end = false;
};

struct Layer2World
{
    std::vector<ActiveObstacle> obstacles;
    std::vector<std::shared_ptr<Geometry>> statics;
    SceneBVH bvh;
    int tris_per_obstacle = 0;
};

inline int affordableRayCount(double budget_ns, double cost_ray_ns, int k_max)
{
    if (budget_ns <= 0.0 || k_max <= 0)
        return 0;

    const double affordable = std::floor(budget_ns / std::max(1.0, cost_ray_ns));
    return static_cast<int>(std::min(affordable, static_cast<double>(k_max)));
}

inline Hit layer2ReferenceHit(const Layer2World &w, const Ray &ray, double max_distance);

inline Hit layer2MeshHit(const Layer2World &w, const Ray &ray, double max_distance)
{
    Hit closest;
    auto consider = [&](const Hit &h) {
        if (h.hit && h.t <= max_distance && (!closest.hit || h.t < closest.t))
            closest = h;
    };
    for (const auto &s : w.statics)
        consider(s->intersect(ray));
    for (const ActiveObstacle &o : w.obstacles)
        consider(o.geometry->intersect(ray));
    return closest;
}

inline ScanResult scanLayer2(
    const Layer2World &world,
    const Layer2Config &cfg,
    const Lidar &lidar,
    const std::vector<int> &azimuth_order)
{
    switch (cfg.mode)
    {
    case Layer2Mode::TrueBrute:
        return scanProgressiveWithIntersector(
            lidar, azimuth_order, cfg.k_max, [&](const Ray &ray) {
                return layer2ReferenceHit(world, ray, cfg.max_range);
            });
    case Layer2Mode::MeshBVH:
        return scanProgressiveWithIntersector(
            lidar, azimuth_order, cfg.k_max, [&](const Ray &ray) {
                return layer2MeshHit(world, ray, cfg.max_range);
            });
    case Layer2Mode::SceneBVH:
        return scanProgressiveWithIntersector(
            lidar, azimuth_order, cfg.k_max, [&](const Ray &ray) {
                return world.bvh.intersect(ray, cfg.max_range);
            });
    }
    return {};
}

inline ScanResult scanLayer2(
    const Layer2World &world,
    const Layer2Config &cfg,
    const Lidar &lidar)
{
    return scanLayer2(
        world, cfg, lidar, progressiveAzimuthOrder(cfg.k_max));
}

template <typename Now>
inline ScanResult scanLayer2UntilDeadline(
    const Layer2World &world,
    const Layer2Config &cfg,
    const Lidar &lidar,
    const std::vector<int> &azimuth_order,
    double budget_ns,
    Now now)
{
    switch (cfg.mode)
    {
    case Layer2Mode::TrueBrute:
        return scanProgressiveUntilDeadline(
            lidar,
            azimuth_order,
            cfg.k_max,
            budget_ns,
            [&](const Ray &ray) {
                return layer2ReferenceHit(world, ray, cfg.max_range);
            },
            now);
    case Layer2Mode::MeshBVH:
        return scanProgressiveUntilDeadline(
            lidar,
            azimuth_order,
            cfg.k_max,
            budget_ns,
            [&](const Ray &ray) {
                return layer2MeshHit(world, ray, cfg.max_range);
            },
            now);
    case Layer2Mode::SceneBVH:
        return scanProgressiveUntilDeadline(
            lidar,
            azimuth_order,
            cfg.k_max,
            budget_ns,
            [&](const Ray &ray) {
                return world.bvh.intersect(ray, cfg.max_range);
            },
            now);
    }
    return {};
}

inline ScanResult scanLayer2UntilDeadline(
    const Layer2World &world,
    const Layer2Config &cfg,
    const Lidar &lidar,
    const std::vector<int> &azimuth_order,
    double budget_ns)
{
    return scanLayer2UntilDeadline(
        world,
        cfg,
        lidar,
        azimuth_order,
        budget_ns,
        [] { return std::chrono::steady_clock::now(); });
}

inline Layer2World makeLayer2World(const Layer2Config &cfg)
{
    std::mt19937 rng(cfg.seed);
    std::uniform_real_distribution<double> size_dist(cfg.min_size, cfg.max_size);
    std::uniform_real_distribution<double> spacing_dist(cfg.spacing_min, cfg.spacing_max);

    Layer2World world;
    world.statics.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, 1), Vec3(0, 0, 0), 0));
    world.statics.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 0, -1), Vec3(0, 0, cfg.height), 1));
    world.statics.push_back(std::make_shared<PlaneGeometry>(Vec3(0, 1, 0), Vec3(0, -cfg.corridor_half_width, 0), 2));
    world.statics.push_back(std::make_shared<PlaneGeometry>(Vec3(0, -1, 0), Vec3(0, cfg.corridor_half_width, 0), 3));

    int object_id = 10;
    double x = 6.0;
    while (x < cfg.course_length)
    {
        const double size = size_dist(rng);
        const double radius = size * 0.5;
        const double margin = radius + 0.1;
        std::uniform_real_distribution<double> y_dist(
            -cfg.corridor_half_width + margin, cfg.corridor_half_width - margin);
        const double y = y_dist(rng);

        auto pillar = makePillar(x, y, radius, cfg.height, cfg.pillar_slices, cfg.pillar_stacks, object_id);

        ActiveObstacle o;
        o.x = x;
        o.center = Vec3(x, y, cfg.height * 0.5);
        o.size = size;
        o.object_id = object_id;
        o.bounds = pillar->bounds();
        o.geometry = pillar;
        world.obstacles.push_back(o);

        ++object_id;
        x += spacing_dist(rng);
    }

    world.tris_per_obstacle = cfg.pillar_slices * cfg.pillar_stacks * 2 + 2 * cfg.pillar_slices;
    world.bvh.rebuild(world.statics, world.obstacles);
    return world;
}

inline Layer2Result runLayer2(const Layer2Config &cfg)
{
    Layer2World world = makeLayer2World(cfg);

    VehicleConfig vcfg;
    vcfg.max_speed = cfg.target_speed + 1.0;
    VehicleState veh;

    PlannerConfig pcfg;
    pcfg.max_speed = cfg.target_speed;
    PlannerScanState planner_scan_state(pcfg.bins);

    Lidar lidar;
    lidar.pose = lidarPoseFromVehicle(veh, vcfg);
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    lidar.maxRange = cfg.max_range;
    const std::vector<int> azimuth_order = progressiveAzimuthOrder(cfg.k_max);

    std::vector<char> hit(world.obstacles.size(), 0);

    Layer2Result result;
    result.mode = cfg.mode;
    result.target_speed = cfg.target_speed;
    result.budget_ms = cfg.budget_ms;
    result.obstacles_total = static_cast<int>(world.obstacles.size());

    const double budget_ns = cfg.budget_ms * 1e6;
    double sum_k = 0.0, sum_in_view = 0.0, sum_scan_ns = 0.0;
    double sum_overrun_ns = 0.0;
    std::vector<double> scan_times_ns;
    scan_times_ns.reserve(cfg.max_frames);
    int stuck = 0;
    Vec3 prev(veh.x, veh.y, 0.0);

    int frame = 0;
    for (; frame < cfg.max_frames; ++frame)
    {
        int in_view = 0;
        for (const ActiveObstacle &o : world.obstacles)
            if (o.x >= veh.x - cfg.view_behind && o.x <= veh.x + cfg.max_range)
                ++in_view;

        lidar.azimuthSamples = cfg.k_max;
        lidar.pose = lidarPoseFromVehicle(veh, vcfg);

        const ScanResult scan = scanLayer2UntilDeadline(
            world, cfg, lidar, azimuth_order, budget_ns);
        const PlannerOutput plan = planFollowTheGap(
            scan,
            planner_scan_state,
            pcfg,
            lidar.maxRange,
            cfg.max_observation_age_frames);

        const VehicleState previous_vehicle = veh;
        updateVehicle(veh, vcfg, plan.speed_command, plan.steering_command, cfg.dt);

        bool first_collision_this_frame = false;
        for (size_t i = 0; i < world.obstacles.size(); ++i)
        {
            const ActiveObstacle &o = world.obstacles[i];
            if (!hit[i] &&
                sweptVehicleIntersectsCircle(
                    previous_vehicle,
                    veh,
                    vcfg,
                    o.center,
                    0.5 * o.size,
                    cfg.collision_motion_tolerance))
            {
                hit[i] = 1;
                first_collision_this_frame = result.collisions == 0;
                ++result.collisions;
            }
        }

        result.wall_contact = sweptVehicleTouchesCorridorWall(
            previous_vehicle,
            veh,
            vcfg,
            cfg.corridor_half_width,
            cfg.collision_motion_tolerance);

        const Vec3 pos(veh.x, veh.y, 0.0);
        result.distance += (pos - prev).norm();
        if (first_collision_this_frame)
            result.distance_to_first_collision = result.distance;
        prev = pos;

        sum_k += scan.rays_completed;
        sum_in_view += in_view;
        sum_scan_ns += scan.elapsed_ns;
        scan_times_ns.push_back(scan.elapsed_ns);
        if (result.frames == 0)
            result.min_k = scan.rays_completed;
        else
            result.min_k = std::min(result.min_k, scan.rays_completed);
        result.max_k = std::max(result.max_k, scan.rays_completed);
        sum_overrun_ns += scan.overrun_ns;
        result.max_overrun_ms =
            std::max(result.max_overrun_ms, scan.overrun_ns / 1e6);
        result.deadline_cutoffs += scan.deadline_reached ? 1 : 0;
        result.execution_overruns += scan.overrun_ns > 0.0 ? 1 : 0;
        ++result.frames;

        if (veh.speed < 0.05)
        {
            if (++stuck > 90)
                break;
        }
        else
            stuck = 0;

        if (result.wall_contact)
            break;

        if (veh.x >= cfg.course_length)
        {
            result.reached_end = true;
            break;
        }
        if (cfg.stop_on_crash && result.collisions > 0)
            break;
    }

    result.avg_k = result.frames > 0 ? sum_k / result.frames : 0.0;
    result.avg_in_view = result.frames > 0 ? sum_in_view / result.frames : 0.0;
    result.avg_scan_ms =
        result.frames > 0 ? (sum_scan_ns / result.frames) / 1e6 : 0.0;
    result.mean_overrun_ms =
        result.execution_overruns > 0
            ? (sum_overrun_ns / result.execution_overruns) / 1e6
            : 0.0;
    if (!scan_times_ns.empty())
    {
        std::sort(scan_times_ns.begin(), scan_times_ns.end());
        const size_t p95_index = static_cast<size_t>(
            std::ceil(0.95 * scan_times_ns.size()) - 1.0);
        result.p95_scan_ms = scan_times_ns[p95_index] / 1e6;
    }
    result.collisions_per_100m = result.distance > 1.0
                                     ? 100.0 * result.collisions / result.distance
                                     : 0.0;
    result.progress = std::clamp(
        veh.x / std::max(cfg.course_length, geom::Epsilon), 0.0, 1.0);
    result.collision_free_completion =
        result.reached_end && result.collisions == 0 && !result.wall_contact;
    return result;
}

/**
 * One-time correctness gate for the Layer 2 pillar course.
 *
 * The whole thesis rests on "all modes return bit-identical hits; only latency
 * differs." If SceneBVH disagreed with ground truth over the high-poly pillars
 * (a top-level traversal bug, a degenerate AABB, a max_distance edge case), the
 * trajectories would not be comparable. This verifies SceneBVH against a
 * triangle-soup reference
 * (`bruteForceIntersect`, no per-mesh BVH) — checking both the scene-level tree
 * and the per-mesh path end-to-end on the geometry that actually drives Layer 2.
 */
struct Layer2Verification
{
    int rays = 0;
    int mismatches = 0;
    double worst_t_err = 0.0;
    bool has_example = false;
    double ex_ref_t = -1.0, ex_bvh_t = -1.0;
    int ex_ref_obj = -1, ex_bvh_obj = -1;
    bool passed() const { return mismatches == 0; }
};

// Ground truth: linear scan over statics (analytic) + triangle-soup over obstacles.
inline Hit layer2ReferenceHit(const Layer2World &w, const Ray &ray, double max_distance)
{
    Hit closest;
    auto consider = [&](const Hit &h) {
        if (h.hit && h.t <= max_distance && (!closest.hit || h.t < closest.t))
            closest = h;
    };
    for (const auto &s : w.statics)
        consider(s->intersect(ray));
    for (const ActiveObstacle &o : w.obstacles)
    {
        auto mesh = std::dynamic_pointer_cast<TriangleMeshGeometry>(o.geometry);
        consider(mesh ? mesh->bruteForceIntersect(ray) : o.geometry->intersect(ray));
    }
    return closest;
}

inline Layer2Verification verifyLayer2Scene(
    const Layer2World &w,
    double max_range,
    double course_length,
    double corridor_half_width,
    unsigned int seed = 7,
    int pose_samples = 40,
    int rays_per_pose = 91)
{
    Layer2Verification v;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x_dist(0.0, course_length);
    std::uniform_real_distribution<double> y_dist(-corridor_half_width * 0.8, corridor_half_width * 0.8);
    std::uniform_real_distribution<double> h_dist(-geom::pi, geom::pi);
    const double tol = 1e-6;

    VehicleConfig vcfg;
    const int denom = std::max(1, rays_per_pose - 1);

    for (int p = 0; p < pose_samples; ++p)
    {
        VehicleState veh;
        veh.x = x_dist(rng);
        veh.y = y_dist(rng);
        veh.heading = h_dist(rng);
        const Eigen::Isometry3d pose = lidarPoseFromVehicle(veh, vcfg);

        for (int r = 0; r < rays_per_pose; ++r)
        {
            const double az = -geom::pi / 2.0 + geom::pi * r / denom;
            const Vec3 local(std::cos(az), std::sin(az), 0.0);
            Ray ray;
            ray.ori = pose.translation();
            ray.dir = pose.rotation() * local;

            const Hit ref = layer2ReferenceHit(w, ray, max_range);
            const Hit got = w.bvh.intersect(ray, max_range);
            ++v.rays;

            const bool rv = ref.hit;
            const bool gv = got.hit;
            const bool mismatch =
                (rv != gv) || (rv && (ref.objId != got.objId || std::abs(ref.t - got.t) > tol));
            if (mismatch)
            {
                ++v.mismatches;
                if (rv && gv)
                    v.worst_t_err = std::max(v.worst_t_err, std::abs(ref.t - got.t));
                if (!v.has_example)
                {
                    v.has_example = true;
                    v.ex_ref_t = rv ? ref.t : -1.0;
                    v.ex_bvh_t = gv ? got.t : -1.0;
                    v.ex_ref_obj = rv ? ref.objId : -1;
                    v.ex_bvh_obj = gv ? got.objId : -1;
                }
            }
        }
    }
    return v;
}

// Convenience: build the course for a config and verify it.
inline Layer2Verification verifyLayer2Course(const Layer2Config &cfg,
                                             int pose_samples = 40,
                                             int rays_per_pose = 91)
{
    Layer2World world = makeLayer2World(cfg);
    return verifyLayer2Scene(world, cfg.max_range, cfg.course_length,
                             cfg.corridor_half_width, cfg.seed + 7u,
                             pose_samples, rays_per_pose);
}
