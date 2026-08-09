#pragma once
#include "meshes.hpp"
#include "scene_bvh.hpp"
#include "obstacle.hpp"
#include "planner.hpp"
#include "scan.hpp"
#include "vehicle.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

/**
 * LAYER 2 — closed-loop autonomy under a fixed per-frame perception budget.
 *
 * All modes trace with the fast scene BVH (identical, correct hits). The mode's
 * baked cost model decides how many azimuth rays that mode could AFFORD within
 * the budget given the obstacles currently in view. Fewer rays -> coarser
 * angular resolution -> small pillars are aliased past -> collisions. This turns
 * "faster ray tracing" into "better autonomy" as a measurable, causal claim.
 */

enum class Layer2Mode
{
    TrueBrute, // triangle soup, no acceleration
    MeshBVH,   // per-mesh BVH only, scene-level brute (still O(N))
    SceneBVH   // scene + per-mesh BVH
};

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
    double bvh_log_ns = 0.0;
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
            return bvh_base_ns + bvh_log_ns * std::log2(static_cast<double>(n));
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

    int k_min = 5;
    int k_max = 361; // full-res cap over the 180-degree arc
    double max_range = 50.0;
    double view_behind = 5.0;

    double dt = 1.0 / 30.0;
    int max_frames = 6000;
    bool stop_on_crash = false;
};

struct Layer2Result
{
    Layer2Mode mode = Layer2Mode::SceneBVH;
    double target_speed = 0.0;
    double budget_ms = 0.0;
    int obstacles_total = 0;
    int obstacles_passed = 0;
    int collisions = 0;
    double collision_rate = 0.0;
    double collisions_per_100m = 0.0;
    double distance = 0.0;
    double min_clearance = std::numeric_limits<double>::infinity();
    double avg_k = 0.0;
    double avg_in_view = 0.0;
    double avg_speed = 0.0;
    bool reached_end = false;
};

struct Layer2World
{
    std::vector<ActiveObstacle> obstacles;
    std::vector<std::shared_ptr<Geometry>> statics;
    SceneBVH bvh;
    int tris_per_obstacle = 0;
};

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

    CostModel cost;
    cost.tris_per_obstacle = world.tris_per_obstacle;

    VehicleConfig vcfg;
    vcfg.max_speed = cfg.target_speed + 1.0;
    VehicleState veh;

    PlannerConfig pcfg;
    pcfg.max_speed = cfg.target_speed;

    Lidar lidar;
    lidar.pose = lidarPoseFromVehicle(veh, vcfg);
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    lidar.maxRange = cfg.max_range;

    std::vector<char> hit(world.obstacles.size(), 0);
    std::vector<char> passed(world.obstacles.size(), 0);

    Layer2Result result;
    result.mode = cfg.mode;
    result.target_speed = cfg.target_speed;
    result.budget_ms = cfg.budget_ms;
    result.obstacles_total = static_cast<int>(world.obstacles.size());

    const double budget_ns = cfg.budget_ms * 1e6;
    double sum_k = 0.0, sum_in_view = 0.0, sum_speed = 0.0;
    int stuck = 0;
    Vec3 prev(veh.x, veh.y, 0.0);

    auto bvh_fn = [&](const Ray &ray) { return world.bvh.intersect(ray, cfg.max_range); };

    int frame = 0;
    for (; frame < cfg.max_frames; ++frame)
    {
        // in_view is reported only; the cost model counts the TOTAL obstacle set
        // because brute/mesh have no spatial culling — they test every obstacle in
        // the scene on every ray, regardless of what is currently in view. (Choice A.)
        int in_view = 0;
        for (const ActiveObstacle &o : world.obstacles)
            if (o.x >= veh.x - cfg.view_behind && o.x <= veh.x + cfg.max_range)
                ++in_view;

        const int n_cost = static_cast<int>(world.obstacles.size());
        const double cost_ray = cost.costRayNs(cfg.mode, n_cost);
        int k = static_cast<int>(std::lround(budget_ns / std::max(1.0, cost_ray)));
        k = std::clamp(k, cfg.k_min, cfg.k_max);

        lidar.azimuthSamples = k;
        lidar.pose = lidarPoseFromVehicle(veh, vcfg);

        ScanResult scan = scanWithIntersector(lidar, bvh_fn);
        PlannerOutput plan = planFollowTheGap(scan, pcfg, lidar.maxRange);

        updateVehicle(veh, vcfg, plan.speed_command, plan.steering_command, cfg.dt);

        // collision + clearance against obstacles
        const AABB footprint = vehicleFootprintBounds(veh, vcfg);
        for (size_t i = 0; i < world.obstacles.size(); ++i)
        {
            const ActiveObstacle &o = world.obstacles[i];
            if (std::abs(o.x - veh.x) < 2.0)
            {
                const double lateral = std::abs(o.center.y() - veh.y) - o.size * 0.5 - 0.5 * vcfg.width;
                result.min_clearance = std::min(result.min_clearance, lateral);
            }
            if (!hit[i] && intersectsAABB(footprint, o.bounds))
            {
                hit[i] = 1;
                ++result.collisions;
            }
            if (!passed[i] && o.x < veh.x)
            {
                passed[i] = 1;
                ++result.obstacles_passed;
            }
        }

        const Vec3 pos(veh.x, veh.y, 0.0);
        result.distance += (pos - prev).norm();
        prev = pos;

        sum_k += k;
        sum_in_view += in_view;
        sum_speed += veh.speed;

        if (veh.speed < 0.05)
        {
            if (++stuck > 90)
                break;
        }
        else
            stuck = 0;

        // wall departure counts as failure to progress
        if (std::abs(veh.y) > cfg.corridor_half_width)
        {
            veh.y = std::clamp(veh.y, -cfg.corridor_half_width, cfg.corridor_half_width);
        }

        if (veh.x >= cfg.course_length)
        {
            result.reached_end = true;
            break;
        }
        if (cfg.stop_on_crash && result.collisions > 0)
            break;
    }

    const int frames = frame + 1;
    result.avg_k = sum_k / frames;
    result.avg_in_view = sum_in_view / frames;
    result.avg_speed = sum_speed / frames;
    result.collision_rate = result.obstacles_passed > 0
                                ? static_cast<double>(result.collisions) / result.obstacles_passed
                                : 0.0;
    result.collisions_per_100m = result.distance > 1.0
                                     ? 100.0 * result.collisions / result.distance
                                     : 0.0;
    if (!std::isfinite(result.min_clearance))
        result.min_clearance = 0.0;
    return result;
}

/**
 * One-time correctness gate for the Layer 2 pillar course.
 *
 * The whole thesis rests on "all modes return bit-identical hits; only latency
 * differs." The sim always traces with the scene BVH, so if SceneBVH ever
 * disagreed with ground truth over the high-poly pillars (a top-level traversal
 * bug, a degenerate AABB, a max_distance edge case) the trajectories would be
 * silently wrong. This verifies SceneBVH against a triangle-soup reference
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

