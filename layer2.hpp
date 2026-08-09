#pragma once
#include "meshes.hpp"
#include "scene_bvh.hpp"
#include "hallway.hpp"
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

// Cost per ray (ns), baked from poly_probe.cpp so Layer 2 is machine-independent.
struct CostModel
{
    double brute_ns_per_tri = 4.40;    // 211416 ns @ 100 obstacles * 480 tris
    double mesh_ns_per_obstacle = 9.3; // per-mesh BVH, scene-level brute
    double bvh_base_ns = 0.0;
    double bvh_log_ns = 12.0; // ~O(log N) traversal
    int tris_per_obstacle = 480;

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
        // obstacles currently in the brute working set (window ahead + a little behind)
        int in_view = 0;
        for (const ActiveObstacle &o : world.obstacles)
            if (o.x >= veh.x - cfg.view_behind && o.x <= veh.x + cfg.max_range)
                ++in_view;

        const double cost_ray = cost.costRayNs(cfg.mode, in_view);
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
