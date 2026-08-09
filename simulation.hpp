#pragma once
#include "accelerated_scene.hpp"
#include "scene_bvh.hpp"
#include "planner.hpp"
#include "vehicle.hpp"
#include <chrono>
#include <vector>

struct SimulationMetrics
{
    double scan_ms = 0.0;
    double planner_ms = 0.0;
    double loop_ms = 0.0;
    int collision_count = 0;
    int verification_mismatches = 0;
    bool accelerator_verified = true;
};

struct SimulationState
{
    HallwayWorld world;
    VehicleConfig vehicle_config;
    VehicleState vehicle;
    PlannerConfig planner_config;
    Lidar lidar;
    Scene scene;
    std::vector<std::shared_ptr<Geometry>> static_objects;
    XBucketScene bucket_scene;
    SceneBVH bvh_scene;
    ScanResult scan;
    PlannerOutput plan;
    SimulationMetrics metrics;
    QueryMode query_mode = QueryMode::BruteForce;
    bool collided = false;
    int scene_revision = -1;
    int frame = 0;
    std::vector<Vec3> path;
};

inline double elapsedMs(
    const std::chrono::steady_clock::time_point &start,
    const std::chrono::steady_clock::time_point &end)
{
    return std::chrono::duration<double, std::milli>(end - start).count();
}

inline void rebuildSceneQueries(SimulationState &sim)
{
    sim.static_objects = makeHallwayStaticObjects(sim.world);
    sim.scene = makeHallwayScene(sim.world);
    sim.bucket_scene.rebuild(sim.static_objects, sim.world.obstacles);
    sim.bvh_scene.rebuild(sim.static_objects, sim.world.obstacles);
    sim.scene_revision = sim.world.revision;
}

inline SimulationState makeSimulation(unsigned int seed = 42)
{
    SimulationState sim;
    sim.world = makeHallwayWorld(seed);
    sim.vehicle.speed = 1.0;
    sim.lidar.pose = lidarPoseFromVehicle(sim.vehicle, sim.vehicle_config);
    updateObstacleWindow(sim.world, sim.vehicle.x);
    rebuildSceneQueries(sim);
    sim.scan = scanScene(sim.scene, sim.lidar);
    sim.plan = planFollowTheGap(sim.scan, sim.planner_config, sim.lidar.maxRange);
    sim.path.push_back(sim.lidar.pose.translation());
    return sim;
}

inline Ray lidarRay(const Lidar &lidar, double azimuth, double elevation)
{
    const Vec3 local_dir(
        std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));

    Ray ray;
    ray.ori = lidar.pose.translation();
    ray.dir = lidar.pose.rotation() * local_dir.normalized();
    return ray;
}

inline int verifyAcceleratedQueries(const SimulationState &sim)
{
    int mismatches = 0;
    constexpr double tolerance = 1e-6;

    auto compare = [&](const Hit &brute, const Hit &accel) {
        const bool brute_valid = brute.hit && brute.t <= sim.lidar.maxRange;
        const bool accel_valid = accel.hit && accel.t <= sim.lidar.maxRange;
        if (brute_valid != accel_valid)
            return 1;
        if (brute_valid && (brute.objId != accel.objId || std::abs(brute.t - accel.t) > tolerance))
            return 1;
        return 0;
    };

    for (double elevation : {-15.0 * geom::deg, 0.0, 15.0 * geom::deg})
    {
        for (int i = 0; i < 24; ++i)
        {
            const double azimuth = (2.0 * geom::pi * i) / 24.0;
            const Ray ray = lidarRay(sim.lidar, azimuth, elevation);
            const Hit brute = sim.scene.intersect(ray);
            mismatches += compare(brute, sim.bucket_scene.intersect(ray, sim.lidar.maxRange));
            mismatches += compare(brute, sim.bvh_scene.intersect(ray, sim.lidar.maxRange));
        }
    }

    return mismatches;
}

inline void stepSimulation(SimulationState &sim, double dt)
{
    const auto loop_start = std::chrono::steady_clock::now();

    updateObstacleWindow(sim.world, sim.vehicle.x);
    if (sim.scene_revision != sim.world.revision)
        rebuildSceneQueries(sim);

    sim.lidar.pose = lidarPoseFromVehicle(sim.vehicle, sim.vehicle_config);

    const auto scan_start = std::chrono::steady_clock::now();
    switch (sim.query_mode)
    {
    case QueryMode::BruteForce:
        sim.scan = scanScene(sim.scene, sim.lidar);
        break;
    case QueryMode::XBuckets:
        sim.scan = scanWithIntersector(
            sim.lidar,
            [&sim](const Ray &ray)
            {
                return sim.bucket_scene.intersect(ray, sim.lidar.maxRange);
            });
        break;
    case QueryMode::SceneBVH:
        sim.scan = scanWithIntersector(
            sim.lidar,
            [&sim](const Ray &ray)
            {
                return sim.bvh_scene.intersect(ray, sim.lidar.maxRange);
            });
        break;
    }
    const auto scan_end = std::chrono::steady_clock::now();
    sim.metrics.scan_ms = elapsedMs(scan_start, scan_end);

    if (sim.query_mode != QueryMode::BruteForce && sim.frame % 60 == 0)
    {
        sim.metrics.verification_mismatches = verifyAcceleratedQueries(sim);
        sim.metrics.accelerator_verified = sim.metrics.verification_mismatches == 0;
    }

    const auto planner_start = std::chrono::steady_clock::now();
    sim.plan = planFollowTheGap(sim.scan, sim.planner_config, sim.lidar.maxRange);
    const auto planner_end = std::chrono::steady_clock::now();
    sim.metrics.planner_ms = elapsedMs(planner_start, planner_end);

    updateVehicle(
        sim.vehicle,
        sim.vehicle_config,
        sim.plan.speed_command,
        sim.plan.steering_command,
        dt);
    sim.lidar.pose = lidarPoseFromVehicle(sim.vehicle, sim.vehicle_config);

    const bool now_collided = checkCollision(sim.world, sim.vehicle, sim.vehicle_config);
    if (now_collided && !sim.collided)
        ++sim.metrics.collision_count;
    sim.collided = now_collided;

    if (sim.path.empty() || (sim.lidar.pose.translation() - sim.path.back()).norm() > 0.15)
        sim.path.push_back(sim.lidar.pose.translation());

    const auto loop_end = std::chrono::steady_clock::now();
    sim.metrics.loop_ms = elapsedMs(loop_start, loop_end);
    ++sim.frame;
}
