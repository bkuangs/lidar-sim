#include "layer2.hpp"        // Layer 2 world/loop + scene-vs-soup verification
#include "accelerated_scene.hpp" // XBucketScene for Layer 1 equivalence
#include "scene.hpp"         // linear-scan reference
#include "meshes.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

bool near(double a, double b, double eps = 1e-9)
{
    return std::abs(a - b) <= eps;
}

// --- shared building blocks (still exercised by both layers) -----------------

void testVehicleDynamics()
{
    VehicleConfig config;
    VehicleState vehicle;
    updateVehicle(vehicle, config, 1.0, 0.0, 1.0);
    assert(vehicle.x > 0.0);
    assert(near(vehicle.y, 0.0));
    assert(near(vehicle.heading, 0.0));

    updateVehicle(vehicle, config, 2.0, 20.0 * geom::deg, 0.2);
    assert(vehicle.steering > 0.0);
    assert(vehicle.heading > 0.0);

    updateVehicle(vehicle, config, 100.0, 100.0 * geom::deg, 1.0);
    assert(vehicle.speed <= config.max_speed + 1e-9);
    assert(vehicle.steering <= config.max_steering + 1e-9);
}

void testFollowTheGap()
{
    PlannerConfig config;
    std::vector<double> ranges(config.bins, 20.0);

    PlannerOutput clear = planFromRanges(ranges, config);
    assert(!clear.blocked);
    assert(std::abs(clear.target_angle) < 2.0 * geom::deg);
    assert(clear.speed_command > config.min_speed);

    ranges.assign(config.bins, 20.0);
    for (int i = angleToBin(config, -10.0 * geom::deg); i <= angleToBin(config, 10.0 * geom::deg); ++i)
        ranges[i] = 0.6;
    PlannerOutput center_blocked = planFromRanges(ranges, config);
    assert(!center_blocked.blocked);
    assert(std::abs(center_blocked.target_angle) > 10.0 * geom::deg);

    ranges.assign(config.bins, 0.5);
    PlannerOutput blocked = planFromRanges(ranges, config);
    assert(blocked.blocked);
    assert(near(blocked.speed_command, 0.0));
}

// --- Layer 1 claim: accelerators return identical hits to the linear scan ----

void testLayer1Equivalence()
{
    const double max_range = 50.0;

    std::vector<std::shared_ptr<Geometry>> statics = {
        std::make_shared<PlaneGeometry>(Vec3(0, 0, 1), Vec3(0, 0, 0), 0),
        std::make_shared<PlaneGeometry>(Vec3(0, 1, 0), Vec3(0, -3, 0), 1),
        std::make_shared<PlaneGeometry>(Vec3(0, -1, 0), Vec3(0, 3, 0), 2),
    };

    std::mt19937 rng(2024);
    std::uniform_real_distribution<double> y_dist(-2.0, 2.0), size_dist(0.3, 0.9);
    std::vector<ActiveObstacle> obstacles;
    for (int i = 0; i < 40; ++i)
    {
        const double x = 3.0 + 1.1 * i;
        const double y = y_dist(rng), size = size_dist(rng), half = size * 0.5;
        std::shared_ptr<Geometry> g;
        switch (i % 3)
        {
        case 0: g = makeCube(Vec3(x, y, half), size, 10 + i); break;
        case 1: g = std::make_shared<SphereGeometry>(Vec3(x, y, half), half, 10 + i); break;
        default: g = makePillar(x, y, half, 3.0, 12, 4, 10 + i); break;
        }
        ActiveObstacle o;
        o.x = x; o.center = Vec3(x, y, half); o.size = size; o.object_id = 10 + i;
        o.bounds = g->bounds(); o.geometry = g;
        obstacles.push_back(o);
    }

    Scene linear; // reference
    linear.objects = statics;
    for (const auto &o : obstacles)
        linear.objects.push_back(o.geometry);

    XBucketScene buckets;
    buckets.rebuild(statics, obstacles);
    SceneBVH bvh;
    bvh.rebuild(statics, obstacles);

    auto valid = [&](const Hit &h) { return h.hit && h.t <= max_range; };
    auto mismatch = [&](const Hit &a, const Hit &b) {
        if (valid(a) != valid(b)) return true;
        return valid(a) && (a.objId != b.objId || std::abs(a.t - b.t) > 1e-6);
    };

    int bucket_mismatches = 0, bvh_mismatches = 0, rays = 0;
    for (double ox : {0.0, 12.0, 25.0, 40.0})
    {
        Ray ray;
        ray.ori = Vec3(ox, 0.0, 1.0);
        for (int a = 0; a < 180; ++a)
        {
            const double az = -geom::pi / 2 + geom::pi * a / 179.0;
            for (double el : {-10.0 * geom::deg, 0.0, 10.0 * geom::deg})
            {
                ray.dir = Vec3(std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el));
                const Hit ref = linear.intersect(ray);
                bucket_mismatches += mismatch(ref, buckets.intersect(ray, max_range)) ? 1 : 0;
                bvh_mismatches += mismatch(ref, bvh.intersect(ray, max_range)) ? 1 : 0;
                ++rays;
            }
        }
    }
    assert(bucket_mismatches == 0);
    assert(bvh_mismatches == 0);
    assert(rays > 0);
}

// --- Layer 2 claim: the scene BVH the sim actually traces == ground truth ----

void testLayer2SceneVerification()
{
    Layer2Config cfg;
    cfg.seed = 1234;
    const Layer2Verification v = verifyLayer2Course(cfg);
    assert(v.rays > 0);
    assert(v.passed());
}

void testLayer2Determinism()
{
    Layer2Config cfg;
    cfg.seed = 777;
    const Layer2World a = makeLayer2World(cfg);
    const Layer2World b = makeLayer2World(cfg);
    assert(a.obstacles.size() == b.obstacles.size());
    assert(a.tris_per_obstacle == b.tris_per_obstacle);
    for (size_t i = 0; i < a.obstacles.size(); ++i)
    {
        assert(near(a.obstacles[i].x, b.obstacles[i].x));
        assert((a.obstacles[i].center - b.obstacles[i].center).norm() < 1e-12);
        assert(a.obstacles[i].object_id == b.obstacles[i].object_id);
    }
}

int main()
{
    testVehicleDynamics();
    testFollowTheGap();
    testLayer1Equivalence();
    testLayer2SceneVerification();
    testLayer2Determinism();

    std::cout << "All lidar_3d tests passed\n";
    return 0;
}
