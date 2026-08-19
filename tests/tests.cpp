#include "accelerated_scene.hpp" // XBucketScene for Layer 1 equivalence
#include "planner.hpp"
#include "scene.hpp"         // linear-scan reference
#include "scene_bvh.hpp"
#include "meshes.hpp"
#include "vehicle.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>

bool near(double a, double b, double eps = 1e-9)
{
    return std::abs(a - b) <= eps;
}

// --- shared building blocks ---------------------------------------------------

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

void testSweptVehicleCollisions()
{
    VehicleConfig config;
    config.length = 2.0;
    config.width = 0.5;

    VehicleState vehicle;
    assert(vehicleIntersectsCircle(
        vehicle, config, Vec3(1.5, 0.0, 0.0), 0.5));
    assert(!vehicleIntersectsCircle(
        vehicle, config, Vec3(1.51, 0.0, 0.0), 0.5));

    vehicle.heading = 45.0 * geom::deg;
    assert(!vehicleIntersectsCircle(
        vehicle, config, Vec3(0.8, -0.8, 0.0), 0.05));

    VehicleState from;
    VehicleState to;
    to.x = 3.0;
    assert(!vehicleIntersectsCircle(
        from, config, Vec3(1.5, 0.0, 0.0), 0.1));
    assert(!vehicleIntersectsCircle(
        to, config, Vec3(1.5, 0.0, 0.0), 0.1));
    assert(sweptVehicleIntersectsCircle(
        from, to, config, Vec3(1.5, 0.0, 0.0), 0.1, 0.01));

    from = VehicleState{};
    to = VehicleState{};
    to.heading = 90.0 * geom::deg;
    assert(!vehicleIntersectsCircle(
        from, config, Vec3(0.7, 0.7, 0.0), 0.05));
    assert(!vehicleIntersectsCircle(
        to, config, Vec3(0.7, 0.7, 0.0), 0.05));
    assert(sweptVehicleIntersectsCircle(
        from, to, config, Vec3(0.7, 0.7, 0.0), 0.05, 0.01));

    from.y = 0.3;
    to.y = 0.3;
    assert(!vehicleTouchesCorridorWall(from, config, 1.315));
    assert(!vehicleTouchesCorridorWall(to, config, 1.315));
    assert(sweptVehicleTouchesCorridorWall(
        from, to, config, 1.315, 0.01));
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

void testPartialScanState()
{
    PlannerConfig config;
    PlannerScanState state(config.bins);
    const int center = angleToBin(config, 0.0);

    ScanResult clear_scan;
    clear_scan.rays_requested = 1;
    clear_scan.rays_completed = 1;
    clear_scan.points.push_back(ScanPoint{0.0, 0.0, 20.0, -1, false});

    PlannerOutput output =
        planFollowTheGap(clear_scan, state, config, 20.0, 3);
    assert(!output.blocked);
    assert(state.completed_rays == 1);
    assert(state.bins[center].state == ScanBinState::Clear);
    assert(state.bins[center].age_frames == 0);

    const ScanResult empty_scan;
    for (int age = 1; age <= 3; ++age)
    {
        output = planFollowTheGap(empty_scan, state, config, 20.0, 3);
        assert(!output.blocked);
        assert(state.bins[center].state == ScanBinState::Clear);
        assert(state.bins[center].age_frames == age);
    }

    output = planFollowTheGap(empty_scan, state, config, 20.0, 3);
    assert(output.blocked);
    assert(state.completed_rays == 0);
    assert(state.bins[center].state == ScanBinState::Unobserved);

    ScanResult hit_scan;
    hit_scan.rays_requested = 1;
    hit_scan.rays_completed = 1;
    hit_scan.points.push_back(ScanPoint{0.0, 0.0, 0.5, 7, true});
    output = planFollowTheGap(hit_scan, state, config, 20.0, 3);
    assert(output.blocked);
    assert(state.bins[center].state == ScanBinState::Hit);
    assert(near(state.bins[center].range, 0.5));
}

void testScanRecordsClearRays()
{
    Lidar lidar;
    lidar.minAzimuth = -10.0 * geom::deg;
    lidar.maxAzimuth = 10.0 * geom::deg;
    lidar.azimuthSamples = 3;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    lidar.maxRange = 25.0;

    const ScanResult scan = scanWithIntersector(lidar, [](const Ray &) {
        return Hit{};
    });
    assert(scan.rays_requested == 3);
    assert(scan.rays_completed == 3);
    assert(scan.points.size() == 3);
    for (const ScanPoint &point : scan.points)
    {
        assert(!point.hit);
        assert(near(point.range, lidar.maxRange));
        assert(point.object_id == -1);
    }
}

int maxProgressiveGap(const std::vector<int> &order, int prefix)
{
    std::vector<int> selected(order.begin(), order.begin() + prefix);
    std::sort(selected.begin(), selected.end());
    int max_gap = 0;
    for (size_t i = 1; i < selected.size(); ++i)
        max_gap = std::max(max_gap, selected[i] - selected[i - 1]);
    return max_gap;
}

void testProgressiveRayOrder()
{
    const std::vector<int> order = progressiveAzimuthOrder(361);
    assert(order.size() == 361);
    assert(order == progressiveAzimuthOrder(361));
    assert(order[0] == 180);
    assert(order[1] == 0);
    assert(order[2] == 360);

    std::vector<int> sorted = order;
    std::sort(sorted.begin(), sorted.end());
    for (int i = 0; i < 361; ++i)
        assert(sorted[i] == i);

    for (int prefix : {3, 5, 9, 17})
    {
        const int ideal_gap =
            static_cast<int>(std::ceil(360.0 / (prefix - 1)));
        assert(maxProgressiveGap(order, prefix) <= ideal_gap);
    }
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

int main()
{
    testVehicleDynamics();
    testSweptVehicleCollisions();
    testFollowTheGap();
    testPartialScanState();
    testScanRecordsClearRays();
    testProgressiveRayOrder();
    testLayer1Equivalence();

    std::cout << "All lidar_3d tests passed\n";
    return 0;
}
