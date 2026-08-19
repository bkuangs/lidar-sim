#include "layer2.hpp"        // Layer 2 world/loop + scene-vs-soup verification
#include "metrics.hpp"
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
    clear_scan.points.push_back(ScanPoint{0.0, 0.0, 20.0, false});

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
    hit_scan.points.push_back(ScanPoint{0.0, 0.0, 0.5, true});
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

    Lidar lidar;
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.azimuthSamples = 5;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;

    const ScanResult scan = scanProgressiveWithIntersector(
        lidar, order, 361, [](const Ray &) { return Hit{}; });
    assert(scan.points.size() == 5);
    assert(near(scan.points[0].azimuth, 0.0));
    assert(near(scan.points[1].azimuth, -90.0 * geom::deg));
    assert(near(scan.points[2].azimuth, 90.0 * geom::deg));
    assert(near(scan.points[3].azimuth, -45.0 * geom::deg));
    assert(near(scan.points[4].azimuth, 45.0 * geom::deg));
}

void testDeadlineScanner()
{
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    Lidar lidar;
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.azimuthSamples = 361;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;

    const std::vector<int> order = progressiveAzimuthOrder(361);
    const std::vector<long long> ticks = {0, 3, 7, 12};
    size_t tick = 0;
    int intersections = 0;
    const ScanResult scan = scanProgressiveUntilDeadline(
        lidar,
        order,
        361,
        10.0,
        [&](const Ray &) {
            ++intersections;
            return Hit{};
        },
        [&] {
            return TimePoint(std::chrono::nanoseconds(ticks.at(tick++)));
        });

    assert(scan.rays_requested == 361);
    assert(scan.rays_attempted == 3);
    assert(scan.rays_completed == 2);
    assert(scan.points.size() == 2);
    assert(intersections == 3);
    assert(scan.deadline_reached);
    assert(near(scan.elapsed_ns, 12.0));
    assert(near(scan.overrun_ns, 2.0));
    assert(near(scan.points[0].azimuth, 0.0));
    assert(near(scan.points[1].azimuth, -90.0 * geom::deg));

    const std::vector<long long> boundary_ticks = {0, 10};
    tick = 0;
    const ScanResult boundary = scanProgressiveUntilDeadline(
        lidar,
        order,
        361,
        10.0,
        [](const Ray &) { return Hit{}; },
        [&] {
            return TimePoint(
                std::chrono::nanoseconds(boundary_ticks.at(tick++)));
        });
    assert(boundary.rays_attempted == 1);
    assert(boundary.rays_completed == 1);
    assert(boundary.deadline_reached);
    assert(near(boundary.overrun_ns, 0.0));

    lidar.azimuthSamples = 3;
    const std::vector<long long> capped_ticks = {0, 1, 2, 3};
    tick = 0;
    const ScanResult capped = scanProgressiveUntilDeadline(
        lidar,
        order,
        361,
        10.0,
        [](const Ray &) { return Hit{}; },
        [&] {
            return TimePoint(
                std::chrono::nanoseconds(capped_ticks.at(tick++)));
        });
    assert(capped.rays_attempted == 3);
    assert(capped.rays_completed == 3);
    assert(!capped.deadline_reached);
    assert(near(capped.elapsed_ns, 3.0));
}

void testPairedAllSeedMetrics()
{
    const std::vector<char> outcomes = {1, 0, 1, 0};
    const std::vector<char> reference = {0, 0, 1, 0};
    const PairedBinaryBootstrap summary =
        pairedBinaryBootstrap(outcomes, reference, 2000, 42);

    assert(summary.samples == 4);
    assert(near(summary.rate_pct, 50.0));
    assert(near(summary.delta_pct, 25.0));
    assert(summary.rate_low_pct <= summary.rate_pct);
    assert(summary.rate_high_pct >= summary.rate_pct);
    assert(summary.delta_low_pct <= summary.delta_pct);
    assert(summary.delta_high_pct >= summary.delta_pct);

    const std::vector<char> failed = {0, 0, 0, 0};
    const PairedBinaryBootstrap failed_summary =
        pairedBinaryBootstrap(failed, reference, 2000, 7);
    assert(failed_summary.samples == 4);
    assert(near(failed_summary.rate_pct, 0.0));
    assert(near(failed_summary.rate_low_pct, 0.0));
    assert(near(failed_summary.rate_high_pct, 0.0));
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

// --- Layer 2 claim: every timed tracer returns the same scan ------------------

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

void testLayer2ModeScans()
{
    Layer2Config cfg;
    cfg.seed = 2468;
    const Layer2World world = makeLayer2World(cfg);

    VehicleConfig vehicle_config;
    VehicleState vehicle;
    vehicle.x = 20.0;
    vehicle.y = 0.5;
    vehicle.heading = 5.0 * geom::deg;

    Lidar lidar;
    lidar.pose = lidarPoseFromVehicle(vehicle, vehicle_config);
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    lidar.azimuthSamples = 31;
    lidar.maxRange = cfg.max_range;

    cfg.mode = Layer2Mode::TrueBrute;
    const ScanResult brute = scanLayer2(world, cfg, lidar);
    cfg.mode = Layer2Mode::MeshBVH;
    const ScanResult mesh = scanLayer2(world, cfg, lidar);
    cfg.mode = Layer2Mode::SceneBVH;
    const ScanResult scene = scanLayer2(world, cfg, lidar);

    assert(brute.points.size() == mesh.points.size());
    assert(brute.points.size() == scene.points.size());
    for (size_t i = 0; i < brute.points.size(); ++i)
    {
        assert(near(brute.points[i].azimuth, mesh.points[i].azimuth));
        assert(near(brute.points[i].azimuth, scene.points[i].azimuth));
        assert(near(brute.points[i].range, mesh.points[i].range, 1e-6));
        assert(near(brute.points[i].range, scene.points[i].range, 1e-6));
    }
}

void testLayer2Budget()
{
    assert(affordableRayCount(499.0, 100.0, 361) == 4);
    assert(affordableRayCount(99.0, 100.0, 361) == 0);
    assert(affordableRayCount(100000.0, 100.0, 361) == 361);

    for (Layer2Mode mode : {Layer2Mode::TrueBrute, Layer2Mode::SceneBVH})
    {
        Layer2Config cfg;
        cfg.mode = mode;
        cfg.budget_ms = 0.0;
        cfg.max_frames = 1;
        const Layer2Result result = runLayer2(cfg);
        assert(result.mode == mode);
        assert(result.frames == 1);
        assert(near(result.avg_k, 0.0));
        assert(result.min_k == 0);
        assert(result.max_k == 0);
        assert(near(result.distance, 0.0));
        assert(near(result.progress, 0.0));
        assert(near(result.distance_to_first_collision, -1.0));
        assert(!result.collision_free_completion);
    }
}

int main()
{
    testVehicleDynamics();
    testSweptVehicleCollisions();
    testFollowTheGap();
    testPartialScanState();
    testScanRecordsClearRays();
    testProgressiveRayOrder();
    testDeadlineScanner();
    testPairedAllSeedMetrics();
    testLayer1Equivalence();
    testLayer2SceneVerification();
    testLayer2Determinism();
    testLayer2ModeScans();
    testLayer2Budget();

    std::cout << "All lidar_3d tests passed\n";
    return 0;
}
