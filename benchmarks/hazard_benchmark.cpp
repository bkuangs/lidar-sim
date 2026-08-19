#include "hazard_trial.hpp"
#include "hazard_timing_scene.hpp"
#include "meshes.hpp"
#include "scan.hpp"
#include "scene_bvh.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr int hazardSlices = 28;
constexpr int hazardStacks = 8;
constexpr int timingBackgroundCount = 100;
constexpr int timingWarmupScans = 20;
constexpr int timingMeasuredScans = 200;
constexpr double rangeTolerance = 1e-6;

enum class TraceMode
{
    TrueBrute,
    MeshBvh,
    SceneBvh
};

const char *modeName(TraceMode mode)
{
    switch (mode)
    {
    case TraceMode::TrueBrute:
        return "true-brute";
    case TraceMode::MeshBvh:
        return "mesh-bvh";
    case TraceMode::SceneBvh:
        return "scene-bvh";
    }
    throw std::runtime_error("unknown trace mode");
}

struct BenchmarkWorld
{
    std::vector<std::shared_ptr<TriangleMeshGeometry>> meshes;
    std::vector<ActiveObstacle> obstacles;
    SceneBVH bvh;
    size_t triangles_per_mesh = 0;
};

void addPillar(
    BenchmarkWorld &world,
    double x,
    double y,
    double radius,
    double height,
    int slices,
    int stacks,
    int object_id,
    unsigned subdivision_passes)
{
    const TriangleMeshData base =
        makePillarMeshData(x, y, radius, height, slices, stacks);
    const TriangleMeshData data =
        subdivideTriangleFaces(base, subdivision_passes);
    auto mesh = std::make_shared<TriangleMeshGeometry>(
        data.vertices, data.triangles, object_id);
    if (world.triangles_per_mesh == 0)
        world.triangles_per_mesh = data.triangles.size();
    else if (world.triangles_per_mesh != data.triangles.size())
        throw std::runtime_error("world meshes must have one triangle count");

    ActiveObstacle obstacle;
    obstacle.x = x;
    obstacle.center = Vec3(x, y, 0.5 * height);
    obstacle.size = 2.0 * radius;
    obstacle.object_id = object_id;
    obstacle.bounds = mesh->bounds();
    obstacle.geometry = mesh;
    world.meshes.push_back(std::move(mesh));
    world.obstacles.push_back(std::move(obstacle));
}

void finishWorld(BenchmarkWorld &world)
{
    world.bvh.rebuild({}, world.obstacles);
}

BenchmarkWorld makeSafetyWorld(const HazardScenario &scenario)
{
    BenchmarkWorld world;
    const Vec3 center = hazardCenter(scenario, VehicleConfig{});
    addPillar(
        world,
        center.x(),
        center.y(),
        0.5 * scenario.hazard.diameter,
        scenario.hazard.height,
        hazardSlices,
        hazardStacks,
        scenario.hazard.object_id,
        0);
    finishWorld(world);

    const AABB bounds = world.meshes.front()->bounds();
    const Vec3 dimensions = bounds.sizes();
    if (std::abs(dimensions.x() - scenario.hazard.diameter) > geom::Epsilon ||
        std::abs(dimensions.y() - scenario.hazard.diameter) > geom::Epsilon ||
        std::abs(dimensions.z() - scenario.hazard.height) > geom::Epsilon)
    {
        throw std::runtime_error("hazard mesh does not preserve its requested dimensions");
    }
    return world;
}

BenchmarkWorld makeTimingWorld(
    const hazard_timing_scene::MeshComplexity &complexity,
    int object_count)
{
    BenchmarkWorld world;
    const std::vector<hazard_timing_scene::ObjectSpec> specs =
        hazard_timing_scene::makeObjectCountSpecs(object_count);
    world.meshes.reserve(specs.size());
    world.obstacles.reserve(specs.size());
    for (const hazard_timing_scene::ObjectSpec &spec :
         specs)
    {
        addPillar(
            world, spec.x, spec.y, spec.radius, spec.height,
            hazard_timing_scene::slices, hazard_timing_scene::stacks,
            spec.object_id,
            complexity.subdivision_passes);
    }
    if (world.obstacles.size() != static_cast<size_t>(object_count))
        throw std::runtime_error("timing scene has the wrong object count");
    const size_t base_triangles =
        makePillarMeshData(
            0.0, 0.0, 1.0, 1.0,
            hazard_timing_scene::slices, hazard_timing_scene::stacks)
            .triangles.size();
    if (world.triangles_per_mesh !=
        base_triangles * static_cast<size_t>(complexity.multiplier))
    {
        throw std::runtime_error(
            std::string(complexity.name) +
            " timing meshes have the wrong triangle count");
    }
    finishWorld(world);
    return world;
}

BenchmarkWorld makeObjectCountTimingWorld(int object_count)
{
    BenchmarkWorld world = makeTimingWorld(
        hazard_timing_scene::meshComplexities.front(), object_count);
    if (world.triangles_per_mesh != hazard_timing_scene::trianglesPerObject)
        throw std::runtime_error("object-count timing meshes must have 120 triangles");
    return world;
}

Lidar makeHorizontalLidar(const Eigen::Isometry3d &pose)
{
    Lidar lidar;
    lidar.pose = pose;
    lidar.minRange = 0.1;
    lidar.maxRange = hazardMaxRange;
    lidar.minAzimuth = -90.0 * geom::deg;
    lidar.maxAzimuth = 90.0 * geom::deg;
    lidar.azimuthSamples = nestedHorizontalRayLayoutBins;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    return lidar;
}

Hit bruteIntersect(
    const BenchmarkWorld &world, const Ray &ray, double max_range)
{
    Hit closest;
    for (const auto &mesh : world.meshes)
    {
        const Hit hit = mesh->bruteForceIntersect(ray);
        if (hit.hit && hit.t <= max_range &&
            (!closest.hit || hit.t < closest.t))
        {
            closest = hit;
        }
    }
    return closest;
}

Hit meshBvhIntersect(
    const BenchmarkWorld &world, const Ray &ray, double max_range)
{
    Hit closest;
    for (const auto &mesh : world.meshes)
    {
        const Hit hit = mesh->intersect(ray);
        if (hit.hit && hit.t <= max_range &&
            (!closest.hit || hit.t < closest.t))
        {
            closest = hit;
        }
    }
    return closest;
}

ScanResult scanWorld(
    const BenchmarkWorld &world,
    TraceMode mode,
    const Lidar &lidar,
    const std::vector<double> &layout)
{
    if (mode == TraceMode::TrueBrute)
    {
        return scanFixedHorizontalWithIntersector(
            lidar, layout, [&](const Ray &ray) {
                return bruteIntersect(world, ray, lidar.maxRange);
            });
    }
    if (mode == TraceMode::MeshBvh)
    {
        return scanFixedHorizontalWithIntersector(
            lidar, layout, [&](const Ray &ray) {
                return meshBvhIntersect(world, ray, lidar.maxRange);
            });
    }
    return scanFixedHorizontalWithIntersector(
        lidar, layout, [&](const Ray &ray) {
            return world.bvh.intersect(ray, lidar.maxRange);
        });
}

void verifyParity(
    const ScanResult &brute,
    const ScanResult &accelerated,
    TraceMode accelerated_mode,
    unsigned scenario_id,
    int ray_count,
    unsigned frame)
{
    if (brute.rays_requested != ray_count ||
        accelerated.rays_requested != ray_count ||
        brute.rays_completed != ray_count ||
        accelerated.rays_completed != ray_count ||
        brute.points.size() != accelerated.points.size())
    {
        throw std::runtime_error(
            "fixed scan emitted the wrong ray count for scenario " +
            std::to_string(scenario_id));
    }

    for (size_t index = 0; index < brute.points.size(); ++index)
    {
        const ScanPoint &reference = brute.points[index];
        const ScanPoint &candidate = accelerated.points[index];
        if (reference.hit != candidate.hit ||
            reference.object_id != candidate.object_id ||
            std::abs(reference.range - candidate.range) > rangeTolerance)
        {
            throw std::runtime_error(
                std::string(modeName(accelerated_mode)) +
                " tracer mismatch for scenario " +
                std::to_string(scenario_id) + ", rays " +
                std::to_string(ray_count) + ", frame " +
                std::to_string(frame) + ", scan point " +
                std::to_string(index));
        }
    }
}

void writeTrial(
    std::ofstream &csv,
    const char *mode,
    const HazardScenario &scenario,
    int ray_count,
    const char *control,
    const HazardResult &result)
{
    csv << mode << ',' << scenario.scenario_id << ',' << ray_count << ','
        << control << ','
        << (result.outcome == HazardOutcome::SafeStop ? "SafeStop" : "Collision")
        << ',' << (result.detected ? "true" : "false") << ','
        << result.detection_range << ',' << result.unbraked_ttc << ','
        << result.stopping_margin << ',' << result.collision_speed
        << ",true,true,true\n";
}

void writeSpeedTrial(
    std::ofstream &csv,
    const char *mode,
    const HazardScenario &scenario,
    int ray_count,
    const char *control,
    const HazardResult &result)
{
    csv << mode << ',' << scenario.scenario_id << ',' << scenario.speed << ','
        << scenario.initial_clearance << ',' << scenario.hazard.diameter << ','
        << scenario.lateral_offset << ',' << scenario.azimuth_phase << ','
        << ray_count << ',' << control << ','
        << (result.outcome == HazardOutcome::SafeStop ? "SafeStop" : "Collision")
        << ',' << (result.detected ? "true" : "false") << ','
        << result.detection_range << ',' << result.unbraked_ttc << ','
        << result.stopping_margin << ',' << result.collision_speed
        << ",true,true,true\n";
}

void runSafety(const std::string &csv_path)
{
    std::ofstream csv(csv_path);
    if (!csv)
        throw std::runtime_error("cannot open CSV for writing: " + csv_path);
    csv << std::setprecision(17);
    csv << "mode,scenario_id,ray_count,control,outcome,detected,"
           "detection_range,unbraked_ttc,stopping_margin,collision_speed,"
           "hit_flags_equal,object_ids_equal,ranges_equal\n";

    const std::vector<HazardScenario> scenarios = generateHazardScenarios();
    if (scenarios.empty())
        throw std::runtime_error("scenario generation produced no valid scenarios");

    size_t rows = 0;
    for (const HazardScenario &scenario : scenarios)
    {
        const BenchmarkWorld world = makeSafetyWorld(scenario);
        for (const int ray_count : nestedHorizontalRayLayoutCounts)
        {
            const std::vector<double> layout =
                nestedHorizontalRayLayout(ray_count, scenario.azimuth_phase);
            const HazardResult result = runHazardTrial(
                scenario,
                [&](const HazardScenario &trial, const HazardFrame &frame) {
                    const Eigen::Isometry3d pose =
                        lidarPoseFromVehicle(frame.vehicle, VehicleConfig{});
                    const Lidar lidar = makeHorizontalLidar(pose);
                    const ScanResult brute =
                        scanWorld(world, TraceMode::TrueBrute, lidar, layout);
                    const ScanResult mesh_bvh =
                        scanWorld(world, TraceMode::MeshBvh, lidar, layout);
                    const ScanResult scene_bvh =
                        scanWorld(world, TraceMode::SceneBvh, lidar, layout);
                    verifyParity(
                        brute, mesh_bvh, TraceMode::MeshBvh,
                        trial.scenario_id, ray_count, frame.index);
                    verifyParity(
                        brute, scene_bvh, TraceMode::SceneBvh,
                        trial.scenario_id, ray_count, frame.index);
                    const std::optional<double> range =
                        scanRangeForObjectId(brute, trial.hazard.object_id);
                    return HazardDetection{
                        range.has_value(), range.value_or(-1.0)};
                });
            writeTrial(
                csv, modeName(TraceMode::TrueBrute), scenario, ray_count,
                "fixed_scan", result);
            writeTrial(
                csv, modeName(TraceMode::MeshBvh), scenario, ray_count,
                "fixed_scan", result);
            writeTrial(
                csv, modeName(TraceMode::SceneBvh), scenario, ray_count,
                "fixed_scan", result);
            rows += 3;
        }

        const HazardResult first_frame = runHazardTrial(
            scenario,
            [](const HazardScenario &trial, const HazardFrame &frame) {
                ScanResult control_scan;
                if (frame.index == 0)
                {
                    control_scan.points.push_back(
                        ScanPoint{0.0, 0.0, frame.hazard_range,
                                  trial.hazard.object_id, true});
                }
                const std::optional<double> range =
                    scanRangeForObjectId(
                        control_scan, trial.hazard.object_id);
                return HazardDetection{
                    range.has_value(), range.value_or(-1.0)};
            });
        writeTrial(
            csv, modeName(TraceMode::TrueBrute), scenario, 361,
            "first_frame_braking", first_frame);
        writeTrial(
            csv, modeName(TraceMode::MeshBvh), scenario, 361,
            "first_frame_braking", first_frame);
        writeTrial(
            csv, modeName(TraceMode::SceneBvh), scenario, 361,
            "first_frame_braking", first_frame);
        rows += 3;
    }
    csv.flush();
    if (!csv)
        throw std::runtime_error("failed while writing CSV: " + csv_path);
    std::cout << "[safety] wrote " << rows << " trial rows for "
              << scenarios.size() << " scenarios to " << csv_path << '\n';
}

void runSpeedSafety(const std::string &csv_path)
{
    std::ofstream csv(csv_path);
    if (!csv)
        throw std::runtime_error("cannot open CSV for writing: " + csv_path);
    csv << std::setprecision(17);
    csv << "mode,scenario_id,speed_mps,initial_clearance,hazard_diameter,"
           "lateral_offset,azimuth_phase,ray_count,control,outcome,detected,"
           "detection_range,unbraked_ttc,stopping_margin,collision_speed,"
           "hit_flags_equal,object_ids_equal,ranges_equal\n";

    const std::vector<HazardScenario> scenarios = generateHazardSpeedScenarios();
    constexpr size_t expectedScenarios =
        hazardSpeedSafetySpeeds.size() *
        hazardSpeedSafetyClearances.size() *
        hazardDiameters.size() * hazardOffsets.size() * hazardPhaseCount;
    if (scenarios.size() != expectedScenarios)
        throw std::runtime_error(
            "speed-safety scenario generation produced incomplete coverage");

    size_t rows = 0;
    for (const HazardScenario &scenario : scenarios)
    {
        const BenchmarkWorld world = makeSafetyWorld(scenario);
        for (const int ray_count : nestedHorizontalRayLayoutCounts)
        {
            const std::vector<double> layout =
                nestedHorizontalRayLayout(ray_count, scenario.azimuth_phase);
            const HazardResult result = runHazardSpeedTrial(
                scenario,
                [&](const HazardScenario &trial, const HazardFrame &frame) {
                    const Eigen::Isometry3d pose =
                        lidarPoseFromVehicle(frame.vehicle, VehicleConfig{});
                    const Lidar lidar = makeHorizontalLidar(pose);
                    const ScanResult brute =
                        scanWorld(world, TraceMode::TrueBrute, lidar, layout);
                    const ScanResult mesh_bvh =
                        scanWorld(world, TraceMode::MeshBvh, lidar, layout);
                    const ScanResult scene_bvh =
                        scanWorld(world, TraceMode::SceneBvh, lidar, layout);
                    verifyParity(
                        brute, mesh_bvh, TraceMode::MeshBvh,
                        trial.scenario_id, ray_count, frame.index);
                    verifyParity(
                        brute, scene_bvh, TraceMode::SceneBvh,
                        trial.scenario_id, ray_count, frame.index);
                    const std::optional<double> range =
                        scanRangeForObjectId(brute, trial.hazard.object_id);
                    return HazardDetection{
                        range.has_value(), range.value_or(-1.0)};
                });
            writeSpeedTrial(
                csv, modeName(TraceMode::TrueBrute), scenario, ray_count,
                "fixed_scan", result);
            writeSpeedTrial(
                csv, modeName(TraceMode::MeshBvh), scenario, ray_count,
                "fixed_scan", result);
            writeSpeedTrial(
                csv, modeName(TraceMode::SceneBvh), scenario, ray_count,
                "fixed_scan", result);
            rows += 3;
        }

        const HazardResult first_frame = runHazardSpeedTrial(
            scenario,
            [](const HazardScenario &trial, const HazardFrame &frame) {
                ScanResult control_scan;
                if (frame.index == 0)
                {
                    control_scan.points.push_back(
                        ScanPoint{0.0, 0.0, frame.hazard_range,
                                  trial.hazard.object_id, true});
                }
                const std::optional<double> range =
                    scanRangeForObjectId(control_scan, trial.hazard.object_id);
                return HazardDetection{
                    range.has_value(), range.value_or(-1.0)};
            });
        for (const TraceMode mode :
             {TraceMode::TrueBrute, TraceMode::MeshBvh, TraceMode::SceneBvh})
        {
            writeSpeedTrial(
                csv, modeName(mode), scenario, 361,
                "first_frame_braking", first_frame);
            ++rows;
        }
    }

    constexpr size_t expectedRows = expectedScenarios *
                                    (nestedHorizontalRayLayoutCounts.size() + 1) *
                                    3;
    if (rows != expectedRows)
        throw std::runtime_error("speed-safety output has the wrong row count");
    csv.flush();
    if (!csv)
        throw std::runtime_error("failed while writing CSV: " + csv_path);
    std::cout << "[speed-safety] wrote " << rows << " trial rows for "
              << scenarios.size() << " scenarios to " << csv_path << '\n';
}

std::vector<Eigen::Isometry3d> makeTimingPoses()
{
    std::vector<Eigen::Isometry3d> poses;
    poses.reserve(hazardPhaseCount);
    for (unsigned index = 0; index < hazardPhaseCount; ++index)
    {
        VehicleState vehicle;
        vehicle.x = 0.35 * (index % 8);
        vehicle.y = -0.7 + 0.2 * (index % 8);
        vehicle.heading =
            (-6.0 + 12.0 * (index % 7) / 6.0) * geom::deg;
        poses.push_back(lidarPoseFromVehicle(vehicle, VehicleConfig{}));
    }
    return poses;
}

double percentile(std::vector<double> values, double fraction)
{
    if (values.empty())
        throw std::runtime_error("cannot aggregate an empty timing sample");
    std::sort(values.begin(), values.end());
    const double position = fraction * (values.size() - 1);
    const size_t lower = static_cast<size_t>(std::floor(position));
    const size_t upper = static_cast<size_t>(std::ceil(position));
    return values[lower] +
           (values[upper] - values[lower]) * (position - lower);
}

void consumeScan(const ScanResult &scan, volatile double &checksum)
{
    checksum += scan.rays_completed;
    for (const ScanPoint &point : scan.points)
    {
        checksum += point.hit
                        ? point.range + 1e-6 * point.object_id
                        : point.range;
    }
}

void verifyTimingScene(
    const BenchmarkWorld &world,
    const std::vector<Eigen::Isometry3d> &poses)
{
    for (unsigned index = 0; index < poses.size(); ++index)
    {
        const Lidar lidar = makeHorizontalLidar(poses[index]);
        for (const int ray_count : nestedHorizontalRayLayoutCounts)
        {
            const std::vector<double> layout =
                nestedHorizontalRayLayout(
                    ray_count,
                    static_cast<double>(index) / hazardPhaseCount);
            const ScanResult brute =
                scanWorld(world, TraceMode::TrueBrute, lidar, layout);
            const ScanResult mesh_bvh =
                scanWorld(world, TraceMode::MeshBvh, lidar, layout);
            const ScanResult scene_bvh =
                scanWorld(world, TraceMode::SceneBvh, lidar, layout);
            verifyParity(
                brute, mesh_bvh, TraceMode::MeshBvh, index, ray_count, 0);
            verifyParity(
                brute, scene_bvh, TraceMode::SceneBvh, index, ray_count, 0);
        }
    }
}

std::pair<double, double> measureTiming(
    const BenchmarkWorld &world,
    TraceMode mode,
    int ray_count,
    const std::vector<Eigen::Isometry3d> &poses)
{
    std::vector<std::vector<double>> layouts;
    layouts.reserve(hazardPhaseCount);
    for (unsigned phase = 0; phase < hazardPhaseCount; ++phase)
    {
        layouts.push_back(nestedHorizontalRayLayout(
            ray_count, static_cast<double>(phase) / hazardPhaseCount));
    }

    volatile double checksum = 0.0;
    for (int scan_index = 0; scan_index < timingWarmupScans; ++scan_index)
    {
        const unsigned pose_index =
            static_cast<unsigned>(scan_index) % hazardPhaseCount;
        const Lidar lidar = makeHorizontalLidar(poses[pose_index]);
        consumeScan(
            scanWorld(world, mode, lidar, layouts[pose_index]), checksum);
    }

    std::vector<double> samples_ms;
    samples_ms.reserve(timingMeasuredScans);
    for (int scan_index = 0; scan_index < timingMeasuredScans; ++scan_index)
    {
        const unsigned pose_index =
            static_cast<unsigned>(scan_index) % hazardPhaseCount;
        const Lidar lidar = makeHorizontalLidar(poses[pose_index]);
        const auto start = std::chrono::steady_clock::now();
        const ScanResult scan =
            scanWorld(world, mode, lidar, layouts[pose_index]);
        const auto end = std::chrono::steady_clock::now();
        samples_ms.push_back(
            std::chrono::duration<double, std::milli>(end - start).count());
        consumeScan(scan, checksum);
    }
    (void)checksum;
    return {percentile(samples_ms, 0.5), percentile(samples_ms, 0.95)};
}

void runTiming(const std::string &csv_path)
{
    std::ofstream csv(csv_path);
    if (!csv)
        throw std::runtime_error("cannot open CSV for writing: " + csv_path);
    csv << std::setprecision(17);
    csv << "complexity,triangles_per_mesh,mode,ray_count,median_ms,p95_ms\n";

    const std::vector<Eigen::Isometry3d> poses = makeTimingPoses();
    size_t rows = 0;
    for (const hazard_timing_scene::MeshComplexity &complexity :
         hazard_timing_scene::meshComplexities)
    {
        const BenchmarkWorld world =
            makeTimingWorld(complexity, timingBackgroundCount);
        verifyTimingScene(world, poses);
        for (const TraceMode mode :
             {TraceMode::TrueBrute, TraceMode::MeshBvh, TraceMode::SceneBvh})
        {
            for (const int ray_count : nestedHorizontalRayLayoutCounts)
            {
                const auto result =
                    measureTiming(world, mode, ray_count, poses);
                csv << complexity.name << ',' << world.triangles_per_mesh << ','
                    << modeName(mode) << ',' << ray_count << ','
                    << result.first << ',' << result.second << '\n';
                ++rows;
            }
        }
    }
    csv.flush();
    if (!csv)
        throw std::runtime_error("failed while writing CSV: " + csv_path);
    std::cout << "[timing] wrote " << rows
              << " rows for 3 complexity levels using exactly "
              << timingBackgroundCount << " background objects, "
              << timingWarmupScans << " warmups and "
              << timingMeasuredScans << " measured scans per row to "
              << csv_path << '\n';
}

void runComplexityObjectCountTiming(const std::string &csv_path)
{
    std::ofstream csv(csv_path);
    if (!csv)
        throw std::runtime_error("cannot open CSV for writing: " + csv_path);
    csv << std::setprecision(17);
    csv << "complexity,object_count,triangles_per_mesh,mode,ray_count,"
           "median_ms,p95_ms\n";

    const std::vector<Eigen::Isometry3d> poses = makeTimingPoses();
    size_t rows = 0;
    for (const hazard_timing_scene::MeshComplexity &complexity :
         hazard_timing_scene::meshComplexities)
    {
        for (const int object_count :
             hazard_timing_scene::matrixObjectCounts)
        {
            const BenchmarkWorld world =
                makeTimingWorld(complexity, object_count);
            verifyTimingScene(world, poses);
            for (const TraceMode mode :
                 {TraceMode::TrueBrute, TraceMode::MeshBvh,
                  TraceMode::SceneBvh})
            {
                for (const int ray_count : nestedHorizontalRayLayoutCounts)
                {
                    const auto result =
                        measureTiming(world, mode, ray_count, poses);
                    csv << complexity.name << ',' << object_count << ','
                        << world.triangles_per_mesh << ','
                        << modeName(mode) << ',' << ray_count << ','
                        << result.first << ',' << result.second << '\n';
                    ++rows;
                }
            }
        }
    }
    csv.flush();
    if (!csv)
        throw std::runtime_error("failed while writing CSV: " + csv_path);
    std::cout << "[complexity-object-count-timing] wrote " << rows
              << " rows for 3 complexity levels by 3 object counts, "
              << timingWarmupScans << " warmups and "
              << timingMeasuredScans << " measured scans per row to "
              << csv_path << '\n';
}

void runObjectCountTiming(const std::string &csv_path)
{
    std::ofstream csv(csv_path);
    if (!csv)
        throw std::runtime_error("cannot open CSV for writing: " + csv_path);
    csv << std::setprecision(17);
    csv << "object_count,triangles_per_mesh,mode,ray_count,median_ms,p95_ms\n";

    const std::vector<Eigen::Isometry3d> poses = makeTimingPoses();
    size_t rows = 0;
    for (const int object_count : hazard_timing_scene::objectCounts)
    {
        const BenchmarkWorld world = makeObjectCountTimingWorld(object_count);
        verifyTimingScene(world, poses);
        for (const TraceMode mode :
             {TraceMode::TrueBrute, TraceMode::MeshBvh, TraceMode::SceneBvh})
        {
            for (const int ray_count : nestedHorizontalRayLayoutCounts)
            {
                const auto result =
                    measureTiming(world, mode, ray_count, poses);
                csv << object_count << ',' << world.triangles_per_mesh << ','
                    << modeName(mode) << ',' << ray_count << ','
                    << result.first << ',' << result.second << '\n';
                ++rows;
            }
        }
    }
    csv.flush();
    if (!csv)
        throw std::runtime_error("failed while writing CSV: " + csv_path);
    std::cout << "[object-count-timing] wrote " << rows
              << " rows for 5 object counts using exactly "
              << hazard_timing_scene::trianglesPerObject
              << " triangles per object, "
              << timingWarmupScans << " warmups and "
              << timingMeasuredScans << " measured scans per row to "
              << csv_path << '\n';
}

void printUsage(std::ostream &output)
{
    output << "Usage:\n"
           << "  hazard_benchmark safety --csv PATH\n"
           << "  hazard_benchmark speed-safety --csv PATH\n"
           << "  hazard_benchmark timing --csv PATH\n"
           << "  hazard_benchmark object-count-timing --csv PATH\n"
           << "  hazard_benchmark complexity-object-count-timing --csv PATH\n";
}
} // namespace

int main(int argc, char **argv)
{
    if (argc != 4 || std::string(argv[2]) != "--csv" ||
        std::string(argv[3]).empty() ||
        (std::string(argv[1]) != "safety" &&
         std::string(argv[1]) != "speed-safety" &&
         std::string(argv[1]) != "timing" &&
         std::string(argv[1]) != "object-count-timing" &&
         std::string(argv[1]) != "complexity-object-count-timing"))
    {
        std::cerr << "error: expected exactly one mode and --csv PATH\n";
        printUsage(std::cerr);
        return 2;
    }

    try
    {
        if (std::string(argv[1]) == "safety")
            runSafety(argv[3]);
        else if (std::string(argv[1]) == "speed-safety")
            runSpeedSafety(argv[3]);
        else if (std::string(argv[1]) == "timing")
            runTiming(argv[3]);
        else if (std::string(argv[1]) == "object-count-timing")
            runObjectCountTiming(argv[3]);
        else
            runComplexityObjectCountTiming(argv[3]);
    }
    catch (const std::exception &error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
