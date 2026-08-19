#include "hazard_trial.hpp"
#include "meshes.hpp"
#include "scan.hpp"
#include "scene_bvh.hpp"

#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

namespace
{
constexpr double rangeTolerance = 1e-6;

void assertParity(const ScanResult &brute, const ScanResult &bvh)
{
    assert(brute.rays_requested == bvh.rays_requested);
    assert(brute.rays_completed == bvh.rays_completed);
    assert(brute.points.size() == bvh.points.size());
    for (size_t i = 0; i < brute.points.size(); ++i)
    {
        assert(brute.points[i].hit == bvh.points[i].hit);
        assert(brute.points[i].object_id == bvh.points[i].object_id);
        assert(std::abs(brute.points[i].range - bvh.points[i].range) <=
               rangeTolerance);
    }
}

void testFixedScansAndTracerParity()
{
    constexpr int hazardId = 1;
    auto hazard = makePillar(4.0, 0.15, 0.25, 2.0, 28, 8, hazardId);
    ActiveObstacle obstacle;
    obstacle.x = 4.0;
    obstacle.center = Vec3(4.0, 0.15, 1.0);
    obstacle.size = 0.5;
    obstacle.object_id = hazardId;
    obstacle.bounds = hazard->bounds();
    obstacle.geometry = hazard;

    const std::vector<ActiveObstacle> obstacles = {obstacle};
    SceneBVH bvh;
    bvh.rebuild({}, obstacles);

    Lidar lidar;
    lidar.pose.translation() = Vec3(0.0, 0.0, 1.2);
    lidar.maxRange = hazardMaxRange;
    for (const int ray_count : nestedHorizontalRayLayoutCounts)
    {
        const std::vector<double> layout =
            nestedHorizontalRayLayout(ray_count, 0.25);
        const ScanResult brute = scanFixedHorizontalWithIntersector(
            lidar, layout,
            [&](const Ray &ray) { return hazard->bruteForceIntersect(ray); });
        const ScanResult accelerated = scanFixedHorizontalWithIntersector(
            lidar, layout,
            [&](const Ray &ray) { return bvh.intersect(ray, lidar.maxRange); });
        assert(brute.rays_requested == ray_count);
        assert(brute.rays_completed == ray_count);
        assert(static_cast<int>(brute.points.size()) == ray_count);
        assertParity(brute, accelerated);
    }
}

void testHazardGeometryDimensions()
{
    const auto hazard = makePillar(3.0, -0.2, 0.25, 2.0, 28, 8, 1);
    const AABB bounds = hazard->bounds();
    assert(std::abs(bounds.sizes().x() - 0.5) <= geom::Epsilon);
    assert(std::abs(bounds.sizes().y() - 0.5) <= geom::Epsilon);
    assert(std::abs(bounds.sizes().z() - 2.0) <= geom::Epsilon);
}

void testOnlyHazardObjectIdTriggersDetection()
{
    ScanResult scan;
    scan.points.push_back(ScanPoint{0.0, 0.0, 1.0, 99, true});
    assert(!scanContainsObjectId(scan, 1));
    scan.points.push_back(ScanPoint{0.1, 0.0, 2.0, 1, true});
    scan.points.push_back(ScanPoint{0.2, 0.0, 1.5, 1, true});
    assert(scanContainsObjectId(scan, 1));
    assert(std::abs(*scanRangeForObjectId(scan, 1) - 1.5) <= geom::Epsilon);
}
} // namespace

int main()
{
    testFixedScansAndTracerParity();
    testHazardGeometryDimensions();
    testOnlyHazardObjectIdTriggersDetection();
    return 0;
}
