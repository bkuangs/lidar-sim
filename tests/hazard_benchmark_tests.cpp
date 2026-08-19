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

void assertHitEqual(const Hit &reference, const Hit &candidate)
{
    assert(reference.hit == candidate.hit);
    assert(reference.objId == candidate.objId);
    assert(std::abs(reference.t - candidate.t) <= rangeTolerance);
}

void assertParity(const ScanResult &reference, const ScanResult &candidate)
{
    assert(reference.rays_requested == candidate.rays_requested);
    assert(reference.rays_completed == candidate.rays_completed);
    assert(reference.points.size() == candidate.points.size());
    for (size_t i = 0; i < reference.points.size(); ++i)
    {
        assert(reference.points[i].hit == candidate.points[i].hit);
        assert(reference.points[i].object_id == candidate.points[i].object_id);
        assert(std::abs(reference.points[i].range - candidate.points[i].range) <=
               rangeTolerance);
    }
}

void testGeometryPreservingSubdivision()
{
    constexpr int objectId = 73;
    const TriangleMeshData base{
        {
            Vec3(2.0, -1.0, 0.0),
            Vec3(2.0, 1.0, 0.0),
            Vec3(2.0, 0.0, 2.0),
        },
        {{0, 1, 2}},
    };
    const TriangleMeshData four_x = subdivideTriangleFaces(base, 1);
    const TriangleMeshData sixteen_x = subdivideTriangleFaces(base, 2);
    assert(base.triangles.size() == 1);
    assert(four_x.triangles.size() == 4);
    assert(sixteen_x.triangles.size() == 16);

    const TriangleMeshGeometry base_geometry(
        base.vertices, base.triangles, objectId);
    const TriangleMeshGeometry four_x_geometry(
        four_x.vertices, four_x.triangles, objectId);
    const TriangleMeshGeometry sixteen_x_geometry(
        sixteen_x.vertices, sixteen_x.triangles, objectId);
    for (const TriangleMeshGeometry *candidate :
         {&four_x_geometry, &sixteen_x_geometry})
    {
        assert((base_geometry.bounds().min() - candidate->bounds().min()).norm() <=
               geom::Epsilon);
        assert((base_geometry.bounds().max() - candidate->bounds().max()).norm() <=
               geom::Epsilon);
        assert((base_geometry.bounds().sizes() - candidate->bounds().sizes()).norm() <=
               geom::Epsilon);
    }

    const std::vector<Vec3> targets = {
        Vec3(2.0, 0.0, 2.0 / 3.0), // face interior
        Vec3(2.0, 0.0, 0.0),       // original edge
        Vec3(2.0, -0.25, 0.5),     // subdivision edge
        Vec3(2.0, -1.0, 0.0),      // vertex
        Vec3(2.0, 0.0, -1e-7),     // near-edge miss
        Vec3(2.0, 1.1, 0.1),       // outside miss
    };
    std::vector<Ray> rays;
    for (const Vec3 &target : targets)
        rays.push_back(Ray{Vec3::Zero(), target.normalized()});
    rays.push_back(Ray{Vec3(0.0, 0.0, 1.0), Vec3(0.0, 1.0, 0.0)});
    rays.push_back(Ray{Vec3(3.0, 0.0, 0.5), Vec3(1.0, 0.0, 0.0)});

    for (const Ray &ray : rays)
    {
        const Hit reference = base_geometry.bruteForceIntersect(ray);
        for (const TriangleMeshGeometry *candidate :
             {&four_x_geometry, &sixteen_x_geometry})
        {
            assertHitEqual(reference, candidate->bruteForceIntersect(ray));
            assertHitEqual(reference, candidate->intersect(ray));
        }
    }
}

void testTimingPillarTriangleCounts()
{
    constexpr int objectId = 91;
    const TriangleMeshData base =
        makePillarMeshData(4.0, 0.15, 0.25, 2.0, 12, 4);
    const TriangleMeshData four_x = subdivideTriangleFaces(base, 1);
    const TriangleMeshData sixteen_x = subdivideTriangleFaces(base, 2);
    assert(base.triangles.size() == 120);
    assert(four_x.triangles.size() == 480);
    assert(sixteen_x.triangles.size() == 1920);

    const TriangleMeshGeometry base_geometry(
        base.vertices, base.triangles, objectId);
    const Ray side_ray{Vec3(0.0, 0.15, 0.75), Vec3(1.0, 0.0, 0.0)};
    for (const TriangleMeshData *data : {&four_x, &sixteen_x})
    {
        const TriangleMeshGeometry candidate(
            data->vertices, data->triangles, objectId);
        assert((base_geometry.bounds().min() - candidate.bounds().min()).norm() <=
               geom::Epsilon);
        assert((base_geometry.bounds().max() - candidate.bounds().max()).norm() <=
               geom::Epsilon);
        assertHitEqual(
            base_geometry.bruteForceIntersect(side_ray),
            candidate.bruteForceIntersect(side_ray));
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
        const ScanResult mesh_bvh = scanFixedHorizontalWithIntersector(
            lidar, layout,
            [&](const Ray &ray) { return hazard->intersect(ray); });
        const ScanResult scene_bvh = scanFixedHorizontalWithIntersector(
            lidar, layout,
            [&](const Ray &ray) { return bvh.intersect(ray, lidar.maxRange); });
        assert(brute.rays_requested == ray_count);
        assert(brute.rays_completed == ray_count);
        assert(static_cast<int>(brute.points.size()) == ray_count);
        assertParity(brute, mesh_bvh);
        assertParity(brute, scene_bvh);
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
    testGeometryPreservingSubdivision();
    testTimingPillarTriangleCounts();
    testFixedScansAndTracerParity();
    testHazardGeometryDimensions();
    testOnlyHazardObjectIdTriggersDetection();
    return 0;
}
