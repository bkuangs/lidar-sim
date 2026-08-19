#include "hazard_trial.hpp"
#include "hazard_timing_scene.hpp"
#include "meshes.hpp"
#include "scan.hpp"
#include "scene_bvh.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <map>
#include <set>
#include <string>
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

void assertObjectSpecEqual(
    const hazard_timing_scene::ObjectSpec &reference,
    const hazard_timing_scene::ObjectSpec &candidate)
{
    assert(reference.x == candidate.x);
    assert(reference.y == candidate.y);
    assert(reference.radius == candidate.radius);
    assert(reference.height == candidate.height);
    assert(reference.object_id == candidate.object_id);
}

void testNestedObjectCountTimingScenes()
{
    using hazard_timing_scene::ObjectSpec;
    const std::vector<ObjectSpec> all =
        hazard_timing_scene::makeObjectCountSpecs();
    const std::vector<ObjectSpec> repeated =
        hazard_timing_scene::makeObjectCountSpecs();
    assert(all.size() == hazard_timing_scene::maxObjectCount);
    assert(repeated.size() == all.size());
    for (size_t index = 0; index < all.size(); ++index)
        assertObjectSpecEqual(all[index], repeated[index]);

    std::set<int> object_ids;
    for (const ObjectSpec &spec : all)
    {
        assert(object_ids.insert(spec.object_id).second);
        const TriangleMeshData mesh = makePillarMeshData(
            spec.x, spec.y, spec.radius, spec.height,
            hazard_timing_scene::slices, hazard_timing_scene::stacks);
        assert(mesh.triangles.size() ==
               hazard_timing_scene::trianglesPerObject);
    }

    const std::vector<ObjectSpec> pr2 =
        hazard_timing_scene::makePr2ObjectSpecs();
    const std::vector<ObjectSpec> nested_100 =
        hazard_timing_scene::makeObjectCountSpecs(100);
    assert(pr2.size() == nested_100.size());
    for (size_t index = 0; index < pr2.size(); ++index)
    {
        assertObjectSpecEqual(pr2[index], nested_100[index]);
        assertObjectSpecEqual(pr2[index], all[index]);
    }
    std::map<int, ObjectSpec> canonical_by_id;
    for (const ObjectSpec &spec : all)
        canonical_by_id.emplace(spec.object_id, spec);

    double min_x = pr2.front().x - pr2.front().radius;
    double max_x = pr2.front().x + pr2.front().radius;
    double min_y = pr2.front().y - pr2.front().radius;
    double max_y = pr2.front().y + pr2.front().radius;
    for (const ObjectSpec &spec : pr2)
    {
        min_x = std::min(min_x, spec.x - spec.radius);
        max_x = std::max(max_x, spec.x + spec.radius);
        min_y = std::min(min_y, spec.y - spec.radius);
        max_y = std::max(max_y, spec.y + spec.radius);
    }

    std::set<int> previous_ids;
    for (const int object_count : hazard_timing_scene::objectCounts)
    {
        const std::vector<ObjectSpec> selected =
            hazard_timing_scene::makeObjectCountSpecs(object_count);
        assert(selected.size() == static_cast<size_t>(object_count));
        assert(std::is_sorted(
            selected.begin(), selected.end(),
            [](const ObjectSpec &left, const ObjectSpec &right)
            { return left.object_id < right.object_id; }));
        std::set<std::pair<int, int>> occupied_macro_cells;
        std::set<int> selected_ids;
        for (const ObjectSpec &spec : selected)
        {
            assertObjectSpecEqual(canonical_by_id.at(spec.object_id), spec);
            assert(selected_ids.insert(spec.object_id).second);
            assert(spec.x - spec.radius >= min_x);
            assert(spec.x + spec.radius <= max_x);
            assert(spec.y - spec.radius >= min_y);
            assert(spec.y + spec.radius <= max_y);
            if (spec.object_id < 1100)
            {
                const int original_index = spec.object_id - 1000;
                occupied_macro_cells.emplace(
                    (original_index / 10) / 2,
                    (original_index % 10) / 2);
            }
        }
        assert(std::includes(
            selected_ids.begin(), selected_ids.end(),
            previous_ids.begin(), previous_ids.end()));
        assert(occupied_macro_cells.size() == 25);
        previous_ids = std::move(selected_ids);
    }
}

void testComplexityObjectCountMatrixCells()
{
    using hazard_timing_scene::MeshComplexity;
    using hazard_timing_scene::ObjectSpec;
    assert(hazard_timing_scene::meshComplexities.size() == 3);
    assert(hazard_timing_scene::matrixObjectCounts ==
           (std::array<int, 3>{{25, 100, 400}}));

    const std::array<const char *, 3> expected_names = {{"1x", "4x", "16x"}};
    const std::array<int, 3> expected_multipliers = {{1, 4, 16}};
    const std::array<unsigned, 3> expected_passes = {{0, 1, 2}};
    for (size_t index = 0;
         index < hazard_timing_scene::meshComplexities.size();
         ++index)
    {
        const MeshComplexity &complexity =
            hazard_timing_scene::meshComplexities[index];
        assert(std::string(complexity.name) == expected_names[index]);
        assert(complexity.multiplier == expected_multipliers[index]);
        assert(complexity.subdivision_passes == expected_passes[index]);
    }

    size_t cells = 0;
    for (const MeshComplexity &complexity :
         hazard_timing_scene::meshComplexities)
    {
        const size_t expected_triangles =
            hazard_timing_scene::trianglesPerObject *
            static_cast<size_t>(complexity.multiplier);
        for (const int object_count :
             hazard_timing_scene::matrixObjectCounts)
        {
            const std::vector<ObjectSpec> specs =
                hazard_timing_scene::makeObjectCountSpecs(object_count);
            const std::vector<ObjectSpec> repeated =
                hazard_timing_scene::makeObjectCountSpecs(object_count);
            assert(specs.size() == static_cast<size_t>(object_count));
            assert(repeated.size() == specs.size());

            size_t total_triangles = 0;
            for (size_t index = 0; index < specs.size(); ++index)
            {
                assertObjectSpecEqual(specs[index], repeated[index]);
                const TriangleMeshData base = makePillarMeshData(
                    specs[index].x, specs[index].y, specs[index].radius,
                    specs[index].height, hazard_timing_scene::slices,
                    hazard_timing_scene::stacks);
                const TriangleMeshData subdivided = subdivideTriangleFaces(
                    base, complexity.subdivision_passes);
                assert(subdivided.triangles.size() == expected_triangles);
                total_triangles += subdivided.triangles.size();
            }
            assert(total_triangles ==
                   static_cast<size_t>(object_count) * expected_triangles);
            ++cells;
        }
    }
    assert(cells == 9);
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
    testNestedObjectCountTimingScenes();
    testComplexityObjectCountMatrixCells();
    testFixedScansAndTracerParity();
    testHazardGeometryDimensions();
    testOnlyHazardObjectIdTriggersDetection();
    return 0;
}
