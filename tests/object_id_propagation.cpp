#include "accelerated_scene.hpp"
#include "scan.hpp"
#include "scene.hpp"
#include "scene_bvh.hpp"
#include <cassert>
#include <memory>

namespace
{
Lidar forwardLidar()
{
    Lidar lidar;
    lidar.minAzimuth = 0.0;
    lidar.maxAzimuth = 0.0;
    lidar.azimuthSamples = 1;
    lidar.minElevation = 0.0;
    lidar.maxElevation = 0.0;
    lidar.elevationSamples = 1;
    lidar.maxRange = 20.0;
    return lidar;
}

void assertSingleObjectId(const ScanResult &scan, int object_id)
{
    assert(scan.points.size() == 1);
    assert(scan.points.front().hit);
    assert(scan.points.front().object_id == object_id);
}

void testObjectIdsAcrossSceneTracers()
{
    constexpr int object_id = 42;
    const Lidar lidar = forwardLidar();
    const auto geometry =
        std::make_shared<SphereGeometry>(Vec3(5.0, 0.0, 0.0), 1.0, object_id);

    Scene linear;
    linear.objects.push_back(geometry);
    assertSingleObjectId(scanWithIntersector(
                             lidar,
                             [&](const Ray &ray) { return linear.intersect(ray); }),
                         object_id);

    ActiveObstacle obstacle;
    obstacle.x = 5.0;
    obstacle.center = Vec3(5.0, 0.0, 0.0);
    obstacle.size = 2.0;
    obstacle.object_id = object_id;
    obstacle.bounds = geometry->bounds();
    obstacle.geometry = geometry;
    const std::vector<ActiveObstacle> obstacles = {obstacle};

    XBucketScene buckets;
    buckets.rebuild({}, obstacles);
    assertSingleObjectId(scanWithIntersector(
                             lidar,
                             [&](const Ray &ray) {
                                 return buckets.intersect(ray, lidar.maxRange);
                             }),
                         object_id);

    SceneBVH bvh;
    bvh.rebuild({}, obstacles);
    assertSingleObjectId(scanWithIntersector(
                             lidar,
                             [&](const Ray &ray) {
                                 return bvh.intersect(ray, lidar.maxRange);
                             }),
                         object_id);
    assertSingleObjectId(scanFixedHorizontalWithIntersector(
                            lidar,
                            {0.0},
                            [&](const Ray &ray) {
                                return bvh.intersect(ray, lidar.maxRange);
                            }),
                         object_id);
}

void testClearRaysHaveNoObjectId()
{
    const ScanResult scan = scanWithIntersector(
        forwardLidar(), [](const Ray &) { return Hit{}; });
    assert(scan.points.size() == 1);
    assert(!scan.points.front().hit);
    assert(scan.points.front().object_id == -1);

    auto out_of_range_intersector = [](const Ray &) {
        return Hit{true, 25.0, 99};
    };
    const ScanPoint out_of_range = traceScanPoint(
        forwardLidar(), 0.0, 0.0, out_of_range_intersector);
    assert(!out_of_range.hit);
    assert(out_of_range.object_id == -1);
}
} // namespace

int main()
{
    testObjectIdsAcrossSceneTracers();
    testClearRaysHaveNoObjectId();
    return 0;
}
