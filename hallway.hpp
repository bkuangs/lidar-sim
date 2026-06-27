#pragma once
#include "scene.hpp"
#include <algorithm>
#include <random>

enum class ObstacleShape
{
    Sphere,
    Cube
};

struct ActiveObstacle
{
    double x = 0.0;
    Vec3 center = Vec3::Zero();
    double size = 0.0;
    int object_id = -1;
    ObstacleShape shape = ObstacleShape::Cube;
    AABB bounds;
    std::shared_ptr<Geometry> geometry;
};

struct HallwayWorld
{
    double half_width = 2.0;
    double height = 3.0;
    double window_behind = 8.0;
    double window_ahead = 60.0;
    double next_obstacle_x = 6.0;
    int next_object_id = 10;
    unsigned int seed = 42;
    int revision = 0;
    std::mt19937 rng;
    std::vector<ActiveObstacle> obstacles;
};

std::shared_ptr<TriangleMeshGeometry> makeCube(
    const Vec3& center,
    double size,
    int object_id)
{
    double h = size * 0.5;

    std::vector<Vec3> v = {
        {center.x() - h, center.y() - h, center.z() - h},
        {center.x() + h, center.y() - h, center.z() - h},
        {center.x() + h, center.y() + h, center.z() - h},
        {center.x() - h, center.y() + h, center.z() - h},
        {center.x() - h, center.y() - h, center.z() + h},
        {center.x() + h, center.y() - h, center.z() + h},
        {center.x() + h, center.y() + h, center.z() + h},
        {center.x() - h, center.y() + h, center.z() + h},
    };

    std::vector<Eigen::Vector3i> t = {
        {0, 1, 2}, {0, 2, 3},
        {4, 6, 5}, {4, 7, 6},
        {0, 4, 5}, {0, 5, 1},
        {1, 5, 6}, {1, 6, 2},
        {2, 6, 7}, {2, 7, 3},
        {3, 7, 4}, {3, 4, 0},
    };

    return std::make_shared<TriangleMeshGeometry>(v, t, object_id);
}

HallwayWorld makeHallwayWorld(unsigned int seed = 42)
{
    std::mt19937 rng(seed);

    std::uniform_real_distribution<double> width_dist(3.0, 5.0);
    std::uniform_real_distribution<double> height_dist(2.6, 4.0);

    HallwayWorld world;
    world.seed = seed;
    world.half_width = width_dist(rng) * 0.5;
    world.height = height_dist(rng);
    world.rng = std::move(rng);

    return world;
}

ActiveObstacle makeRandomObstacle(HallwayWorld &world, double x)
{
    constexpr double wall_margin = 0.15;

    std::uniform_real_distribution<double> size_dist(0.35, 1.1);
    std::uniform_int_distribution<int> shape_dist(0, 1);

    double size = size_dist(world.rng);
    double half = size * 0.5;

    std::uniform_real_distribution<double> y_dist(
        -world.half_width + half + wall_margin,
        world.half_width - half - wall_margin);

    Vec3 center(x, y_dist(world.rng), half);
    int object_id = world.next_object_id++;
    ObstacleShape shape = shape_dist(world.rng) == 0
                              ? ObstacleShape::Sphere
                              : ObstacleShape::Cube;

    std::shared_ptr<Geometry> geometry = shape == ObstacleShape::Sphere
                                             ? std::static_pointer_cast<Geometry>(
                                                   std::make_shared<SphereGeometry>(center, half, object_id))
                                             : std::static_pointer_cast<Geometry>(
                                                   makeCube(center, size, object_id));

    ActiveObstacle obstacle;
    obstacle.x = x;
    obstacle.center = center;
    obstacle.size = size;
    obstacle.object_id = object_id;
    obstacle.shape = shape;
    obstacle.bounds = geometry->bounds();
    obstacle.geometry = geometry;

    return obstacle;
}

void updateObstacleWindow(HallwayWorld &world, double lidar_x)
{
    const double min_x = lidar_x - world.window_behind;
    const size_t before = world.obstacles.size();
    world.obstacles.erase(
        std::remove_if(
            world.obstacles.begin(),
            world.obstacles.end(),
            [min_x](const ActiveObstacle &obstacle)
            {
                return obstacle.x < min_x;
            }),
        world.obstacles.end());

    bool changed = world.obstacles.size() != before;

    std::uniform_real_distribution<double> spacing_dist(2.5, 6.0);
    const double max_x = lidar_x + world.window_ahead;

    while (world.next_obstacle_x < max_x)
    {
        world.obstacles.push_back(makeRandomObstacle(world, world.next_obstacle_x));

        world.next_obstacle_x += spacing_dist(world.rng);
        changed = true;
    }

    if (changed)
        ++world.revision;
}

std::vector<std::shared_ptr<Geometry>> makeHallwayStaticObjects(const HallwayWorld &world)
{
    std::vector<std::shared_ptr<Geometry>> objects;

    objects.push_back(std::make_shared<PlaneGeometry>(
        Vec3(0, 0, 1), Vec3(0, 0, 0), 0)); // floor

    objects.push_back(std::make_shared<PlaneGeometry>(
        Vec3(0, 0, -1), Vec3(0, 0, world.height), 1)); // ceiling

    objects.push_back(std::make_shared<PlaneGeometry>(
        Vec3(0, 1, 0), Vec3(0, -world.half_width, 0), 2)); // left wall

    objects.push_back(std::make_shared<PlaneGeometry>(
        Vec3(0, -1, 0), Vec3(0, world.half_width, 0), 3)); // right wall

    return objects;
}

Scene makeHallwayScene(const HallwayWorld &world)
{
    Scene scene;

    scene.objects = makeHallwayStaticObjects(world);

    for (const ActiveObstacle &obstacle : world.obstacles)
        scene.objects.push_back(obstacle.geometry);

    return scene;
}
