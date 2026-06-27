#include "simulation.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

bool near(double a, double b, double eps = 1e-9)
{
    return std::abs(a - b) <= eps;
}

void testDeterministicHallway()
{
    HallwayWorld a = makeHallwayWorld(123);
    HallwayWorld b = makeHallwayWorld(123);
    updateObstacleWindow(a, 0.0);
    updateObstacleWindow(b, 0.0);

    assert(near(a.half_width, b.half_width));
    assert(near(a.height, b.height));
    assert(a.obstacles.size() == b.obstacles.size());

    for (size_t i = 0; i < a.obstacles.size(); ++i)
    {
        assert(near(a.obstacles[i].x, b.obstacles[i].x));
        assert((a.obstacles[i].center - b.obstacles[i].center).norm() < 1e-9);
        assert(a.obstacles[i].shape == b.obstacles[i].shape);
        assert(a.obstacles[i].object_id == b.obstacles[i].object_id);
    }
}

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

void testBucketEquivalence()
{
    SimulationState sim = makeSimulation(99);
    int mismatches = verifyAcceleratedQueries(sim);
    assert(mismatches == 0);
}

void testCollision()
{
    HallwayWorld world = makeHallwayWorld(7);
    world.obstacles.clear();

    ActiveObstacle obstacle;
    obstacle.x = 0.0;
    obstacle.center = Vec3(0.0, 0.0, 0.25);
    obstacle.size = 0.5;
    obstacle.object_id = 10;
    obstacle.shape = ObstacleShape::Cube;
    obstacle.geometry = makeCube(obstacle.center, obstacle.size, obstacle.object_id);
    obstacle.bounds = obstacle.geometry->bounds();
    world.obstacles.push_back(obstacle);

    VehicleConfig config;
    VehicleState vehicle;
    assert(checkCollision(world, vehicle, config));

    vehicle.x = 5.0;
    vehicle.y = 0.0;
    assert(!checkCollision(world, vehicle, config));

    vehicle.y = world.half_width;
    assert(checkCollision(world, vehicle, config));
}

int main()
{
    testDeterministicHallway();
    testVehicleDynamics();
    testFollowTheGap();
    testBucketEquivalence();
    testCollision();

    std::cout << "All lidar_3d tests passed\n";
    return 0;
}
