#pragma once
#include "geometry.hpp"
#include <algorithm>
#include <cmath>

struct VehicleConfig
{
    double wheelbase = 1.6;
    double length = 1.2;
    double width = 0.8;
    double lidar_height = 1.2;
    double max_speed = 4.0;
    double max_accel = 2.0;
    double max_brake = 4.0;
    double max_steering = 30.0 * geom::deg;
    double max_steering_rate = 90.0 * geom::deg;
};

struct VehicleState
{
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
    double steering = 0.0;
};

inline double clampDelta(double current, double target, double max_delta)
{
    return current + std::clamp(target - current, -max_delta, max_delta);
}

inline void updateVehicle(
    VehicleState &vehicle,
    const VehicleConfig &config,
    double target_speed,
    double target_steering,
    double dt)
{
    target_speed = std::clamp(target_speed, 0.0, config.max_speed);
    target_steering = std::clamp(target_steering, -config.max_steering, config.max_steering);

    const double accel_limit = target_speed >= vehicle.speed ? config.max_accel : config.max_brake;
    vehicle.speed = clampDelta(vehicle.speed, target_speed, accel_limit * dt);
    vehicle.steering = clampDelta(vehicle.steering, target_steering, config.max_steering_rate * dt);

    vehicle.x += vehicle.speed * std::cos(vehicle.heading) * dt;
    vehicle.y += vehicle.speed * std::sin(vehicle.heading) * dt;
    vehicle.heading += vehicle.speed / config.wheelbase * std::tan(vehicle.steering) * dt;
}

inline Eigen::Isometry3d lidarPoseFromVehicle(
    const VehicleState &vehicle,
    const VehicleConfig &config)
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Vec3(vehicle.x, vehicle.y, config.lidar_height);
    pose.linear() = Eigen::AngleAxisd(vehicle.heading, Vec3::UnitZ()).toRotationMatrix();
    return pose;
}

inline bool vehicleIntersectsCircle(
    const VehicleState &vehicle,
    const VehicleConfig &config,
    const Vec3 &center,
    double radius)
{
    const double dx = center.x() - vehicle.x;
    const double dy = center.y() - vehicle.y;
    const double c = std::cos(vehicle.heading);
    const double s = std::sin(vehicle.heading);
    const double local_x = c * dx + s * dy;
    const double local_y = -s * dx + c * dy;
    const double closest_x = std::clamp(
        local_x, -0.5 * config.length, 0.5 * config.length);
    const double closest_y = std::clamp(
        local_y, -0.5 * config.width, 0.5 * config.width);
    const double separation_x = local_x - closest_x;
    const double separation_y = local_y - closest_y;
    return separation_x * separation_x + separation_y * separation_y <=
           radius * radius + geom::Epsilon;
}

inline VehicleState interpolateVehicleState(
    const VehicleState &from,
    const VehicleState &to,
    double t)
{
    VehicleState state;
    state.x = from.x + t * (to.x - from.x);
    state.y = from.y + t * (to.y - from.y);
    const double heading_delta =
        std::remainder(to.heading - from.heading, 2.0 * geom::pi);
    state.heading = from.heading + t * heading_delta;
    state.speed = from.speed + t * (to.speed - from.speed);
    state.steering = from.steering + t * (to.steering - from.steering);
    return state;
}

inline int vehicleMotionSubdivisions(
    const VehicleState &from,
    const VehicleState &to,
    const VehicleConfig &config,
    double tolerance)
{
    const double translation = std::hypot(to.x - from.x, to.y - from.y);
    const double heading_delta =
        std::abs(std::remainder(to.heading - from.heading, 2.0 * geom::pi));
    const double corner_radius =
        std::hypot(0.5 * config.length, 0.5 * config.width);
    const double motion_bound = translation + heading_delta * corner_radius;
    return std::max(
        1,
        static_cast<int>(
            std::ceil(motion_bound / std::max(tolerance, geom::Epsilon))));
}

inline bool sweptVehicleIntersectsCircle(
    const VehicleState &from,
    const VehicleState &to,
    const VehicleConfig &config,
    const Vec3 &center,
    double radius,
    double tolerance)
{
    const double broad_radius =
        std::hypot(0.5 * config.length, 0.5 * config.width) + radius;
    if (center.x() < std::min(from.x, to.x) - broad_radius ||
        center.x() > std::max(from.x, to.x) + broad_radius ||
        center.y() < std::min(from.y, to.y) - broad_radius ||
        center.y() > std::max(from.y, to.y) + broad_radius)
    {
        return false;
    }

    const int subdivisions =
        vehicleMotionSubdivisions(from, to, config, tolerance);
    for (int step = 0; step <= subdivisions; ++step)
    {
        const double t = static_cast<double>(step) / subdivisions;
        if (vehicleIntersectsCircle(
                interpolateVehicleState(from, to, t),
                config,
                center,
                radius))
        {
            return true;
        }
    }
    return false;
}

inline bool vehicleTouchesCorridorWall(
    const VehicleState &vehicle,
    const VehicleConfig &config,
    double corridor_half_width)
{
    const double s = std::abs(std::sin(vehicle.heading));
    const double c = std::abs(std::cos(vehicle.heading));
    const double lateral_extent =
        0.5 * config.length * s + 0.5 * config.width * c;
    return std::abs(vehicle.y) + lateral_extent >=
           corridor_half_width - geom::Epsilon;
}

inline bool sweptVehicleTouchesCorridorWall(
    const VehicleState &from,
    const VehicleState &to,
    const VehicleConfig &config,
    double corridor_half_width,
    double tolerance)
{
    const int subdivisions =
        vehicleMotionSubdivisions(from, to, config, tolerance);
    for (int step = 0; step <= subdivisions; ++step)
    {
        const double t = static_cast<double>(step) / subdivisions;
        if (vehicleTouchesCorridorWall(
                interpolateVehicleState(from, to, t),
                config,
                corridor_half_width))
        {
            return true;
        }
    }
    return false;
}
