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

inline AABB vehicleFootprintBounds(
    const VehicleState &vehicle,
    const VehicleConfig &config)
{
    AABB bbox;
    bbox.setEmpty();

    const double c = std::cos(vehicle.heading);
    const double s = std::sin(vehicle.heading);
    const Vec3 forward(c, s, 0.0);
    const Vec3 right(-s, c, 0.0);
    const Vec3 center(vehicle.x, vehicle.y, 0.0);

    for (double x_sign : {-1.0, 1.0})
    {
        for (double y_sign : {-1.0, 1.0})
        {
            const Vec3 corner =
                center +
                x_sign * 0.5 * config.length * forward +
                y_sign * 0.5 * config.width * right;
            bbox.extend(corner);
        }
    }

    bbox.extend(Vec3(vehicle.x, vehicle.y, config.lidar_height));
    return bbox;
}

inline bool intersectsAABB(const AABB &a, const AABB &b)
{
    return (a.min().x() <= b.max().x() && a.max().x() >= b.min().x()) &&
           (a.min().y() <= b.max().y() && a.max().y() >= b.min().y()) &&
           (a.min().z() <= b.max().z() && a.max().z() >= b.min().z());
}
