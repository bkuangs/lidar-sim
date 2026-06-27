#pragma once
#include <vector>
#include <map>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include "scene.hpp"
#include "lidar.hpp"

using Vec3 = Eigen::Vector3d;

struct ScanPoint
{
    Vec3 point_world;
    Vec3 point_sensor;
    Vec3 normal_world;
    double range;
    double intensity;
    double azimuth;
    double elevation;
    int object_id;
};

struct ScanResult
{
    std::vector<ScanPoint> points;
    int total_rays = 0;
    std::map<int, int> hit_counts;
};

template <typename Intersector>
ScanResult scanWithIntersector(const Lidar &lidar, Intersector intersect)
{
    ScanResult scan;
    scan.points.reserve(lidar.azimuthSamples * lidar.elevationSamples);

    // convert scan angles to local ray direction
    auto scanDirection = [](double azimuth, double elevation)
    {
        return Vec3(
                   std::cos(elevation) * std::cos(azimuth),
                   std::cos(elevation) * std::sin(azimuth),
                   std::sin(elevation))
            .normalized();
    };

    for (int v = 0; v < lidar.elevationSamples; ++v)
    {
        // scan progress
        double vf = static_cast<double>(v) / (lidar.elevationSamples - 1);

        // elevation angle
        double elevation =
            lidar.minElevation +
            vf * (lidar.maxElevation - lidar.minElevation);

        for (int h = 0; h < lidar.azimuthSamples; ++h)
        {
            ++scan.total_rays;

            double hf = static_cast<double>(h) / lidar.azimuthSamples;

            // horizontal angle
            double azimuth =
                lidar.minAzimuth +
                hf * (lidar.maxAzimuth - lidar.minAzimuth);

            Vec3 localDir = scanDirection(azimuth, elevation);

            Ray ray;
            ray.ori = lidar.pose.translation();         // ray starts at lidar pose
            ray.dir = lidar.pose.rotation() * localDir; // convert local ray direction to world frame

            Hit hit = intersect(ray);

            if (hit.hit && hit.t >= lidar.minRange && hit.t <= lidar.maxRange)
            {
                Vec3 point_world = ray.ori + hit.t * ray.dir;
                Vec3 point_sensor = hit.t * localDir;
                double intensity = std::clamp(hit.n.dot(-ray.dir), 0.0, 1.0);

                ScanPoint point{
                    point_world,
                    point_sensor,
                    hit.n,
                    hit.t,
                    intensity,
                    azimuth,
                    elevation,
                    hit.objId};

                scan.points.push_back(point);
                ++scan.hit_counts[hit.objId];
            }
        }
    }

    return scan;
}

inline ScanResult scanScene(const Scene &scene, const Lidar &lidar)
{
    return scanWithIntersector(lidar, [&scene](const Ray &ray)
                               { return scene.intersect(ray); });
}
