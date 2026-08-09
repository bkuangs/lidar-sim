#pragma once
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include "lidar.hpp"

using Vec3 = Eigen::Vector3d;

// The follow-the-gap planner only needs a per-ray polar range: it slices by
// elevation, bins by azimuth, and keeps the nearest range per bin. The full
// point cloud (world/sensor coords, normals, intensity, per-object hit counts)
// was only ever consumed by the retired real-time viewer, so it is not built.
struct ScanPoint
{
    double azimuth;
    double elevation;
    double range;
};

struct ScanResult
{
    std::vector<ScanPoint> points;
};

template <typename Intersector>
ScanResult scanWithIntersector(const Lidar &lidar, Intersector intersect)
{
    ScanResult scan;
    scan.points.reserve(lidar.azimuthSamples * lidar.elevationSamples);

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
        double vf = lidar.elevationSamples > 1
                        ? static_cast<double>(v) / (lidar.elevationSamples - 1)
                        : 0.0;
        double elevation =
            lidar.minElevation +
            vf * (lidar.maxElevation - lidar.minElevation);

        for (int h = 0; h < lidar.azimuthSamples; ++h)
        {
            double hf = static_cast<double>(h) / lidar.azimuthSamples;
            double azimuth =
                lidar.minAzimuth +
                hf * (lidar.maxAzimuth - lidar.minAzimuth);

            Vec3 localDir = scanDirection(azimuth, elevation);

            Ray ray;
            ray.ori = lidar.pose.translation();
            ray.dir = lidar.pose.rotation() * localDir;

            Hit hit = intersect(ray);

            if (hit.hit && hit.t >= lidar.minRange && hit.t <= lidar.maxRange)
                scan.points.push_back(ScanPoint{azimuth, elevation, hit.t});
        }
    }

    return scan;
}
