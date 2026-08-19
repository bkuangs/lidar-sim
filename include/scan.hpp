#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <vector>
#include <Eigen/Dense>
#include "lidar.hpp"

using Vec3 = Eigen::Vector3d;

// The follow-the-gap planner needs a per-ray polar range and object identity:
// it slices by elevation, bins by azimuth, and keeps the nearest range per bin.
// The full point cloud (world/sensor coords, normals, intensity) was only ever
// consumed by the retired real-time viewer, so it is not built.
struct ScanPoint
{
    double azimuth;
    double elevation;
    double range;
    int object_id;
    bool hit;
};

struct ScanResult
{
    std::vector<ScanPoint> points;
    int rays_requested = 0;
    int rays_completed = 0;
};

inline std::optional<double> scanRangeForObjectId(
    const ScanResult &scan, int object_id)
{
    std::optional<double> nearest_range;
    for (const ScanPoint &point : scan.points)
    {
        if (point.hit && point.object_id == object_id &&
            (!nearest_range || point.range < *nearest_range))
        {
            nearest_range = point.range;
        }
    }
    return nearest_range;
}

inline bool scanContainsObjectId(const ScanResult &scan, int object_id)
{
    return scanRangeForObjectId(scan, object_id).has_value();
}

inline std::vector<int> progressiveAzimuthOrder(int sample_count)
{
    if (sample_count <= 0)
        return {};

    std::vector<int> order;
    order.reserve(sample_count);
    std::vector<char> selected(sample_count, 0);
    auto add = [&](int index) {
        if (!selected[index])
        {
            selected[index] = 1;
            order.push_back(index);
        }
    };

    add((sample_count - 1) / 2);
    add(0);
    add(sample_count - 1);

    while (static_cast<int>(order.size()) < sample_count)
    {
        int best_left = -1;
        int best_right = -1;
        int previous = -1;
        for (int index = 0; index < sample_count; ++index)
        {
            if (!selected[index])
                continue;
            if (previous >= 0 && index - previous > 1 &&
                (best_left < 0 || index - previous > best_right - best_left))
            {
                best_left = previous;
                best_right = index;
            }
            previous = index;
        }

        add((best_left + best_right) / 2);
    }

    return order;
}

inline constexpr int nestedHorizontalRayLayoutBins = 361;
inline constexpr std::array<int, 9> nestedHorizontalRayLayoutCounts = {
    0, 5, 9, 17, 33, 65, 129, 257, 361};

inline std::vector<double> nestedHorizontalRayLayout(int ray_count, double phase)
{
    if (!std::isfinite(phase) || phase < 0.0 || phase >= 1.0)
        throw std::invalid_argument("phase must be in [0, 1)");
    if (std::find(
            nestedHorizontalRayLayoutCounts.begin(),
            nestedHorizontalRayLayoutCounts.end(),
            ray_count) == nestedHorizontalRayLayoutCounts.end())
        throw std::invalid_argument("unsupported nested horizontal ray count");

    const std::vector<int> order =
        progressiveAzimuthOrder(nestedHorizontalRayLayoutBins);
    std::vector<double> azimuths;
    azimuths.reserve(ray_count);
    for (int rank = 0; rank < ray_count; ++rank)
    {
        const int index = order[rank];
        azimuths.push_back(
            (-90.0 +
             (static_cast<double>(index) + phase) * 180.0 /
                 nestedHorizontalRayLayoutBins) *
            geom::deg);
    }
    return azimuths;
}

inline double lidarElevation(const Lidar &lidar, int sample_index)
{
    const double fraction =
        lidar.elevationSamples > 1
            ? static_cast<double>(sample_index) / (lidar.elevationSamples - 1)
            : 0.0;
    return lidar.minElevation +
           fraction * (lidar.maxElevation - lidar.minElevation);
}

template <typename Intersector>
ScanPoint traceScanPoint(
    const Lidar &lidar,
    double azimuth,
    double elevation,
    Intersector &intersect)
{
    const Vec3 local_direction(
        std::cos(elevation) * std::cos(azimuth),
        std::cos(elevation) * std::sin(azimuth),
        std::sin(elevation));

    Ray ray;
    ray.ori = lidar.pose.translation();
    ray.dir = lidar.pose.rotation() * local_direction.normalized();

    const Hit hit = intersect(ray);
    const bool valid_hit =
        hit.hit && hit.t >= lidar.minRange && hit.t <= lidar.maxRange;
    return ScanPoint{
        azimuth,
        elevation,
        valid_hit ? hit.t : lidar.maxRange,
        valid_hit ? hit.objId : -1,
        valid_hit,
    };
}

template <typename Intersector>
ScanResult scanWithIntersector(const Lidar &lidar, Intersector intersect)
{
    ScanResult scan;
    scan.rays_requested = lidar.azimuthSamples * lidar.elevationSamples;
    scan.points.reserve(scan.rays_requested);

    for (int v = 0; v < lidar.elevationSamples; ++v)
    {
        const double elevation = lidarElevation(lidar, v);

        for (int h = 0; h < lidar.azimuthSamples; ++h)
        {
            const double fraction =
                static_cast<double>(h) / lidar.azimuthSamples;
            const double azimuth =
                lidar.minAzimuth +
                fraction * (lidar.maxAzimuth - lidar.minAzimuth);
            scan.points.push_back(
                traceScanPoint(lidar, azimuth, elevation, intersect));
            ++scan.rays_completed;
        }
    }

    return scan;
}

template <typename Intersector>
ScanResult scanFixedHorizontalWithIntersector(
    const Lidar &lidar,
    const std::vector<double> &azimuths,
    Intersector intersect)
{
    ScanResult scan;
    scan.rays_requested = static_cast<int>(azimuths.size());
    scan.points.reserve(azimuths.size());
    for (const double azimuth : azimuths)
    {
        scan.points.push_back(traceScanPoint(lidar, azimuth, 0.0, intersect));
        ++scan.rays_completed;
    }
    return scan;
}
