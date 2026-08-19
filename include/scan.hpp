#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
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
    bool hit;
};

struct ScanResult
{
    std::vector<ScanPoint> points;
    int rays_requested = 0;
    int rays_attempted = 0;
    int rays_completed = 0;
    double elapsed_ns = 0.0;
    double overrun_ns = 0.0;
    bool deadline_reached = false;
};

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

inline double lidarElevation(const Lidar &lidar, int sample_index)
{
    const double fraction =
        lidar.elevationSamples > 1
            ? static_cast<double>(sample_index) / (lidar.elevationSamples - 1)
            : 0.0;
    return lidar.minElevation +
           fraction * (lidar.maxElevation - lidar.minElevation);
}

inline double progressiveAzimuth(
    const Lidar &lidar,
    int sample_index,
    int grid_samples)
{
    const double fraction =
        grid_samples > 1
            ? static_cast<double>(sample_index) / (grid_samples - 1)
            : 0.5;
    return lidar.minAzimuth +
           fraction * (lidar.maxAzimuth - lidar.minAzimuth);
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

template <typename Intersector, typename Now>
ScanResult scanProgressiveUntilDeadline(
    const Lidar &lidar,
    const std::vector<int> &azimuth_order,
    int grid_samples,
    double budget_ns,
    Intersector intersect,
    Now now)
{
    const auto start = now();
    ScanResult scan;
    if (grid_samples <= 0 || lidar.elevationSamples <= 0)
        return scan;

    const int selected_samples = std::min(
        std::max(0, lidar.azimuthSamples),
        static_cast<int>(azimuth_order.size()));
    scan.rays_requested = selected_samples * lidar.elevationSamples;
    scan.points.reserve(scan.rays_requested);

    if (budget_ns <= 0.0)
    {
        scan.deadline_reached = true;
        return scan;
    }

    using TimePoint = decltype(start);
    using Duration = typename TimePoint::duration;
    const auto deadline =
        start + std::chrono::duration_cast<Duration>(
                    std::chrono::duration<double, std::nano>(budget_ns));

    for (int v = 0; v < lidar.elevationSamples; ++v)
    {
        const double elevation = lidarElevation(lidar, v);

        for (int rank = 0; rank < selected_samples; ++rank)
        {
            const int sample_index = azimuth_order[rank];
            const double azimuth =
                progressiveAzimuth(lidar, sample_index, grid_samples);
            scan.points.push_back(
                traceScanPoint(lidar, azimuth, elevation, intersect));
            ++scan.rays_attempted;

            const auto completed_at = now();
            scan.elapsed_ns =
                std::chrono::duration<double, std::nano>(completed_at - start).count();
            if (completed_at > deadline)
            {
                scan.points.pop_back();
                scan.overrun_ns =
                    std::chrono::duration<double, std::nano>(
                        completed_at - deadline)
                        .count();
                scan.deadline_reached = true;
                return scan;
            }

            ++scan.rays_completed;
            if (completed_at == deadline)
            {
                scan.deadline_reached = true;
                return scan;
            }
        }
    }

    return scan;
}

template <typename Intersector>
ScanResult scanProgressiveWithIntersector(
    const Lidar &lidar,
    const std::vector<int> &azimuth_order,
    int grid_samples,
    Intersector intersect)
{
    ScanResult scan;
    if (grid_samples <= 0 || lidar.elevationSamples <= 0)
        return scan;

    const int selected_samples = std::min(
        std::max(0, lidar.azimuthSamples),
        static_cast<int>(azimuth_order.size()));
    scan.rays_requested = selected_samples * lidar.elevationSamples;
    scan.points.reserve(scan.rays_requested);

    for (int v = 0; v < lidar.elevationSamples; ++v)
    {
        const double elevation = lidarElevation(lidar, v);

        for (int rank = 0; rank < selected_samples; ++rank)
        {
            const int sample_index = azimuth_order[rank];
            const double azimuth =
                progressiveAzimuth(lidar, sample_index, grid_samples);
            scan.points.push_back(
                traceScanPoint(lidar, azimuth, elevation, intersect));
            ++scan.rays_completed;
        }
    }

    return scan;
}
