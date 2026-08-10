#pragma once
#include "geometry.hpp"
#include "scan.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

struct PlannerConfig
{
    int bins = 181;
    double min_angle = -90.0 * geom::deg;
    double max_angle = 90.0 * geom::deg;
    double slice_elevation = 3.0 * geom::deg;
    double obstacle_clearance = 0.55;
    double min_safe_range = 1.2;
    double slow_range = 3.0;
    double fast_range = 8.0;
    double min_speed = 0.4;
    double max_speed = 3.0;
};

struct PlannerOutput
{
    bool blocked = false;
    double target_angle = 0.0;
    double steering_command = 0.0;
    double speed_command = 0.0;
    double min_forward_range = 0.0;
    int gap_start = -1;
    int gap_end = -1;
};

inline double normalizeAngle(double angle)
{
    while (angle > geom::pi)
        angle -= 2.0 * geom::pi;
    while (angle < -geom::pi)
        angle += 2.0 * geom::pi;
    return angle;
}

inline double binAngle(const PlannerConfig &config, int bin)
{
    if (config.bins <= 1)
        return 0.0;

    const double t = static_cast<double>(bin) / static_cast<double>(config.bins - 1);
    return config.min_angle + t * (config.max_angle - config.min_angle);
}

inline int angleToBin(const PlannerConfig &config, double angle)
{
    const double t = (angle - config.min_angle) / (config.max_angle - config.min_angle);
    return std::clamp(static_cast<int>(std::round(t * (config.bins - 1))), 0, config.bins - 1);
}

inline PlannerOutput planFromRanges(
    const std::vector<double> &ranges,
    const PlannerConfig &config)
{
    PlannerOutput output;
    if (ranges.empty())
    {
        output.blocked = true;
        return output;
    }

    std::vector<bool> blocked(ranges.size(), false);
    for (size_t i = 0; i < ranges.size(); ++i)
    {
        const double range = ranges[i];
        if (range <= config.min_safe_range)
        {
            blocked[i] = true;
            continue;
        }

        if (std::isfinite(range) && range < config.fast_range)
        {
            const double half_angle = std::asin(std::clamp(config.obstacle_clearance / range, 0.0, 1.0));
            const int center = static_cast<int>(i);
            const int radius = std::max(1, angleToBin(config, binAngle(config, center) + half_angle) - center);
            for (int b = std::max(0, center - radius); b <= std::min(static_cast<int>(ranges.size()) - 1, center + radius); ++b)
                blocked[b] = true;
        }
    }

    int best_start = -1;
    int best_end = -1;
    int current_start = -1;
    for (int i = 0; i < static_cast<int>(blocked.size()); ++i)
    {
        if (!blocked[i] && current_start < 0)
            current_start = i;

        const bool at_end = i == static_cast<int>(blocked.size()) - 1;
        if ((blocked[i] || at_end) && current_start >= 0)
        {
            const int current_end = blocked[i] ? i - 1 : i;
            if (best_start < 0 || current_end - current_start > best_end - best_start)
            {
                best_start = current_start;
                best_end = current_end;
            }
            current_start = -1;
        }
    }

    if (best_start < 0)
    {
        output.blocked = true;
        output.speed_command = 0.0;
        output.min_forward_range = 0.0;
        return output;
    }

    output.gap_start = best_start;
    output.gap_end = best_end;
    const int target_bin = (best_start + best_end) / 2;
    output.target_angle = binAngle(config, target_bin);
    output.steering_command = output.target_angle;

    const int forward_window = std::max(1, angleToBin(config, 10.0 * geom::deg) - angleToBin(config, 0.0));
    const int forward_center = angleToBin(config, 0.0);
    output.min_forward_range = std::numeric_limits<double>::infinity();
    for (int i = std::max(0, forward_center - forward_window);
         i <= std::min(static_cast<int>(ranges.size()) - 1, forward_center + forward_window);
         ++i)
    {
        output.min_forward_range = std::min(output.min_forward_range, ranges[i]);
    }

    if (!std::isfinite(output.min_forward_range))
        output.min_forward_range = config.fast_range;

    const double speed_t = std::clamp(
        (output.min_forward_range - config.slow_range) / (config.fast_range - config.slow_range),
        0.0,
        1.0);
    output.speed_command = config.min_speed + speed_t * (config.max_speed - config.min_speed);

    return output;
}

inline PlannerOutput planFollowTheGap(
    const ScanResult &scan,
    const PlannerConfig &config,
    double lidar_max_range)
{
    std::vector<double> ranges(config.bins, lidar_max_range);

    for (const ScanPoint &point : scan.points)
    {
        if (std::abs(point.elevation) > config.slice_elevation)
            continue;

        const double angle = normalizeAngle(point.azimuth);
        if (angle < config.min_angle || angle > config.max_angle)
            continue;

        const int bin = angleToBin(config, angle);
        ranges[bin] = std::min(ranges[bin], point.range);
    }

    return planFromRanges(ranges, config);
}
