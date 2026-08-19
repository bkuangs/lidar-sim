#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace hazard_timing_scene
{
constexpr int baseObjectCount = 100;
constexpr int maxObjectCount = 400;
constexpr int slices = 12;
constexpr int stacks = 4;
constexpr int trianglesPerObject = 120;
constexpr double objectHeight = 2.0;
constexpr std::array<int, 5> objectCounts = {{25, 50, 100, 200, 400}};

struct ObjectSpec
{
    double x;
    double y;
    double radius;
    double height;
    int object_id;
};

inline std::vector<ObjectSpec> makePr2ObjectSpecs()
{
    std::vector<ObjectSpec> specs;
    specs.reserve(baseObjectCount);
    for (int index = 0; index < baseObjectCount; ++index)
    {
        const int column = index % 10;
        const int row = index / 10;
        specs.push_back(ObjectSpec{
            2.5 + 2.0 * column + 0.13 * (row % 3),
            -4.5 + row + 0.07 * (column % 4),
            0.18 + 0.02 * (index % 5),
            objectHeight,
            1000 + index,
        });
    }
    return specs;
}

inline double radicalInverse(unsigned value, unsigned base)
{
    double result = 0.0;
    double denominator = 1.0;
    while (value > 0)
    {
        denominator *= base;
        result += static_cast<double>(value % base) / denominator;
        value /= base;
    }
    return result;
}

inline std::vector<ObjectSpec> makeObjectCountSpecs()
{
    const std::vector<ObjectSpec> base = makePr2ObjectSpecs();
    constexpr std::array<std::array<int, 2>, 4> localOrder = {{
        {{0, 0}},
        {{1, 1}},
        {{0, 1}},
        {{1, 0}},
    }};

    std::vector<ObjectSpec> specs;
    specs.reserve(maxObjectCount);
    for (const auto &local : localOrder)
    {
        for (int block_row = 0; block_row < 5; ++block_row)
        {
            for (int block_column = 0; block_column < 5; ++block_column)
            {
                const int row = 2 * block_row + local[0];
                const int column = 2 * block_column + local[1];
                specs.push_back(base[row * 10 + column]);
            }
        }
    }

    double min_x = specs.front().x - specs.front().radius;
    double max_x = specs.front().x + specs.front().radius;
    double min_y = specs.front().y - specs.front().radius;
    double max_y = specs.front().y + specs.front().radius;
    for (const ObjectSpec &spec : specs)
    {
        min_x = std::min(min_x, spec.x - spec.radius);
        max_x = std::max(max_x, spec.x + spec.radius);
        min_y = std::min(min_y, spec.y - spec.radius);
        max_y = std::max(max_y, spec.y + spec.radius);
    }

    constexpr double spacing = 0.005;
    constexpr unsigned maxCandidates = 100000;
    for (unsigned candidate = 1;
         candidate <= maxCandidates && specs.size() < maxObjectCount;
         ++candidate)
    {
        const int index = static_cast<int>(specs.size());
        const double radius = 0.18 + 0.02 * (index % 5);
        const double x =
            min_x + radius +
            (max_x - min_x - 2.0 * radius) * radicalInverse(candidate, 2);
        const double y =
            min_y + radius +
            (max_y - min_y - 2.0 * radius) * radicalInverse(candidate, 3);

        bool separated = true;
        for (const ObjectSpec &existing : specs)
        {
            const double required = radius + existing.radius + spacing;
            const double dx = x - existing.x;
            const double dy = y - existing.y;
            if (dx * dx + dy * dy < required * required)
            {
                separated = false;
                break;
            }
        }
        if (separated)
        {
            specs.push_back(ObjectSpec{
                x, y, radius, objectHeight, 1000 + index});
        }
    }

    if (specs.size() != maxObjectCount)
        throw std::runtime_error("could not construct the fixed-domain timing scene");
    return specs;
}

inline std::vector<ObjectSpec> makeObjectCountSpecs(int object_count)
{
    bool supported = false;
    for (const int expected : objectCounts)
        supported = supported || object_count == expected;
    if (!supported)
        throw std::invalid_argument("unsupported timing object count");

    std::vector<ObjectSpec> specs = makeObjectCountSpecs();
    specs.resize(static_cast<size_t>(object_count));
    return specs;
}
} // namespace hazard_timing_scene
