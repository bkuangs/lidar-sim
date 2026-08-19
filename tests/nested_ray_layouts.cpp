#include "scan.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
bool near(double a, double b, double eps = 1e-12)
{
    return std::abs(a - b) <= eps;
}

void testNestedHorizontalRayLayouts()
{
    constexpr double phase = 0.25;
    const std::vector<double> full = nestedHorizontalRayLayout(361, phase);
    const std::vector<int> order =
        progressiveAzimuthOrder(nestedHorizontalRayLayoutBins);
    for (int rank = 0; rank < nestedHorizontalRayLayoutBins; ++rank)
    {
        const double expected =
            (-90.0 + (order[rank] + phase) * 180.0 / 361.0) * geom::deg;
        assert(near(full[rank], expected));
    }

    for (const int ray_count : nestedHorizontalRayLayoutCounts)
    {
        const std::vector<double> layout =
            nestedHorizontalRayLayout(ray_count, phase);
        assert(static_cast<int>(layout.size()) == ray_count);
        assert(layout == nestedHorizontalRayLayout(ray_count, phase));
        assert(std::equal(layout.begin(), layout.end(), full.begin()));

        std::vector<double> unique = layout;
        std::sort(unique.begin(), unique.end());
        assert(std::adjacent_find(unique.begin(), unique.end()) == unique.end());
    }

    const std::vector<double> zero = nestedHorizontalRayLayout(0, phase);
    assert(zero.empty());

    const std::vector<double> unshifted = nestedHorizontalRayLayout(361, 0.0);
    assert(near(
        *std::min_element(unshifted.begin(), unshifted.end()),
        -90.0 * geom::deg));
    assert(near(
        *std::max_element(unshifted.begin(), unshifted.end()),
        (-90.0 + 360.0 * 180.0 / 361.0) * geom::deg));
    for (const double azimuth : unshifted)
    {
        assert(azimuth >= -90.0 * geom::deg);
        assert(azimuth < 90.0 * geom::deg);
    }

    const std::vector<double> shifted = nestedHorizontalRayLayout(361, 0.5);
    for (const double azimuth : shifted)
    {
        assert(azimuth > -90.0 * geom::deg);
        assert(azimuth < 90.0 * geom::deg);
    }
}

void testInvalidNestedHorizontalRayLayoutInputs()
{
    for (const double phase : {
             -0.01, 1.0, std::nan(""),
             std::numeric_limits<double>::infinity()})
    {
        bool rejected = false;
        try
        {
            (void)nestedHorizontalRayLayout(5, phase);
        }
        catch (const std::invalid_argument &)
        {
            rejected = true;
        }
        assert(rejected);
    }

    for (const int ray_count : {-1, 1, 4, 6, 362})
    {
        bool rejected = false;
        try
        {
            (void)nestedHorizontalRayLayout(ray_count, 0.0);
        }
        catch (const std::invalid_argument &)
        {
            rejected = true;
        }
        assert(rejected);
    }
}
} // namespace

int main()
{
    testNestedHorizontalRayLayouts();
    testInvalidNestedHorizontalRayLayoutInputs();
    return 0;
}
