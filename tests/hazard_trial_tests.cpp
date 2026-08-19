#include "hazard_trial.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

void testDeterministicValidGeneration()
{
    const std::vector<HazardScenario> first = generateHazardScenarios();
    const std::vector<HazardScenario> second = generateHazardScenarios();
    assert(first.size() == 3u * 3u * 5u * 32u);
    assert(first.size() == second.size());
    for (size_t i = 0; i < first.size(); ++i)
    {
        validateHazardScenario(first[i]);
        assert(first[i].scenario_id == second[i].scenario_id);
        assert(first[i].initial_clearance == second[i].initial_clearance);
        assert(first[i].hazard.diameter == second[i].hazard.diameter);
        assert(first[i].lateral_offset == second[i].lateral_offset);
        assert(first[i].azimuth_phase == second[i].azimuth_phase);
    }
}

void testZeroDetectionCollides()
{
    for (const HazardScenario &scenario : generateHazardScenarios())
    {
        const HazardResult result = runHazardTrial(
            scenario, [](const HazardScenario &, const HazardFrame &) {
                return HazardDetection{};
            });
        assert(result.outcome == HazardOutcome::Collision);
        assert(!result.detected);
    }
}

void testFirstFrameDetectionSafelyStops()
{
    for (const HazardScenario &scenario : generateHazardScenarios())
    {
        const HazardResult result = runHazardTrial(
            scenario, [](const HazardScenario &, const HazardFrame &frame) {
                return HazardDetection{
                    frame.index == 0,
                    frame.index == 0 ? frame.hazard_range : -1.0};
            });
        assert(result.outcome == HazardOutcome::SafeStop);
        assert(result.detected);
        assert(result.stopping_margin >= 0.0);
    }
}

void testLateDetectionCollides()
{
    const HazardScenario scenario = generateHazardScenarios().front();
    const HazardResult result = runHazardTrial(
        scenario, [](const HazardScenario &, const HazardFrame &frame) {
            return HazardDetection{
                frame.index == 10,
                frame.index == 10 ? frame.hazard_range : -1.0};
        });
    assert(result.detected);
    assert(result.stopping_margin < 0.0);
    assert(result.outcome == HazardOutcome::Collision);
    assert(result.collision_speed > 0.0);
}

void testDetectionRangeComesFromDetector()
{
    const HazardScenario scenario = generateHazardScenarios().back();
    const HazardResult result = runHazardTrial(
        scenario, [](const HazardScenario &, const HazardFrame &frame) {
            return HazardDetection{
                frame.index == 0, frame.index == 0 ? 7.25 : -1.0};
        });
    assert(result.detected);
    assert(std::abs(result.detection_range - 7.25) <= geom::Epsilon);
}

void testSmallestHazardCannotTunnel()
{
    VehicleConfig vehicle_config;
    VehicleState vehicle;
    const HazardSweep sweep = sweptStraightVehicleCircle(
        vehicle,
        2.0,
        vehicle_config,
        Vec3(1.0, 0.0, 1.0),
        0.05);
    assert(sweep.contact);
    assert(sweep.distance_to_contact > 0.0);
    assert(sweep.distance_to_contact < 2.0);
}

int main()
{
    testDeterministicValidGeneration();
    testZeroDetectionCollides();
    testFirstFrameDetectionSafelyStops();
    testLateDetectionCollides();
    testDetectionRangeComesFromDetector();
    testSmallestHazardCannotTunnel();
    std::cout << "Hazard trial tests passed\n";
    return 0;
}
