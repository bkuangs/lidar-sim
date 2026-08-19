#include "hazard_trial.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <set>
#include <tuple>

namespace
{
using CommonTuple = std::tuple<double, double, double, double>;

HazardDetection firstFrameDetection(
    const HazardScenario &, const HazardFrame &frame)
{
    return HazardDetection{
        frame.index == 0,
        frame.index == 0 ? frame.hazard_range : -1.0};
}

void testExactControlsAndDeterministicCoverage()
{
    assert(hazardSpeedSafetySpeeds ==
           (std::array<double, 3>{{2.0, 3.0, 4.0}}));
    assert(hazardSpeedSafetyClearances ==
           (std::array<double, 2>{{4.0, 8.0}}));
    assert(hazardBraking == 4.0);
    assert(hazardFramePeriod == 1.0 / 30.0);
    assert(hazardPhaseCount == 32);
    assert(hazardDiameters ==
           (std::array<double, 3>{{0.10, 0.25, 0.50}}));
    assert(hazardOffsets ==
           (std::array<double, 5>{{-0.30, -0.15, 0.0, 0.15, 0.30}}));

    const std::vector<HazardScenario> first = generateHazardSpeedScenarios();
    const std::vector<HazardScenario> second = generateHazardSpeedScenarios();
    assert(first.size() == 2880);
    assert(first.size() == second.size());

    std::set<unsigned> ids;
    std::array<std::set<CommonTuple>, 3> tuples_by_speed;
    std::array<size_t, 3> counts = {{0, 0, 0}};
    for (size_t index = 0; index < first.size(); ++index)
    {
        const HazardScenario &scenario = first[index];
        const HazardScenario &repeated = second[index];
        validateHazardSpeedScenario(scenario);
        assert(scenario.scenario_id == index);
        assert(scenario.scenario_id == repeated.scenario_id);
        assert(scenario.speed == repeated.speed);
        assert(scenario.initial_clearance == repeated.initial_clearance);
        assert(scenario.hazard.diameter == repeated.hazard.diameter);
        assert(scenario.lateral_offset == repeated.lateral_offset);
        assert(scenario.azimuth_phase == repeated.azimuth_phase);
        assert(ids.insert(scenario.scenario_id).second);

        const unsigned speed_index = hazardValueIndex(
            scenario.speed, hazardSpeedSafetySpeeds.data(),
            hazardSpeedSafetySpeeds.size(), "speed");
        tuples_by_speed[speed_index].emplace(
            scenario.initial_clearance,
            scenario.hazard.diameter,
            scenario.lateral_offset,
            scenario.azimuth_phase);
        ++counts[speed_index];
    }
    assert(ids.size() == 2880);
    for (unsigned speed = 0; speed < counts.size(); ++speed)
    {
        assert(counts[speed] == 960);
        assert(tuples_by_speed[speed].size() == 960);
        assert(tuples_by_speed[speed] == tuples_by_speed[0]);
    }
}

void testEveryGeneratedScenarioMeetsPhysicalControls()
{
    for (const HazardScenario &scenario : generateHazardSpeedScenarios())
    {
        const HazardResult undetected = runHazardSpeedTrial(
            scenario, [](const HazardScenario &, const HazardFrame &) {
                return HazardDetection{};
            });
        assert(undetected.outcome == HazardOutcome::Collision);
        assert(!undetected.detected);

        const HazardResult first_frame =
            runHazardSpeedTrial(scenario, firstFrameDetection);
        assert(first_frame.outcome == HazardOutcome::SafeStop);
        assert(first_frame.detected);
        assert(first_frame.stopping_margin > geom::Epsilon);
    }
}

void testFourMetersPerSecondAtTwoMetersIsInvalid()
{
    HazardScenario excluded;
    excluded.speed = 4.0;
    excluded.initial_clearance = 2.0;
    excluded.hazard.diameter = 0.10;

    bool strict_control_rejected = false;
    try
    {
        validateHazardScenarioPhysics(excluded, VehicleConfig{}, true);
    }
    catch (const std::invalid_argument &)
    {
        strict_control_rejected = true;
    }
    assert(strict_control_rejected);

    const HazardResult first_frame = runValidatedHazardTrial(
        excluded, firstFrameDetection, VehicleConfig{});
    assert(first_frame.outcome == HazardOutcome::Collision);
    assert(first_frame.detected);
    assert(std::abs(first_frame.stopping_margin) <= geom::Epsilon);

    bool common_domain_rejected = false;
    try
    {
        validateHazardSpeedScenario(excluded);
    }
    catch (const std::invalid_argument &)
    {
        common_domain_rejected = true;
    }
    assert(common_domain_rejected);
}

void testSpeedAwareIdValidation()
{
    HazardScenario scenario = generateHazardSpeedScenarios().front();
    ++scenario.scenario_id;
    bool rejected = false;
    try
    {
        validateHazardSpeedScenario(scenario);
    }
    catch (const std::invalid_argument &)
    {
        rejected = true;
    }
    assert(rejected);

    const std::vector<HazardScenario> base = generateHazardScenarios();
    assert(base.size() == 1440);
    assert(base.front().speed == 3.0);
    assert(base.front().scenario_id == 0);
    assert(base.back().scenario_id == 1439);
}
} // namespace

int main()
{
    testExactControlsAndDeterministicCoverage();
    testEveryGeneratedScenarioMeetsPhysicalControls();
    testFourMetersPerSecondAtTwoMetersIsInvalid();
    testSpeedAwareIdValidation();
    std::cout << "Hazard speed trial tests passed\n";
    return 0;
}
