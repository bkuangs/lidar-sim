#pragma once

#include "geometry.hpp"
#include "vehicle.hpp"

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

enum class HazardOutcome
{
    SafeStop,
    Collision
};

struct HazardSpec
{
    double diameter = 0.10;
    double height = 2.0;
    int object_id = 1;
};

struct HazardScenario
{
    unsigned scenario_id = 0;
    double speed = 3.0;
    double initial_clearance = 2.0;
    double lateral_offset = 0.0;
    double azimuth_phase = 0.0;
    HazardSpec hazard;
};

struct HazardResult
{
    HazardOutcome outcome = HazardOutcome::Collision;
    bool detected = false;
    double detection_range = -1.0;
    double unbraked_ttc = -1.0;
    double stopping_margin = -1.0;
    double collision_speed = 0.0;
};

struct HazardFrame
{
    unsigned index = 0;
    VehicleState vehicle;
    double hazard_range = -1.0;
};

struct HazardDetection
{
    bool detected = false;
    double range = -1.0;
};

using HazardDetector =
    std::function<HazardDetection(const HazardScenario &, const HazardFrame &)>;

inline constexpr double hazardFramePeriod = 1.0 / 30.0;
inline constexpr double hazardBraking = 4.0;
inline constexpr double hazardMaxRange = 20.0;
inline constexpr unsigned hazardPhaseCount = 32;
inline constexpr unsigned hazardFrameLimit = 256;

inline constexpr std::array<double, 3> hazardClearances = {2.0, 4.0, 8.0};
inline constexpr std::array<double, 3> hazardDiameters = {0.10, 0.25, 0.50};
inline constexpr std::array<double, 5> hazardOffsets = {
    -0.30, -0.15, 0.0, 0.15, 0.30};
inline constexpr std::array<double, 3> hazardSpeedSafetySpeeds = {2.0, 3.0, 4.0};
inline constexpr std::array<double, 2> hazardSpeedSafetyClearances = {4.0, 8.0};

inline unsigned hazardScenarioId(
    unsigned clearance_index,
    unsigned diameter_index,
    unsigned offset_index,
    unsigned phase_index)
{
    return (((clearance_index * hazardDiameters.size() + diameter_index) *
             hazardOffsets.size() +
             offset_index) *
            hazardPhaseCount +
            phase_index);
}

inline unsigned hazardSpeedScenarioId(
    unsigned speed_index,
    unsigned clearance_index,
    unsigned diameter_index,
    unsigned offset_index,
    unsigned phase_index)
{
    return (((((speed_index * hazardSpeedSafetyClearances.size() +
                clearance_index) *
                   hazardDiameters.size() +
               diameter_index) *
                  hazardOffsets.size() +
              offset_index) *
                 hazardPhaseCount +
             phase_index));
}

inline Vec3 hazardCenter(
    const HazardScenario &scenario, const VehicleConfig &vehicle_config)
{
    const double radius = 0.5 * scenario.hazard.diameter;
    return Vec3(
        0.5 * vehicle_config.length + radius + scenario.initial_clearance,
        scenario.lateral_offset,
        0.5 * scenario.hazard.height);
}

inline double hazardRangeAtVehicle(
    const HazardScenario &scenario,
    const VehicleState &vehicle,
    const VehicleConfig &vehicle_config)
{
    const Vec3 center = hazardCenter(scenario, vehicle_config);
    return std::max(
        0.0,
        std::hypot(center.x() - vehicle.x, center.y() - vehicle.y) -
            0.5 * scenario.hazard.diameter);
}

struct HazardSweep
{
    bool contact = false;
    double distance_to_contact = -1.0;
};

// Exact circle-vs-rectangle sweep for this benchmark's straight, unsteered motion.
inline HazardSweep sweptStraightVehicleCircle(
    const VehicleState &from,
    double travel,
    const VehicleConfig &vehicle_config,
    const Vec3 &circle_center,
    double circle_radius)
{
    if (travel < -geom::Epsilon || circle_radius <= 0.0)
        throw std::invalid_argument("hazard sweep requires positive travel and radius");
    if (std::abs(from.heading) > geom::Epsilon)
        throw std::invalid_argument("hazard sweep requires a straight vehicle heading");

    const double lateral_separation = std::max(
        0.0, std::abs(circle_center.y() - from.y) - 0.5 * vehicle_config.width);
    if (lateral_separation > circle_radius + geom::Epsilon)
        return {};

    const double forward_reach = std::sqrt(std::max(
        0.0,
        circle_radius * circle_radius -
            lateral_separation * lateral_separation));
    const double contact_front_x = circle_center.x() - forward_reach;
    const double front_x = from.x + 0.5 * vehicle_config.length;
    const double distance_to_contact = contact_front_x - front_x;

    if (distance_to_contact <= geom::Epsilon)
        return {true, 0.0};
    if (distance_to_contact <= travel + geom::Epsilon)
        return {true, distance_to_contact};
    return {};
}

inline double straightLineDistanceToHazardContact(
    const HazardScenario &scenario,
    const VehicleState &vehicle,
    const VehicleConfig &vehicle_config)
{
    const HazardSweep sweep = sweptStraightVehicleCircle(
        vehicle,
        std::numeric_limits<double>::infinity(),
        vehicle_config,
        hazardCenter(scenario, vehicle_config),
        0.5 * scenario.hazard.diameter);
    return sweep.contact ? sweep.distance_to_contact : -1.0;
}

inline unsigned hazardValueIndex(
    double value, const double *values, size_t count, const char *name)
{
    for (unsigned index = 0; index < count; ++index)
        if (std::abs(value - values[index]) <= geom::Epsilon)
            return index;
    throw std::invalid_argument(
        std::string("hazard scenario has unsupported ") + name);
}

inline void validateHazardScenarioPhysics(
    const HazardScenario &scenario,
    const VehicleConfig &vehicle_config,
    bool contact_at_stopping_endpoint_is_invalid)
{
    if (!std::isfinite(scenario.speed) ||
        !std::isfinite(scenario.initial_clearance) ||
        !std::isfinite(scenario.lateral_offset) ||
        !std::isfinite(scenario.azimuth_phase) ||
        !std::isfinite(scenario.hazard.diameter) ||
        !std::isfinite(scenario.hazard.height) ||
        !std::isfinite(vehicle_config.length) ||
        !std::isfinite(vehicle_config.width) ||
        !std::isfinite(vehicle_config.max_brake) ||
        vehicle_config.length <= 0.0 || vehicle_config.width <= 0.0 ||
        vehicle_config.max_brake <= 0.0)
        throw std::invalid_argument("hazard scenario requires a positive vehicle footprint and brake");
    if (std::abs(scenario.hazard.height - 2.0) > geom::Epsilon ||
        scenario.hazard.object_id != 1)
        throw std::invalid_argument("hazard scenario has unsupported benchmark parameters");

    const double scaled_phase = scenario.azimuth_phase * hazardPhaseCount;
    const unsigned phase_index = static_cast<unsigned>(std::llround(scaled_phase));
    if (phase_index >= hazardPhaseCount ||
        std::abs(scaled_phase - phase_index) > geom::Epsilon)
        throw std::invalid_argument("hazard scenario has an invalid azimuth phase");

    VehicleState initial_vehicle;
    const Vec3 center = hazardCenter(scenario, vehicle_config);
    const double radius = 0.5 * scenario.hazard.diameter;
    if (sweptStraightVehicleCircle(
            initial_vehicle, 0.0, vehicle_config, center, radius)
            .contact)
        throw std::invalid_argument("hazard scenario starts in collision");

    const double center_range = std::hypot(center.x(), center.y());
    const double azimuth = std::atan2(center.y(), center.x());
    if (center_range - radius > hazardMaxRange + geom::Epsilon ||
        azimuth < -90.0 * geom::deg - geom::Epsilon ||
        azimuth >= 90.0 * geom::deg)
        throw std::invalid_argument("hazard scenario is outside LiDAR range or FOV");

    const double contact_distance =
        straightLineDistanceToHazardContact(scenario, initial_vehicle, vehicle_config);
    if (contact_distance < 0.0)
        throw std::invalid_argument("unbraked hazard scenario cannot collide");
    const double stopping_distance =
        scenario.speed * scenario.speed / (2.0 * hazardBraking);
    if (contact_at_stopping_endpoint_is_invalid
            ? contact_distance <= stopping_distance + geom::Epsilon
            : contact_distance + geom::Epsilon < stopping_distance)
        throw std::invalid_argument("first-frame braking cannot safely stop");
}

inline void validateHazardScenario(
    const HazardScenario &scenario,
    const VehicleConfig &vehicle_config = VehicleConfig{})
{
    if (std::abs(scenario.speed - 3.0) > geom::Epsilon)
        throw std::invalid_argument("hazard scenario has unsupported benchmark parameters");
    const unsigned clearance_index = hazardValueIndex(
        scenario.initial_clearance, hazardClearances.data(),
        hazardClearances.size(), "clearance");
    const unsigned diameter_index = hazardValueIndex(
        scenario.hazard.diameter, hazardDiameters.data(),
        hazardDiameters.size(), "diameter");
    const unsigned offset_index = hazardValueIndex(
        scenario.lateral_offset, hazardOffsets.data(),
        hazardOffsets.size(), "lateral offset");
    validateHazardScenarioPhysics(scenario, vehicle_config, false);
    const unsigned phase_index = static_cast<unsigned>(
        std::llround(scenario.azimuth_phase * hazardPhaseCount));
    if (scenario.scenario_id != hazardScenarioId(
                                    clearance_index,
                                    diameter_index,
                                    offset_index,
                                    phase_index))
        throw std::invalid_argument(
            "hazard scenario ID does not identify its parameter tuple");
}

inline void validateHazardSpeedScenario(
    const HazardScenario &scenario,
    const VehicleConfig &vehicle_config = VehicleConfig{})
{
    const unsigned speed_index = hazardValueIndex(
        scenario.speed, hazardSpeedSafetySpeeds.data(),
        hazardSpeedSafetySpeeds.size(), "speed");
    const unsigned clearance_index = hazardValueIndex(
        scenario.initial_clearance, hazardSpeedSafetyClearances.data(),
        hazardSpeedSafetyClearances.size(), "speed-safety clearance");
    const unsigned diameter_index = hazardValueIndex(
        scenario.hazard.diameter, hazardDiameters.data(),
        hazardDiameters.size(), "diameter");
    const unsigned offset_index = hazardValueIndex(
        scenario.lateral_offset, hazardOffsets.data(),
        hazardOffsets.size(), "lateral offset");
    validateHazardScenarioPhysics(scenario, vehicle_config, true);
    const unsigned phase_index = static_cast<unsigned>(
        std::llround(scenario.azimuth_phase * hazardPhaseCount));
    if (scenario.scenario_id != hazardSpeedScenarioId(
                                    speed_index,
                                    clearance_index,
                                    diameter_index,
                                    offset_index,
                                    phase_index))
        throw std::invalid_argument(
            "speed-safety scenario ID does not identify its parameter tuple");
}

inline std::vector<HazardScenario> generateHazardScenarios()
{
    std::vector<HazardScenario> scenarios;
    scenarios.reserve(
        hazardClearances.size() * hazardDiameters.size() *
        hazardOffsets.size() * hazardPhaseCount);
    for (unsigned clearance = 0; clearance < hazardClearances.size(); ++clearance)
        for (unsigned diameter = 0; diameter < hazardDiameters.size(); ++diameter)
            for (unsigned offset = 0; offset < hazardOffsets.size(); ++offset)
                for (unsigned phase = 0; phase < hazardPhaseCount; ++phase)
                {
                    HazardScenario scenario;
                    scenario.scenario_id =
                        hazardScenarioId(clearance, diameter, offset, phase);
                    scenario.initial_clearance = hazardClearances[clearance];
                    scenario.lateral_offset = hazardOffsets[offset];
                    scenario.azimuth_phase =
                        static_cast<double>(phase) / hazardPhaseCount;
                    scenario.hazard.diameter = hazardDiameters[diameter];
                    validateHazardScenario(scenario);
                    scenarios.push_back(scenario);
                }
    return scenarios;
}

inline std::vector<HazardScenario> generateHazardSpeedScenarios()
{
    std::vector<HazardScenario> scenarios;
    scenarios.reserve(
        hazardSpeedSafetySpeeds.size() *
        hazardSpeedSafetyClearances.size() *
        hazardDiameters.size() * hazardOffsets.size() * hazardPhaseCount);
    for (unsigned speed = 0; speed < hazardSpeedSafetySpeeds.size(); ++speed)
        for (unsigned clearance = 0;
             clearance < hazardSpeedSafetyClearances.size(); ++clearance)
            for (unsigned diameter = 0; diameter < hazardDiameters.size(); ++diameter)
                for (unsigned offset = 0; offset < hazardOffsets.size(); ++offset)
                    for (unsigned phase = 0; phase < hazardPhaseCount; ++phase)
                    {
                        HazardScenario scenario;
                        scenario.scenario_id = hazardSpeedScenarioId(
                            speed, clearance, diameter, offset, phase);
                        scenario.speed = hazardSpeedSafetySpeeds[speed];
                        scenario.initial_clearance =
                            hazardSpeedSafetyClearances[clearance];
                        scenario.lateral_offset = hazardOffsets[offset];
                        scenario.azimuth_phase =
                            static_cast<double>(phase) / hazardPhaseCount;
                        scenario.hazard.diameter = hazardDiameters[diameter];
                        validateHazardSpeedScenario(scenario);
                        scenarios.push_back(scenario);
                    }
    return scenarios;
}

inline HazardResult runValidatedHazardTrial(
    const HazardScenario &scenario,
    const HazardDetector &detector,
    const VehicleConfig &vehicle_config,
    unsigned frame_limit = hazardFrameLimit)
{
    if (!detector)
        throw std::invalid_argument("hazard trial requires a detector");
    if (frame_limit == 0)
        throw std::invalid_argument("hazard trial requires a positive frame limit");

    VehicleState vehicle;
    vehicle.speed = scenario.speed;
    HazardResult result;

    for (unsigned frame = 0; frame < frame_limit; ++frame)
    {
        const HazardFrame scan_frame = {
            frame,
            vehicle,
            hazardRangeAtVehicle(scenario, vehicle, vehicle_config)};
        const HazardDetection detection = detector(scenario, scan_frame);
        if (!result.detected && detection.detected)
        {
            if (!std::isfinite(detection.range) || detection.range < 0.0)
                throw std::invalid_argument(
                    "hazard detector returned an invalid detection range");
            result.detected = true;
            result.detection_range = detection.range;
            const double contact_distance = straightLineDistanceToHazardContact(
                scenario, vehicle, vehicle_config);
            result.unbraked_ttc = contact_distance / vehicle.speed;
            result.stopping_margin =
                contact_distance - vehicle.speed * vehicle.speed / (2.0 * hazardBraking);
        }

        const double braking = result.detected ? hazardBraking : 0.0;
        const double stopping_time = braking > 0.0 ? vehicle.speed / braking : hazardFramePeriod;
        const double motion_time = std::min(hazardFramePeriod, stopping_time);
        const double travel =
            vehicle.speed * motion_time - 0.5 * braking * motion_time * motion_time;
        const HazardSweep sweep = sweptStraightVehicleCircle(
            vehicle,
            travel,
            vehicle_config,
            hazardCenter(scenario, vehicle_config),
            0.5 * scenario.hazard.diameter);
        if (sweep.contact)
        {
            result.outcome = HazardOutcome::Collision;
            result.collision_speed = std::sqrt(std::max(
                0.0,
                vehicle.speed * vehicle.speed -
                    2.0 * braking * sweep.distance_to_contact));
            return result;
        }

        vehicle.x += travel;
        vehicle.speed = std::max(0.0, vehicle.speed - braking * motion_time);
        if (vehicle.speed <= geom::Epsilon)
        {
            result.outcome = HazardOutcome::SafeStop;
            return result;
        }
    }
    throw std::runtime_error(
        "hazard trial exceeded the conservative frame limit without an outcome");
}

inline HazardResult runHazardTrial(
    const HazardScenario &scenario,
    const HazardDetector &detector,
    const VehicleConfig &vehicle_config = VehicleConfig{},
    unsigned frame_limit = hazardFrameLimit)
{
    validateHazardScenario(scenario, vehicle_config);
    return runValidatedHazardTrial(
        scenario, detector, vehicle_config, frame_limit);
}

inline HazardResult runHazardSpeedTrial(
    const HazardScenario &scenario,
    const HazardDetector &detector,
    const VehicleConfig &vehicle_config = VehicleConfig{},
    unsigned frame_limit = hazardFrameLimit)
{
    validateHazardSpeedScenario(scenario, vehicle_config);
    return runValidatedHazardTrial(
        scenario, detector, vehicle_config, frame_limit);
}
