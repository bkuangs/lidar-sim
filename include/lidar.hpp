#pragma once
#include "geometry.hpp"

struct Lidar
{
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();

    double minRange = 0.1;
    double maxRange = 50.0;

    double minAzimuth = 0.0;
    double maxAzimuth = 2.0 * geom::pi;
    int azimuthSamples = 360;

    double minElevation = -30.0 * geom::deg;
    double maxElevation = 30.0 * geom::deg;
    int elevationSamples = 32;
};