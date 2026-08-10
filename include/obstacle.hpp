#pragma once
#include "geometry.hpp"
#include <memory>

// Shared obstacle POD used by the scene BVH, the linear-scan reference, and the
// benchmark world builders. Decoupled from any particular world generator so the
// core acceleration structures don't depend on a scene-authoring file.
enum class ObstacleShape
{
    Sphere,
    Cube
};

struct ActiveObstacle
{
    double x = 0.0;
    Vec3 center = Vec3::Zero();
    double size = 0.0;
    int object_id = -1;
    ObstacleShape shape = ObstacleShape::Cube;
    AABB bounds;
    std::shared_ptr<Geometry> geometry;
};
