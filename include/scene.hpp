#pragma once
#include "geometry.hpp"

struct Scene
{
    std::vector<std::shared_ptr<Geometry>> objects;

    Hit intersect(const Ray &ray) const
    {
        Hit closest;

        for (const auto &object : objects)
        {
            Hit h = object->intersect(ray);
            // hit is valid, no hit has been stored yet or less than closest
            if (h.hit && (closest.t < 0.0 || h.t < closest.t))
            {
                closest = h;
            }
        }

        return closest;
    }
};