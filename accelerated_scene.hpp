#pragma once
#include "obstacle.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

class XBucketScene
{
public:
    void rebuild(
        const std::vector<std::shared_ptr<Geometry>> &static_objects,
        const std::vector<ActiveObstacle> &obstacles,
        double bucket_size = 4.0)
    {
        static_objects_ = static_objects;
        obstacles_ = &obstacles;
        bucket_size_ = bucket_size;
        buckets_.clear();

        if (obstacles.empty())
        {
            min_x_ = 0.0;
            max_x_ = 0.0;
            return;
        }

        min_x_ = std::numeric_limits<double>::infinity();
        max_x_ = -std::numeric_limits<double>::infinity();
        for (const ActiveObstacle &obstacle : obstacles)
        {
            min_x_ = std::min(min_x_, obstacle.bounds.min().x());
            max_x_ = std::max(max_x_, obstacle.bounds.max().x());
        }

        const int bucket_count = std::max(1, bucketIndex(max_x_) + 1);
        buckets_.assign(bucket_count, {});

        for (int i = 0; i < static_cast<int>(obstacles.size()); ++i)
        {
            const int first = std::clamp(bucketIndex(obstacles[i].bounds.min().x()), 0, bucket_count - 1);
            const int last = std::clamp(bucketIndex(obstacles[i].bounds.max().x()), 0, bucket_count - 1);
            for (int b = first; b <= last; ++b)
                buckets_[b].push_back(i);
        }
    }

    Hit intersect(const Ray &ray, double max_distance) const
    {
        Hit closest = intersectObjects(static_objects_, ray, max_distance);

        if (!obstacles_ || obstacles_->empty() || buckets_.empty())
            return closest;

        double t0 = 0.0;
        double t1 = max_distance;
        if (std::abs(ray.dir.x()) < geom::Epsilon)
        {
            if (ray.ori.x() < min_x_ || ray.ori.x() > max_x_)
                return closest;
        }
        else
        {
            double tx0 = (min_x_ - ray.ori.x()) / ray.dir.x();
            double tx1 = (max_x_ - ray.ori.x()) / ray.dir.x();
            if (tx0 > tx1)
                std::swap(tx0, tx1);

            t0 = std::max(t0, tx0);
            t1 = std::min(t1, tx1);
            if (t1 < t0)
                return closest;
        }

        const double x0 = ray.ori.x() + std::max(0.0, t0) * ray.dir.x();
        const double x1 = ray.ori.x() + t1 * ray.dir.x();
        int first = bucketIndex(std::min(x0, x1));
        int last = bucketIndex(std::max(x0, x1));
        first = std::clamp(first, 0, static_cast<int>(buckets_.size()) - 1);
        last = std::clamp(last, 0, static_cast<int>(buckets_.size()) - 1);

        std::vector<char> seen(obstacles_->size(), 0);
        for (int bucket = first; bucket <= last; ++bucket)
        {
            for (int obstacle_index : buckets_[bucket])
            {
                if (seen[obstacle_index])
                    continue;
                seen[obstacle_index] = 1;

                Hit hit = (*obstacles_)[obstacle_index].geometry->intersect(ray);
                if (hit.hit && hit.t <= max_distance && (!closest.hit || hit.t < closest.t))
                    closest = hit;
            }
        }

        return closest;
    }

private:
    int bucketIndex(double x) const
    {
        return static_cast<int>(std::floor((x - min_x_) / bucket_size_));
    }

    static Hit intersectObjects(
        const std::vector<std::shared_ptr<Geometry>> &objects,
        const Ray &ray,
        double max_distance)
    {
        Hit closest;
        for (const auto &object : objects)
        {
            Hit hit = object->intersect(ray);
            if (hit.hit && hit.t <= max_distance && (!closest.hit || hit.t < closest.t))
                closest = hit;
        }
        return closest;
    }

    std::vector<std::shared_ptr<Geometry>> static_objects_;
    const std::vector<ActiveObstacle> *obstacles_ = nullptr;
    std::vector<std::vector<int>> buckets_;
    double bucket_size_ = 4.0;
    double min_x_ = 0.0;
    double max_x_ = 0.0;
};
