#include "simulation.hpp"
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

void updatePointCloud(const ScanResult &scan, open3d::geometry::PointCloud &pcd)
{
    pcd.points_.clear();
    pcd.colors_.clear();

    pcd.points_.reserve(scan.points.size());
    pcd.colors_.reserve(scan.points.size());

    for (const ScanPoint &point : scan.points)
    {
        double c = 0.25 + 0.75 * point.intensity;
        if (point.object_id >= 10)
            pcd.colors_.push_back(Vec3(1.0, 0.55, 0.15));
        else
            pcd.colors_.push_back(Vec3(c, c, c));

        pcd.points_.push_back(point.point_world);
    }
}

void updateFirstPersonCamera(
    open3d::visualization::Visualizer &vis,
    const Lidar &lidar)
{
    constexpr double lookahead = 8.0;

    const Vec3 eye = lidar.pose.translation();
    const Vec3 forward = lidar.pose.rotation() * Vec3(1.0, 0.0, 0.0);
    const Vec3 up = lidar.pose.rotation() * Vec3(0.0, 0.0, 1.0);

    auto &view = vis.GetViewControl();
    view.SetLookat(eye + lookahead * forward);
    view.SetFront(forward);
    view.SetUp(up);
}

std::shared_ptr<open3d::geometry::TriangleMesh> makeObstacleMesh(const ActiveObstacle &obstacle)
{
    std::shared_ptr<open3d::geometry::TriangleMesh> mesh;
    if (obstacle.shape == ObstacleShape::Sphere)
    {
        mesh = open3d::geometry::TriangleMesh::CreateSphere(obstacle.size * 0.5, 16);
        mesh->Translate(obstacle.center);
    }
    else
    {
        mesh = open3d::geometry::TriangleMesh::CreateBox(obstacle.size, obstacle.size, obstacle.size);
        mesh->Translate(obstacle.center - Vec3::Constant(obstacle.size * 0.5));
    }

    mesh->ComputeVertexNormals();
    mesh->PaintUniformColor(Vec3(0.85, 0.30, 0.12));
    return mesh;
}

std::shared_ptr<open3d::geometry::LineSet> makeLineSet()
{
    auto lines = std::make_shared<open3d::geometry::LineSet>();
    lines->points_.push_back(Vec3::Zero());
    lines->points_.push_back(Vec3::UnitX());
    lines->lines_.push_back(Eigen::Vector2i(0, 1));
    lines->colors_.push_back(Vec3(0.0, 1.0, 0.0));
    return lines;
}

void updateVehicleLines(
    const SimulationState &sim,
    open3d::geometry::LineSet &vehicle_lines)
{
    vehicle_lines.points_.clear();
    vehicle_lines.lines_.clear();
    vehicle_lines.colors_.clear();

    const double c = std::cos(sim.vehicle.heading);
    const double s = std::sin(sim.vehicle.heading);
    const Vec3 forward(c, s, 0.0);
    const Vec3 right(-s, c, 0.0);
    const Vec3 center(sim.vehicle.x, sim.vehicle.y, 0.08);
    const double half_l = 0.5 * sim.vehicle_config.length;
    const double half_w = 0.5 * sim.vehicle_config.width;

    vehicle_lines.points_ = {
        center + half_l * forward + half_w * right,
        center + half_l * forward - half_w * right,
        center - half_l * forward - half_w * right,
        center - half_l * forward + half_w * right};

    vehicle_lines.lines_ = {
        Eigen::Vector2i(0, 1),
        Eigen::Vector2i(1, 2),
        Eigen::Vector2i(2, 3),
        Eigen::Vector2i(3, 0)};

    const Vec3 color = sim.collided ? Vec3(1.0, 0.0, 0.0) : Vec3(0.1, 0.9, 0.35);
    vehicle_lines.colors_.assign(vehicle_lines.lines_.size(), color);
}

void updatePathLines(
    const SimulationState &sim,
    open3d::geometry::LineSet &path_lines)
{
    path_lines.points_.clear();
    path_lines.lines_.clear();
    path_lines.colors_.clear();

    if (sim.path.size() < 2)
    {
        const Vec3 origin = sim.lidar.pose.translation() + Vec3(0.0, 0.0, -sim.vehicle_config.lidar_height + 0.12);
        path_lines.points_ = {origin, origin + Vec3(0.001, 0.0, 0.0)};
        path_lines.lines_ = {Eigen::Vector2i(0, 1)};
        path_lines.colors_ = {Vec3(0.05, 0.65, 1.0)};
        return;
    }

    for (const Vec3 &point : sim.path)
        path_lines.points_.push_back(point + Vec3(0.0, 0.0, -sim.vehicle_config.lidar_height + 0.12));

    for (int i = 1; i < static_cast<int>(path_lines.points_.size()); ++i)
    {
        path_lines.lines_.push_back(Eigen::Vector2i(i - 1, i));
        path_lines.colors_.push_back(Vec3(0.05, 0.65, 1.0));
    }
}

void updateDirectionLines(
    const SimulationState &sim,
    open3d::geometry::LineSet &target_lines,
    open3d::geometry::LineSet &steering_lines)
{
    const Vec3 origin = sim.lidar.pose.translation();
    const Vec3 forward = sim.lidar.pose.rotation() * Vec3(1.0, 0.0, 0.0);
    const Vec3 target_dir =
        sim.lidar.pose.rotation() *
        Vec3(std::cos(sim.plan.target_angle), std::sin(sim.plan.target_angle), 0.0);
    const Vec3 steering_dir =
        Eigen::AngleAxisd(sim.vehicle.steering, Vec3::UnitZ()).toRotationMatrix() * forward;

    target_lines.points_ = {origin, origin + 5.0 * target_dir};
    target_lines.lines_ = {Eigen::Vector2i(0, 1)};
    target_lines.colors_ = {sim.plan.blocked ? Vec3(1.0, 0.0, 0.0) : Vec3(1.0, 0.95, 0.1)};

    steering_lines.points_ = {origin + Vec3(0.0, 0.0, -0.2), origin + Vec3(0.0, 0.0, -0.2) + 3.0 * steering_dir};
    steering_lines.lines_ = {Eigen::Vector2i(0, 1)};
    steering_lines.colors_ = {Vec3(0.1, 1.0, 0.55)};
}

std::shared_ptr<open3d::geometry::LineSet> makeHallwayLines(const HallwayWorld &world)
{
    auto lines = std::make_shared<open3d::geometry::LineSet>();
    constexpr double start_x = -10.0;
    constexpr double end_x = 80.0;
    const double w = world.half_width;
    const double h = world.height;

    lines->points_ = {
        Vec3(start_x, -w, 0.0), Vec3(end_x, -w, 0.0),
        Vec3(start_x, w, 0.0), Vec3(end_x, w, 0.0),
        Vec3(start_x, -w, h), Vec3(end_x, -w, h),
        Vec3(start_x, w, h), Vec3(end_x, w, h)};

    lines->lines_ = {
        Eigen::Vector2i(0, 1),
        Eigen::Vector2i(2, 3),
        Eigen::Vector2i(4, 5),
        Eigen::Vector2i(6, 7),
        Eigen::Vector2i(0, 4),
        Eigen::Vector2i(2, 6)};
    lines->colors_.assign(lines->lines_.size(), Vec3(0.35, 0.35, 0.38));
    return lines;
}

void printMetrics(const SimulationState &sim)
{
    std::cout << std::fixed << std::setprecision(2)
              << "mode=" << queryModeName(sim.query_mode)
              << " scan=" << sim.metrics.scan_ms << "ms"
              << " planner=" << sim.metrics.planner_ms << "ms"
              << " loop=" << sim.metrics.loop_ms << "ms"
              << " speed=" << sim.vehicle.speed << "m/s"
              << " steer=" << sim.vehicle.steering / geom::deg << "deg"
              << " collisions=" << sim.metrics.collision_count;

    if (sim.query_mode == QueryMode::XBuckets)
    {
        std::cout << " accel_check="
                  << (sim.metrics.accelerator_verified ? "ok" : "mismatch")
                  << "(" << sim.metrics.verification_mismatches << ")";
    }

    std::cout << '\n';
}

void refreshObstacleMeshes(
    open3d::visualization::Visualizer &vis,
    const SimulationState &sim,
    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> &obstacle_meshes)
{
    for (auto &mesh : obstacle_meshes)
        vis.RemoveGeometry(mesh, false);
    obstacle_meshes.clear();

    for (const ActiveObstacle &obstacle : sim.world.obstacles)
    {
        auto mesh = makeObstacleMesh(obstacle);
        obstacle_meshes.push_back(mesh);
        vis.AddGeometry(mesh, false);
    }
}

void runSimulation()
{
    SimulationState sim = makeSimulation(42);

    constexpr double dt = 1.0 / 30.0;
    bool exit_viewer = false;
    bool fpv_camera = true;

    open3d::visualization::VisualizerWithKeyCallback vis;
    vis.CreateVisualizerWindow("Follow-The-Gap LiDAR Benchmark", 1280, 720);

    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    updatePointCloud(sim.scan, *pcd);
    vis.AddGeometry(pcd);

    auto hallway_lines = makeHallwayLines(sim.world);
    auto vehicle_lines = makeLineSet();
    auto path_lines = makeLineSet();
    auto target_lines = makeLineSet();
    auto steering_lines = makeLineSet();

    updateVehicleLines(sim, *vehicle_lines);
    updatePathLines(sim, *path_lines);
    updateDirectionLines(sim, *target_lines, *steering_lines);

    vis.AddGeometry(hallway_lines, false);
    vis.AddGeometry(vehicle_lines, false);
    vis.AddGeometry(path_lines, false);
    vis.AddGeometry(target_lines, false);
    vis.AddGeometry(steering_lines, false);

    std::vector<std::shared_ptr<open3d::geometry::TriangleMesh>> obstacle_meshes;
    int rendered_revision = -1;
    refreshObstacleMeshes(vis, sim, obstacle_meshes);
    rendered_revision = sim.world.revision;

    vis.RegisterKeyCallback(
        GLFW_KEY_ESCAPE,
        [&exit_viewer](open3d::visualization::Visualizer *) {
            exit_viewer = true;
            return false;
        });

    vis.RegisterKeyCallback(
        GLFW_KEY_F,
        [&fpv_camera](open3d::visualization::Visualizer *) {
            fpv_camera = !fpv_camera;
            return false;
        });

    vis.RegisterKeyCallback(
        GLFW_KEY_M,
        [&sim](open3d::visualization::Visualizer *) {
            sim.query_mode = sim.query_mode == QueryMode::BruteForce
                                 ? QueryMode::XBuckets
                                 : QueryMode::BruteForce;
            std::cout << "query mode: " << queryModeName(sim.query_mode) << '\n';
            return false;
        });

    while (!exit_viewer)
    {
        if (!vis.PollEvents())
            break;

        stepSimulation(sim, dt);

        if (rendered_revision != sim.world.revision)
        {
            refreshObstacleMeshes(vis, sim, obstacle_meshes);
            rendered_revision = sim.world.revision;
        }

        updatePointCloud(sim.scan, *pcd);
        updateVehicleLines(sim, *vehicle_lines);
        updatePathLines(sim, *path_lines);
        updateDirectionLines(sim, *target_lines, *steering_lines);

        if (fpv_camera)
            updateFirstPersonCamera(vis, sim.lidar);

        vis.UpdateGeometry(pcd);
        vis.UpdateGeometry(vehicle_lines);
        vis.UpdateGeometry(path_lines);
        vis.UpdateGeometry(target_lines);
        vis.UpdateGeometry(steering_lines);
        vis.UpdateRender();

        if (sim.frame % 30 == 0)
            printMetrics(sim);

        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
}

int main()
{
    runSimulation();
    return 0;
}
