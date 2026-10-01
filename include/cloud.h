// Copyright by BeeX [2026]
#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

namespace final_v1 {

// Where the jaws could close, as vision offers it.
struct GraspPose {
    Eigen::Vector3d point, bar, approach;
};

// A box of voxels. Cell index = x + nx * (y + ny * z).
struct VoxelBox {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    double          voxel  = 1.0;
    long            nx = 0, ny = 0, nz = 0;

    long            count() const { return nx * ny * nz; }
    long            index(long x, long y, long z) const { return x + nx * (y + ny * z); }
    long            cellOf(const Eigen::Vector3d &p) const;  // -1 outside
    Eigen::Vector3d centre(long cell) const;
    void            coords(long cell, long &x, long &y, long &z) const;
};

// Occupied cells in the arm base frame
struct ObstacleMap {
    VoxelBox              box;
    std::vector<uint32_t> obstacle, handle;
};

// Pinhole camera looking down -z with +y up, so image v grows downwards.
struct CameraModel {
    int    width = 0, height = 0;
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;

    Eigen::Vector2d project(const Eigen::Vector3d &p) const { return {cx + fx * p.x() / -p.z(), cy - fy * p.y() / -p.z()}; }
    bool            contains(double u, double v) const { return u >= 0.0 && v >= 0.0 && u < width && v < height; }
    size_t          index(int u, int v) const { return static_cast<size_t>(v) * width + u; }  // into an organised cloud
};

// One capture: an organised cloud (one point per pixel, row-major) and its grasp poses, in the camera frame.
struct Frame {
    std::vector<Eigen::Vector3f> points;
    std::vector<GraspPose>       poses;
    Eigen::Isometry3d            camera_to_arm = Eigen::Isometry3d::Identity();
};

struct CloudSettings {
    double voxel = 0.0, crop_radius = 0.0;
    double depth_tolerance = 0.0;
    int    min_agreeing_neighbours = 0, slope_window_px = 0;
    int    min_points_per_voxel = 0;
    double free_space_tolerance = 0.0, max_ray_stretch = 1.0;
    double handle_radius = 0.0, bar_gap = 0.0;
    double corridor_length = 0.0, corridor_radius = 0.0;
    int    consensus_min_frames = 0;
    double match = 0.0, max_axis = 0.0, outlier = 0.0, max_spread = 0.0, duplicate = 0.0;
    int    vote_min_frames = 0, vote_radius = 0;
};

// What a look produced, in the arm base frame.
struct Scene {
    std::vector<GraspPose> poses;       
    std::vector<GraspPose> candidates;  
    ObstacleMap            map;
    std::string            summary;
};

Scene processFrames(const std::vector<Frame> &frames, const CameraModel &camera, const CloudSettings &s);

// A camera_link cloud in any order put back on the pixel grid: the nearest point per pixel, NaN where none fell.
std::vector<Eigen::Vector3f> onPixelGrid(const std::vector<Eigen::Vector3f> &points, const CameraModel &camera);

// Spots on the bar that at least min_frames frames agree on, averaged.
std::vector<GraspPose> agreeOnSpots(const std::vector<std::vector<GraspPose>> &frames, const CloudSettings &s,
                                    std::string &summary);

double median(std::vector<double> v);

}  // namespace final_v1
