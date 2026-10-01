// Copyright by BeeX [2026]
#pragma once

#include <collision.h>

#include <functional>
#include <limits>

namespace final_v1 {

// A move of the vehicle relative to where it is now.
struct VehiclePose {
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
};

Eigen::Isometry3d toIsometry(const VehiclePose &p);

// Where the arm can put its grasp point, over (radial, z) around the base axis, dilated by one cell.
class ReachMap {
public:
    ReachMap(const Arm &arm, double along, double floor_z, double cell);
    bool reachable(double radial, double z) const;
    bool baseAngleFor(double azimuth) const;  // whether the base window can face this way

private:
    const Arm        &arm_;
    double            cell_, z_min_;
    int               nr_ = 0, nz_ = 0;
    std::vector<bool> cells_;
};

struct ParkSettings {
    double box_xy = 0.0, box_z = 0.0, box_yaw = 0.0;
    double coarse_step = 0.0, coarse_yaw = 0.0, fine_step = 0.0, fine_yaw = 0.0;
    int    refine_count = 0;
    double standoff_min = 0.0, standoff_max = 0.0;
    int    screen_count = 0;
    double screen_budget_s = 0.0;
    int    screen_blade_stride = 1, exact_count = 0, transit_samples = 0;
    double reach_cell = 0.0;
    double grasp_point_from_mount = 0.0, link_radius = 0.0, link_step = 0.0, blade_step = 0.0;
    double swing_band = 0.0;
};

enum class ParkDecision { MOVE, STAY, NOWHERE };

struct ParkChoice {
    ParkDecision decision = ParkDecision::NOWHERE;
    VehiclePose  move;              // relative to where the vehicle stands
    int          held  = 0;
    double       swing = 0.0;  // least largest joint move from home to a holding posture: a lower bound on the arm's move
    std::string  summary;
};

// Searches a bounded box of vehicle moves for one that brings the candidates into the arm's reach.
class ParkSearch {
public:
    ParkSearch(const Arm &arm, const Hull &hull, const ParkSettings &s, const Eigen::Isometry3d &vehicle_to_arm,
               const Eigen::Isometry3d &camera_to_vehicle, const CameraModel &camera);

    // Candidates and map in the arm frame; home is where the arm is held during the drive.
    ParkChoice choose(const std::vector<GraspPose> &candidates, const ObstacleMap &map, const Joints &home,
                      const std::function<bool()> &cancelled) const;

private:
    struct Scored {
        VehiclePose move;
        int         admitted = 0;
    };
    struct Holds {
        int    held  = 0;
        double swing = std::numeric_limits<double>::infinity();
    };

    int     admitted(const std::vector<GraspPose> &grasps, const VehiclePose &move) const;
    void    shortlist(const std::vector<GraspPose> &grasps, std::vector<Scored> &out) const;
    Holds   verify(const std::vector<GraspPose> &grasps, const VehiclePose &move, ObstacleGrid &grid, const Joints &home,
                   size_t stride) const;
    Eigen::Isometry3d armToCameraAfter(const VehiclePose &move) const;  // from the arm frame now
    bool    inView(const Eigen::Vector3d &point, const Eigen::Isometry3d &arm_to_camera) const;
    bool    transitClear(const VehiclePose &to, ObstacleGrid &grid, const Joints &home) const;
    Eigen::Isometry3d armMove(const VehiclePose &move) const;  // the arm frame after the move, in the arm frame now

    const Arm        &arm_;
    Hull              hull_;
    ParkSettings      s_;
    Eigen::Isometry3d vehicle_to_arm_;
    Eigen::Isometry3d camera_to_vehicle_;
    CameraModel       camera_;
    ReachMap          reach_;
};

}  // namespace final_v1
