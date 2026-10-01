// Copyright by BeeX [2026]
#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace final_v1 {

enum Joint : int { BASE = 0, SHOULDER, ELBOW, WRIST, JOINT_COUNT };
constexpr const char *JOINT_KEYS[JOINT_COUNT] = {"base", "shoulder", "elbow", "wrist"};

using Joints = std::array<double, JOINT_COUNT>;

// Every joint moves at one speed, so the largest joint move sets the time.
inline double largestMove(const Joints &a, const Joints &b) {
    double largest = 0.0;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        largest = std::max(largest, std::fabs(b[j] - a[j]));
    }
    return largest;
}

// The posture `t` of the way from a to b, every joint moving in step.
inline Joints lerp(const Joints &a, const Joints &b, double t) {
    Joints q;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        q[j] = a[j] + (b[j] - a[j]) * t;
    }
    return q;
}

inline double rad(double degrees) { return degrees * M_PI / 180.0; }
inline double deg(double radians) { return radians * 180.0 / M_PI; }

// A pose from xyz metres and roll, pitch, yaw degrees, the URDF convention.
inline Eigen::Isometry3d fromXyzRpy(const Eigen::Vector3d &xyz, const Eigen::Vector3d &rpy_deg) {
    Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
    t.linear()          = (Eigen::AngleAxisd(rad(rpy_deg.z()), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rad(rpy_deg.y()), Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(rad(rpy_deg.x()), Eigen::Vector3d::UnitX()))
                         .toRotationMatrix();
    t.translation() = xyz;
    return t;
}

// Planar offsets between the joint axes, metres.
struct ArmGeometry {
    double base_axis_z    = 0.0;
    double shoulder_x     = 0.0;
    double shoulder_z     = 0.0;
    double upper_x        = 0.0;
    double upper_z        = 0.0;
    double fore_x         = 0.0;
    double fore_z         = 0.0;
    double wrist_to_mount = 0.0;
    double elbow_sign     = 1.0;  // -1 when the elbow frame is turned half a turn
};

// Reads the geometry from the URDF joint origins; throws std::invalid_argument if the chain has another shape.
ArmGeometry geometryFromUrdf(const std::string &urdf_xml, const std::array<std::string, JOINT_COUNT> &joint_names,
                             const std::string &mount_joint);

struct BladeBand {
    double approach_min, approach_max, closing_min, closing_max, hinge_half_width;
};

struct JawShape {
    double                 mount_to_throat       = 0.0;
    double                 mount_to_tip          = 0.0;
    double                 palm_length           = 0.0;
    double                 open_len              = 0.0;
    double                 hinge_roll_at_zero    = 0.0;
    double                 blade_rotation_per_m  = 0.0;
    double                 hinge_offset_closing  = 0.0;
    double                 hinge_offset_approach = 0.0;
    std::vector<BladeBand> blades;
};

// Model angle = sign * (reported - offset); limits are in reported radians.
struct ArmConfig {
    ArmGeometry geometry;
    JawShape    jaw;
    Joints      offset{};
    Joints      sign{};
    Joints      min{};
    Joints      max{};
    double      shoulder_clear_of_housing = -std::numeric_limits<double>::infinity();
};

struct ArmPoints {
    Eigen::Vector3d shoulder, elbow, wrist, mount, throat, tip;
};

struct JawAxes {
    Eigen::Vector3d approach, hinge, closing;
};

struct Segment {
    Eigen::Vector3d a, b;
};

// The collision shape at one posture: link centrelines, tool points and samples of both open blades.
struct Body {
    std::array<Segment, 4>       links;
    Eigen::Vector3d              throat, tip;
    std::vector<Eigen::Vector3d> blades;
};

enum class Ik { SOLVED, OUT_OF_REACH, JOINT_LIMIT };

// Kinematics of the arm in model radians, in the arm base frame.
class Arm {
public:
    // Samples the blade profile every `blade_step` metres; throws std::invalid_argument on a bad config.
    Arm(const ArmConfig &config, double blade_step);

    Joints           toModel(const Joints &reported) const;
    Joints           toReported(const Joints &model) const;
    double           lower(int joint) const { return lower_[joint]; }
    double           upper(int joint) const { return upper_[joint]; }
    bool             withinLimits(const Joints &q) const;
    Joints           clamped(Joints q) const {  // the nearest posture inside the limits
        for (int j = 0; j < JOINT_COUNT; ++j) {
            q[j] = std::clamp(q[j], lower_[j], upper_[j]);
        }
        return q;
    }

    ArmPoints points(const Joints &q) const;
    JawAxes   axes(const Joints &q) const;
    void      body(const Joints &q, Body &out, size_t blade_stride = 1) const;

    double mountDistance() const { return config_.geometry.wrist_to_mount; }
    double tipDistance() const { return mountDistance() + config_.jaw.mount_to_tip; }
    double reach(double along) const;  // farthest the point `along` the wrist axis gets from the base origin

    // Puts the point `along` metres down the wrist axis on `target`; the wrist is copied from `seed`.
    Ik   solve(const Eigen::Vector3d &target, double along, bool elbow_up, const Joints &seed, Joints &out) const;
    bool fitIntoLimits(int joint, double angle, double seed, double &out) const;
    // The two wrist rolls that close the blades across `bar`; false when the bar lies along the approach.
    bool rollsAcrossBar(const Joints &q, const Eigen::Vector3d &bar, double rolls[2]) const;
    // The open blades' inner faces `depth` metres down the approach from the mount, on the closing line; the gap between
    // them is shut where low is not below high. False where no blade reaches, as past the tips.
    bool gapAt(double depth, double &low, double &high) const;

private:
    double upperLength() const { return std::hypot(config_.geometry.upper_x, config_.geometry.upper_z); }
    double upperAngle() const { return std::atan2(config_.geometry.upper_x, config_.geometry.upper_z); }
    void   forearm(double along, double &length, double &angle) const;

    ArmConfig                    config_;
    Joints                       lower_{};
    Joints                       upper_{};
    std::vector<Eigen::Vector3d> blade_local_;  // one blade, then its mirror
    double                       blade_step_ = 0.0;
};

}  // namespace final_v1
