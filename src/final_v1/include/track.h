// Copyright by BeeX [2026]
#pragma once

#include <cloud.h>
#include <mine.h>

#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace final_v1 {

struct TrackSettings {
    double gate      = 0.0;  // metres a cloud point may sit from the fitted mine and still count
    double min_fit   = 0.0;  // share of the visible mine the look's fit must land on the cloud
    double min_seen  = 0.0;  // share of the look's matches below which a frame's fit is held, not trusted
    double max_speed = 0.0;  // m/s the mine can move; a fit moving it faster is a wrong lock
};

// One frame of tracking. Points are in camera_link (x right, y up, looking down -z).
struct TrackResult {
    bool                     ok = false;  // this frame's fit is trusted
    Eigen::Vector3d          target    = Eigen::Vector3d::Zero();
    Eigen::Vector3d          offset    = Eigen::Vector3d::Zero();  // target now minus where the look saw it
    double                   seen = 0.0;  // share of the look's matches found in this frame
    std::vector<Eigen::Vector3f> shown;   // a spread of the fitted mine's visible samples
    std::vector<uint8_t>     matched;     // per shown sample: it lies on the cloud
};

// Follows the mine while the arm moves blind. Every mine shape is fitted to the look around the target and the one that
// explains the cloud best is fitted again each frame (point-to-plane ICP), carrying the target with it.
class MineTracker {
public:
    MineTracker(std::vector<MineModel> mines, const TrackSettings &s, const CameraModel &camera);

    // False when no mine explains min_fit of itself in the look: nothing is tracked.
    bool               start(const std::vector<Eigen::Vector3f> &look, const Eigen::Vector3d &target, double stamp);
    TrackResult        update(const std::vector<Eigen::Vector3f> &points, double stamp);
    void               stop() { active_ = false; }
    bool               active() const { return active_; }
    const std::string &mine() const;  // the mine start chose
    double             fit() const { return fit_; }  // the share of it the look explained

private:
    struct Fit {
        Eigen::Isometry3d pose    = Eigen::Isometry3d::Identity();  // mine to camera_link
        int               inliers = 0;
    };
    std::vector<int> visible(const MineModel &m, const Eigen::Isometry3d &pose) const;
    Fit              fit(const MineModel &m, const std::vector<Eigen::Vector3f> &cloud, Eigen::Isometry3d pose, int iters, double gate0,
                         double gate1, bool keep_turn) const;
    double           explains(const MineModel &m, const std::vector<Eigen::Vector3f> &cloud, const Eigen::Isometry3d &pose) const;
    Fit              place(const MineModel &m, const std::vector<Eigen::Vector3f> &look, const Eigen::Vector3d &target, double &share) const;
    bool             pixel(const Eigen::Vector3d &p, int &u, int &v) const;

    std::vector<MineModel> mines_;
    TrackSettings          s_;
    CameraModel            camera_;
    size_t                 chosen_ = 0;
    bool                   active_ = false;
    Eigen::Isometry3d      pose_   = Eigen::Isometry3d::Identity();
    Eigen::Vector3d        target_ = Eigen::Vector3d::Zero();  // in the mine's frame
    Eigen::Vector3d        start_  = Eigen::Vector3d::Zero();  // where the look saw it
    int                    start_inliers_ = 0;
    double                 fit_ = 0.0, trusted_at_ = 0.0;
};

}  // namespace final_v1
