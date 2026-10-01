// Copyright by BeeX [2026]
#include <track.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

namespace final_v1 {
namespace {

bool finite(const Eigen::Vector3f &p) { return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z()) && p.z() < 0.0f; }

const double kGateWide  = 0.03;   // first iterations: how far the mine may have moved since the last fit
const double kHuber     = 0.004;  // residuals past this weigh less
const int    kIters     = 10;
const int    kMaxPoints = 6000;   // visible samples fitted each iteration
const double kDamping   = 1e-3;   // share of the mean Hessian diagonal
const double kRoi       = 0.25;   // metres around the target searched for the top face
const double kFace      = 0.003;  // plane tolerance of the top face
const double kFront     = 0.05;   // depth band from the nearest point that holds the second top-face guess
const int    kSteps     = 4;      // centre guesses from the target inwards, up to the mine's radius
const size_t kShown     = 800;    // samples shown

}  // namespace

MineTracker::MineTracker(std::vector<MineModel> mines, const TrackSettings &s, const CameraModel &camera)
        : mines_(std::move(mines)), s_(s), camera_(camera) {}

const std::string &MineTracker::mine() const {
    static const std::string none;
    return mines_.empty() ? none : mines_[chosen_].name;
}

bool MineTracker::pixel(const Eigen::Vector3d &p, int &u, int &v) const {
    if (p.z() >= -1e-3) {
        return false;
    }
    const Eigen::Vector2d px = camera_.project(p);
    u                        = static_cast<int>(std::lround(px.x()));
    v                        = static_cast<int>(std::lround(px.y()));
    return u >= 0 && v >= 0 && u < camera_.width && v < camera_.height;
}

// A mine is convex, so a sample is seen when its outward normal faces the camera.
std::vector<int> MineTracker::visible(const MineModel &m, const Eigen::Isometry3d &pose) const {
    std::vector<int> seen;
    for (size_t i = 0; i < m.points.size(); ++i) {
        int                   u = 0, v = 0;
        const Eigen::Vector3d q = pose * m.points[i].cast<double>();
        if (pixel(q, u, v) && (pose.linear() * m.normals[i].cast<double>()).dot(q) < 0.0) {
            seen.push_back(static_cast<int>(i));
        }
    }
    if (seen.size() > static_cast<size_t>(kMaxPoints)) {  // an even stride keeps the spread
        std::vector<int> kept;
        for (size_t k = 0; k < static_cast<size_t>(kMaxPoints); ++k) {
            kept.push_back(seen[k * seen.size() / kMaxPoints]);
        }
        seen.swap(kept);
    }
    return seen;
}

// Each sample meets the cloud point at its own pixel; the gate narrows from gate0 to gate1. keep_turn holds the turn about the
// mine's own axis, which a cone gives the fit no hold on.
MineTracker::Fit MineTracker::fit(const MineModel &m, const std::vector<Eigen::Vector3f> &cloud, Eigen::Isometry3d pose, int iters,
                                  double gate0, double gate1, bool keep_turn) const {
    const std::vector<int> seen = visible(m, pose);
    Fit                    out;
    for (int it = 0; it < iters; ++it) {
        const double                gate = gate0 + (gate1 - gate0) * std::min(1.0, it / (0.6 * std::max(1, iters - 1)));
        const Eigen::Matrix3d       r    = pose.linear();
        Eigen::Matrix<double, 6, 6> a    = Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b    = Eigen::Matrix<double, 6, 1>::Zero();
        int                         n    = 0;
        for (const int i : seen) {
            const Eigen::Vector3d q = pose * m.points[i].cast<double>();
            int                   u = 0, v = 0;
            if (!pixel(q, u, v)) {
                continue;
            }
            const Eigen::Vector3f &y = cloud[camera_.index(u, v)];
            if (!finite(y) || (y.cast<double>() - q).norm() > gate) {
                continue;
            }
            const Eigen::Vector3d normal = r * m.normals[i].cast<double>();
            const double          e      = normal.dot(y.cast<double>() - q);
            const double          w      = std::fabs(e) <= kHuber ? 1.0 : kHuber / std::fabs(e);
            Eigen::Matrix<double, 6, 1> j;
            j << q.cross(normal), normal;
            a += w * j * j.transpose();
            b += w * e * j;
            ++n;
        }
        if (n < 50) {
            return Fit();
        }
        out.inliers = n;
        a.diagonal().array() += kDamping * a.trace() / 6.0;
        Eigen::Matrix<double, 6, 1> x;
        if (keep_turn) {
            const Eigen::Vector3d       axis = r.col(1), across = axis.unitOrthogonal();
            Eigen::Matrix<double, 6, 5> basis = Eigen::Matrix<double, 6, 5>::Zero();
            basis.block<3, 1>(0, 0) = across, basis.block<3, 1>(0, 1) = axis.cross(across);
            basis.block<3, 3>(3, 2) = Eigen::Matrix3d::Identity();
            x = basis * (basis.transpose() * a * basis).ldlt().solve(basis.transpose() * b);
        } else {
            x = a.ldlt().solve(b);
        }
        const Eigen::Vector3d w    = x.head<3>();
        const Eigen::Matrix3d turn = w.norm() > 1e-12 ? Eigen::AngleAxisd(w.norm(), w.normalized()).toRotationMatrix() : Eigen::Matrix3d::Identity();
        pose.linear()      = turn * pose.linear();
        pose.translation() = turn * pose.translation() + x.tail<3>();
    }
    out.pose = pose;
    return out;
}

// The share of the visible mine that lands on the cloud within the gate.
double MineTracker::explains(const MineModel &m, const std::vector<Eigen::Vector3f> &cloud, const Eigen::Isometry3d &pose) const {
    const std::vector<int> seen = visible(m, pose);
    int                    hit  = 0;
    for (const int i : seen) {
        const Eigen::Vector3d q = pose * m.points[i].cast<double>();
        int                   u = 0, v = 0;
        if (pixel(q, u, v)) {
            const Eigen::Vector3f &y = cloud[camera_.index(u, v)];
            hit += finite(y) && (y.cast<double>() - q).norm() <= s_.gate;
        }
    }
    return seen.empty() ? 0.0 : static_cast<double>(hit) / seen.size();
}

// The top faces the camera. Its plane is the largest flat patch around the target, or the largest in its nearest few
// centimetres; its middle may lie outside the patch on a big mine, so it is tried at the patch's middle and at steps from the
// target towards it, up to the mine's radius.
MineTracker::Fit MineTracker::place(const MineModel &m, const std::vector<Eigen::Vector3f> &look, const Eigen::Vector3d &target,
                                    double &share) const {
    std::vector<Eigen::Vector3d> around;
    double                       front = std::numeric_limits<double>::infinity();
    for (int v = 0; v < camera_.height; v += 2) {
        for (int u = 0; u < camera_.width; u += 2) {
            const Eigen::Vector3f &p = look[camera_.index(u, v)];
            if (finite(p) && (p.cast<double>() - target).norm() < kRoi && -p.z() < -target.z() + kGateWide) {
                around.push_back(p.cast<double>());
                front = std::min(front, static_cast<double>(-p.z()));
            }
        }
    }
    std::vector<Eigen::Vector3d> nearest;
    std::copy_if(around.begin(), around.end(), std::back_inserter(nearest), [&](const Eigen::Vector3d &p) { return -p.z() < front + kFront; });
    double radius = 0.0;
    for (const Eigen::Vector3f &p : m.points) {
        radius = std::max(radius, static_cast<double>(std::hypot(p.x(), p.z())));
    }

    const auto tryPose = [&](const Eigen::Isometry3d &guess, Fit &best) {
        Fit f = fit(m, look, guess, 15, 0.04, kGateWide, false);
        f     = fit(m, look, f.pose, 20, kGateWide, s_.gate, false);
        const double explained = f.inliers > 0 ? explains(m, look, f.pose) : 0.0;
        if (explained > share) {
            best = f, share = explained;
        }
    };
    Fit best;
    share = 0.0;
    for (const std::vector<Eigen::Vector3d> *patch : {&around, &nearest}) {
        if (patch->size() < 100 || (patch == &nearest && nearest.size() == around.size())) {
            continue;
        }
        std::mt19937                          rng(1);
        std::uniform_int_distribution<size_t> pick(0, patch->size() - 1);
        int                                   most = 0;
        Eigen::Vector3d                       normal = Eigen::Vector3d::UnitZ(), on = patch->front();
        for (int k = 0; k < 300; ++k) {
            const Eigen::Vector3d a = (*patch)[pick(rng)], n = ((*patch)[pick(rng)] - a).cross((*patch)[pick(rng)] - a);
            if (n.norm() < 1e-9) {
                continue;
            }
            const int count = static_cast<int>(std::count_if(patch->begin(), patch->end(), [&](const Eigen::Vector3d &p) {
                return std::fabs(n.normalized().dot(p - a)) < kFace;
            }));
            if (count > most) {
                most = count, normal = n.normalized(), on = a;
            }
        }
        Eigen::Vector3d middle = Eigen::Vector3d::Zero();
        for (const Eigen::Vector3d &p : *patch) {
            middle += std::fabs(normal.dot(p - on)) < kFace ? p : Eigen::Vector3d::Zero();
        }
        middle /= std::max(most, 1);
        if (normal.dot(middle) > 0) {
            normal = -normal;
        }
        const Eigen::Vector3d x = Eigen::Vector3d::UnitY().cross(normal).normalized();
        Eigen::Matrix3d       facing;
        facing << x, normal, x.cross(normal);
        const Eigen::Vector3d foot   = target - normal.dot(target - middle) * normal;  // the target dropped onto the plane
        const Eigen::Vector3d inward = (middle - foot).norm() > 1e-6 ? Eigen::Vector3d((middle - foot).normalized()) : Eigen::Vector3d::Zero();
        std::vector<Eigen::Vector3d> centres = {middle};
        for (int k = 1; k <= kSteps && inward.norm() > 0.0; ++k) {
            centres.push_back(foot + inward * radius * k / kSteps);
        }
        for (const Eigen::Vector3d &centre : centres) {
            Eigen::Isometry3d guess = Eigen::Isometry3d::Identity();
            guess.linear() = facing, guess.translation() = centre;
            tryPose(guess, best);
        }
    }
    return best;
}

bool MineTracker::start(const std::vector<Eigen::Vector3f> &look, const Eigen::Vector3d &target, double stamp) {
    active_ = false, fit_ = 0.0, start_ = target;
    if (look.size() != static_cast<size_t>(camera_.width) * camera_.height) {
        return false;
    }
    Fit best;
    for (size_t k = 0; k < mines_.size(); ++k) {
        double    share = 0.0;
        const Fit f     = place(mines_[k], look, target, share);
        if (share > fit_) {
            best = f, fit_ = share, chosen_ = k;
        }
    }
    if (best.inliers == 0 || fit_ < s_.min_fit) {
        return false;
    }
    pose_           = best.pose;
    target_         = pose_.inverse() * target;
    start_inliers_  = best.inliers;
    trusted_at_     = stamp;
    return active_ = true;
}

TrackResult MineTracker::update(const std::vector<Eigen::Vector3f> &points, double stamp) {
    TrackResult r;
    if (!active_ || points.size() != static_cast<size_t>(camera_.width) * camera_.height) {
        r.target = start_;
        return r;
    }
    const MineModel &m = mines_[chosen_];
    const Fit        f = fit(m, points, pose_, kIters, kGateWide, s_.gate, true);
    r.seen             = static_cast<double>(f.inliers) / start_inliers_;
    r.ok = r.seen >= s_.min_seen && (f.pose.translation() - pose_.translation()).norm() <= s_.gate + s_.max_speed * (stamp - trusted_at_);
    if (r.ok) {
        pose_ = f.pose, trusted_at_ = stamp;
    }
    r.target    = pose_ * target_;
    r.offset    = r.target - start_;
    const std::vector<int> seen = visible(m, pose_);
    for (size_t k = 0; k < seen.size(); k += std::max<size_t>(1, seen.size() / kShown)) {
        const Eigen::Vector3d  q = pose_ * m.points[seen[k]].cast<double>();
        int                    u = 0, v = 0;
        const bool             inside = pixel(q, u, v);
        r.shown.push_back(q.cast<float>());
        r.matched.push_back(inside && finite(points[camera_.index(u, v)]) && (points[camera_.index(u, v)].cast<double>() - q).norm() <= s_.gate);
    }
    return r;
}

}  // namespace final_v1
