// Copyright by BeeX [2026]
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <utility>

namespace final_v1 {

// Joins each cloud with the poses of the same camera frame. Every cloud is held; poses take the one nearest in time within
// `slop`, whichever of the two arrives first, and a pair is handed out at most every `gap` seconds, by the cloud's stamp.
// What finds no partner within `wait` seconds of the newest stamp is dropped; missed() tells of poses dropped that way.
// Never more than kHeld of either are held, so stamps that stop advancing cannot pile clouds up. Stamps are nanoseconds;
// not thread safe.
template <class Cloud, class Poses>
class FramePairs {
public:
    FramePairs(double gap_s, double slop_s, double wait_s) : gap_(ns(gap_s)), slop_(ns(slop_s)), wait_(ns(wait_s)) {}

    static constexpr size_t kHeld = 12;

    void addCloud(int64_t stamp, const Cloud &cloud) {
        rewind(stamp);
        clouds_.push_back({stamp, cloud});
        if (clouds_.size() > kHeld) {
            clouds_.pop_front();
        }
        match(stamp);
    }

    void addPoses(int64_t stamp, const Poses &poses) {
        rewind(stamp);
        poses_.push_back({stamp, poses, -1});
        if (poses_.size() > kHeld) {
            dropPoses();
        }
        match(stamp);
    }

    // The oldest joined pair not yet handed out.
    bool next(Cloud &cloud, Poses &poses) {
        if (ready_.empty()) {
            return false;
        }
        cloud = ready_.front().first;
        poses = ready_.front().second;
        ready_.pop_front();
        return true;
    }

    // True once after poses were dropped: how far the nearest cloud was stamped while they waited, -1 with none held.
    bool missed(int64_t &nearest) {
        if (!missed_) {
            return false;
        }
        missed_ = false;
        nearest = missed_by_;
        return true;
    }

private:
    struct Waiting {
        int64_t stamp;
        Poses   poses;
        int64_t nearest;
    };

    static int64_t ns(double seconds) { return static_cast<int64_t>(seconds * 1e9); }

    // A stamp far behind the newest means the clock went back: start over.
    void rewind(int64_t stamp) {
        if (seen_ && stamp < newest_ - wait_) {
            clouds_.clear();
            poses_.clear();
            taken_ = seen_ = false;
        }
    }

    void match(int64_t stamp) {
        newest_ = seen_ ? std::max(newest_, stamp) : stamp;
        seen_   = true;
        for (auto p = poses_.begin(); p != poses_.end();) {
            const auto best = std::min_element(clouds_.begin(), clouds_.end(), [&](const auto &a, const auto &b) {
                return std::llabs(a.first - p->stamp) < std::llabs(b.first - p->stamp);
            });
            const int64_t off = best == clouds_.end() ? -1 : std::llabs(best->first - p->stamp);
            if (off < 0 || off > slop_) {
                if (off >= 0 && (p->nearest < 0 || off < p->nearest)) {
                    p->nearest = off;
                }
                ++p;
                continue;
            }
            if (!taken_ || best->first < last_taken_ || best->first - last_taken_ >= gap_) {
                ready_.push_back({best->second, p->poses});
                taken_      = true;
                last_taken_ = best->first;
            }
            clouds_.erase(clouds_.begin(), best + 1);  // poses come in frame order: nothing older will be asked for
            p = poses_.erase(p);
        }
        while (!clouds_.empty() && clouds_.front().first < newest_ - wait_) {
            clouds_.pop_front();
        }
        while (!poses_.empty() && poses_.front().stamp < newest_ - wait_) {
            dropPoses();
        }
    }

    void dropPoses() {
        missed_    = true;
        missed_by_ = poses_.front().nearest;
        poses_.pop_front();
    }

    int64_t                                gap_, slop_, wait_;
    bool                                   taken_ = false, seen_ = false, missed_ = false;
    int64_t                                last_taken_ = 0, newest_ = 0, missed_by_ = -1;
    std::deque<std::pair<int64_t, Cloud>>  clouds_;
    std::deque<Waiting>                    poses_;
    std::deque<std::pair<Cloud, Poses>>    ready_;
};

}  // namespace final_v1
