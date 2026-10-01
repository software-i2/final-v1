// Copyright by BeeX [2026]
#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <utility>

namespace final_v1 {

// Joins each cloud with the poses of the same camera frame. A cloud is taken at most every `gap` seconds, by its own stamp;
// its poses are the nearest in time within `slop`, whichever of the two arrives first. What finds no partner within `wait`
// seconds of the newest stamp is dropped. Stamps are nanoseconds; not thread safe.
template <class Cloud, class Poses>
class FramePairs {
public:
    FramePairs(double gap_s, double slop_s, double wait_s) : gap_(ns(gap_s)), slop_(ns(slop_s)), wait_(ns(wait_s)) {}

    // False when the cloud came sooner than `gap` after the last one taken.
    bool addCloud(int64_t stamp, const Cloud &cloud) {
        rewind(stamp);
        if (taken_ && stamp >= last_taken_ && stamp - last_taken_ < gap_) {
            return false;
        }
        taken_      = true;
        last_taken_ = stamp;
        clouds_.push_back({stamp, cloud});
        match(stamp);
        return true;
    }

    void addPoses(int64_t stamp, const Poses &poses) {
        rewind(stamp);
        poses_.push_back({stamp, poses});
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

private:
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
        for (auto c = clouds_.begin(); c != clouds_.end();) {
            auto best = poses_.end();
            for (auto p = poses_.begin(); p != poses_.end(); ++p) {
                const int64_t off = std::llabs(p->first - c->first);
                if (off <= slop_ && (best == poses_.end() || off < std::llabs(best->first - c->first))) {
                    best = p;
                }
            }
            if (best == poses_.end()) {
                ++c;
                continue;
            }
            ready_.push_back({c->second, best->second});
            poses_.erase(best);
            c = clouds_.erase(c);
        }
        while (!clouds_.empty() && clouds_.front().first < newest_ - wait_) {
            clouds_.pop_front();
        }
        while (!poses_.empty() && poses_.front().first < newest_ - wait_) {
            poses_.pop_front();
        }
    }

    int64_t                                gap_, slop_, wait_;
    bool                                   taken_ = false, seen_ = false;
    int64_t                                last_taken_ = 0, newest_ = 0;
    std::deque<std::pair<int64_t, Cloud>>  clouds_;
    std::deque<std::pair<int64_t, Poses>>  poses_;
    std::deque<std::pair<Cloud, Poses>>    ready_;
};

}  // namespace final_v1
