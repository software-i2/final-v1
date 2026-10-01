// Copyright by BeeX [2026]
#include <pick.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace final_v1 {

Pick::Pick(const PickConfig &c, const Arm &arm, PickIO &io)
        : c_(c),
          arm_(arm),
          io_(io),
          park_(arm, c.hull, c.park, c.arm_to_vehicle.inverse(), c.camera_to_vehicle, c.camera),
          follower_(c.follow),
          tracker_(c.mines, c.track, c.camera) {
    c_.cloud.crop_radius += arm_.reach(arm_.tipDistance());
    seedPlanner(static_cast<unsigned>(c_.seed));
    entered_ = io_.now();
    publishState("waiting for start");
}

void Pick::tick() {
    if (stop_requested_.exchange(false)) {
        if (isWorking(state_)) {
            apply(Event::STOP, "stopped");
        } else if (!io_.standby()) {  // after a pick the arm still holds its last posture
            io_.log(Level::ERROR, "stop: standby failed, the arm may still be powered");
        }
    }
    if (manual_requested_.exchange(false)) {
        if (c_.arm_move) {
            manual_planned_ = false;
            apply(Event::GOTO, "manual command");
        } else {
            io_.log(Level::WARN, "manual command ignored: the arm is not to move");
        }
    }
    if (start_requested_.exchange(false)) {
        if (isWorking(state_)) {
            io_.log(Level::WARN, std::string("start ignored: already in ") + stateName(state_));
        } else {
            park_attempt_ = look_attempt_ = 0;
            grabbed_         = false;
            miss_            = std::numeric_limits<double>::quiet_NaN();
            surveying_since_ = reparking_since_ = 0.0;
            run_started_                        = io_.now();
            io_.showMap(ObstacleMap(), Eigen::Isometry3d::Identity(), false);
            io_.showPath({});
            if (c_.track_enabled) {
                io_.clearTrack();
            }
            c_.track_steer = steer_requested_;
            c_.park_skip   = skip_requested_;
            apply(Event::START, std::string(c_.track_steer ? "steer mode" : "open loop") + (c_.park_skip ? ", parking skipped" : ""));
        }
    }
    std::string message;
    const Event event = step(message);
    if (!stop_requested_) {
        apply(event, message);
    }
    if (++ticks_ % std::max(1, static_cast<int>(c_.loop_hz / 2.0)) == 0) {
        publishBody();
    }
}

// --- requests ---

void Pick::requestStart(bool steer, bool skip_park) {
    steer_requested_ = steer;
    skip_requested_  = skip_park;
    start_requested_ = true;
}

void Pick::requestPoint(const Eigen::Vector3d &grasp_point) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    manual_point_     = grasp_point;
    manual_is_point_  = true;
    manual_requested_ = true;
}

void Pick::requestPosture(const Joints &reported) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    manual_posture_   = reported;
    manual_is_point_  = false;
    manual_requested_ = true;
}

bool Pick::reset(std::string &message) {
    if (working_) {
        message = "a pick is running, stop it first";
        return false;
    }
    std::string homed;
    message = !c_.arm_move ? "arm not homed: the arm is not to move" : io_.home(homed) ? "arm " + homed : "arm not homed: the driver did not answer";
    return true;
}

// --- sensors ---

void Pick::cloudSeen() {
    const double now = io_.now();
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    frame_seen_ = now;
}

void Pick::addFrame(Frame frame) {
    frame.camera_to_arm = c_.arm_to_vehicle.inverse() * c_.camera_to_vehicle;

    std::lock_guard<std::mutex> lock(sensor_mutex_);
    frames_.push_back(std::move(frame));
    while (frames_.size() > static_cast<size_t>(c_.frames_needed)) {
        frames_.pop_front();
    }
}

void Pick::addPoseless() {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    ++poseless_;
}

void Pick::joints(const Joints &reported, double stamp) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    joints_       = reported;
    joints_stamp_ = stamp;
}

void Pick::jaw(double position, double stamp) {
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    jaw_       = position;
    jaw_stamp_ = stamp;
}

bool Pick::tracking() {
    std::lock_guard<std::mutex> lock(track_mutex_);
    return tracker_.active();
}

// While the arm moves blind, follow the mine in the cloud; with track/steer, steer() moves the goal with it.
void Pick::trackFrame(const std::vector<Eigen::Vector3f> &points) {
    std::lock_guard<std::mutex> lock(track_mutex_);
    if (!tracker_.active()) {
        return;
    }
    track_ = tracker_.update(points, io_.now());
    trail_.push_back(track_.target);
    io_.showTrack(track_, trail_);
}

// Reported radians, if newer than the timeout.
bool Pick::freshJoints(Joints &q, double &stamp) {
    const double now = io_.now();
    std::lock_guard<std::mutex> lock(sensor_mutex_);
    if (joints_stamp_ == 0.0 || now - joints_stamp_ > c_.joint_state_timeout_s) {
        return false;
    }
    q     = joints_;
    stamp = joints_stamp_;
    return true;
}

// --- state machine ---

void Pick::apply(Event event, const std::string &message) {
    const State next = nextState(state_, event);
    if (next == state_) {
        detail_.clear();
        return;
    }
    if (next == State::COLLECT) {
        spot_phase_ = !c_.park_skip && (state_ == State::STREAM || state_ == State::RESURVEY || state_ == State::REPARK);
        if (!spot_phase_) {
            io_.showMap(ObstacleMap(), Eigen::Isometry3d::Identity(), true);
        }
    }
    enter(next, message);
}

void Pick::enter(State next, std::string message) {
    const State  left  = state_;
    const double now   = io_.now();
    const double spent = now - entered_;
    state_             = next;
    working_           = isWorking(next);
    entered_           = now;
    if (next != State::GOTOGRASP && next != State::CLOSEJAW) {
        std::lock_guard<std::mutex> lock(track_mutex_);
        tracker_.stop();
    }
    char line[200];
    switch (next) {
    case State::GOTOGRASP: {
        // follow the chosen candidate from where the grasp look saw it.
        if (!c_.track_enabled) {
            break;
        }
        std::lock_guard<std::mutex> lock(track_mutex_);
        if (!tracker_.start(look_, scene_camera_to_arm_.inverse() * scene_.candidates[plan_.candidate].point, look_stamp_)) {
            std::snprintf(line, sizeof(line), "no mine fits the look (best %s explains %.0f%%): open loop", tracker_.mine().c_str(),
                          100 * tracker_.fit());
            io_.log(Level::WARN, line);
        }
        track_ = TrackResult();
        trail_.clear();
        steer_note_ = "on the plan";
        break;
    }
    case State::COLLECT: {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        frames_.clear();
        poseless_ = 0;
        break;
    }
    case State::PICKSPOT:
        surveying_since_ = 0.0;
        break;
    case State::CLOSEJAW: {
        if (!c_.jaw_close) {
            caught_ = handleCaught();
            break;  // checkJaw reports the judgement and leaves the jaw alone
        }
        std::string why;
        if (!io_.closeJaw(why)) {
            enter(State::FAIL, "could not close the jaw: " + why);
            return;
        }
        jaw_closed_at_   = io_.now();
        jaw_last_seen_   = 0.0;
        jaw_still_since_ = 0.0;
        break;
    }
    case State::RESURVEY:
        if (surveying_since_ == 0.0) {
            surveying_since_ = now;
        }
        std::snprintf(line, sizeof(line), "; %.0f/%.0f s", now - surveying_since_, c_.spot_search_s);
        message += line;
        break;
    case State::RETARGET:
        ++look_attempt_;
        std::snprintf(line, sizeof(line), "; look %u/%d", look_attempt_, c_.retarget_attempts);
        message += line;
        break;
    case State::REPARK:
        look_attempt_ = 0;
        ++park_attempt_;
        if (reparking_since_ == 0.0) {
            reparking_since_ = now;
        }
        std::snprintf(line, sizeof(line), "; spot %u/%d", park_attempt_, c_.park_attempts);
        message += line;
        break;
    case State::ESTOP:
        if (!io_.standby()) {
            message += "; standby failed, the arm may still be powered";
        }
        break;
    default:
        break;
    }
    publishState(message, left, spent);
}

Event Pick::step(std::string &message) {
    switch (state_) {
    case State::STREAM: return checkStream(message);
    case State::COLLECT: return checkCollect(message);
    case State::PROCESS: return process(message);
    case State::PICKSPOT: return pickSpot(message);
    case State::PICKGRASP: return pickGrasp(message);
    case State::GOTOGRASP: return follow(message);
    case State::MANUAL: return manual(message);
    case State::CLOSEJAW: return checkJaw(message);
    case State::RESURVEY: return retry(surveying_since_, c_.spot_search_s, false, Event::SURVEY_AGAIN, Event::OUT_OF_TIME, message);
    case State::RETARGET:
        return retry(0.0, 0.0, look_attempt_ >= static_cast<uint32_t>(c_.retarget_attempts), Event::LOOK_AGAIN, Event::OUT_OF_LOOKS, message);
    case State::REPARK:
        return retry(reparking_since_, c_.repark_search_s, c_.park_skip || park_attempt_ >= static_cast<uint32_t>(c_.park_attempts), Event::PARK_AGAIN,
                     Event::OUT_OF_PARKS, message);
    default: return Event::NONE;
    }
}

// Looks again at once unless a count or a clock has run out.
Event Pick::retry(double since, double budget_s, bool out_of_tries, Event again, Event done, std::string &message) {
    if (out_of_tries || (since != 0.0 && io_.now() - since >= budget_s)) {
        message = out_of_tries ? "out of tries" : "out of time";
        return done;
    }
    message = "looking again";
    return again;
}

// A camera already producing when start was pressed counts.
Event Pick::checkStream(std::string &message) {
    double seen = 0.0;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        seen = frame_seen_;
    }
    const double now = io_.now();
    if (seen != 0.0 && now - seen < c_.stream_timeout_s) {
        message = "camera up";
        return Event::STREAMING;
    }
    if (now - entered_ >= c_.stream_timeout_s) {
        message = "no camera";
        return Event::NO_STREAM;
    }
    return Event::NONE;
}

// Silence is a fault; frames that carry no grasp poses are a survey with nothing in view.
Event Pick::checkCollect(std::string &message) {
    size_t have = 0, poseless = 0;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        have     = frames_.size();
        poseless = poseless_;
    }
    if (have >= static_cast<size_t>(c_.frames_needed)) {
        message = "collected";
        return Event::FRAMES_IN;
    }
    if (io_.now() - entered_ < c_.frame_timeout_s) {
        return Event::NONE;
    }
    const size_t enough = static_cast<size_t>(std::max({1, c_.cloud.consensus_min_frames, c_.cloud.vote_min_frames}));
    const std::string count = std::to_string(have) + " with poses";
    if (have >= enough) {
        message = "only " + count;
        return Event::FRAMES_IN;
    }
    if (have + poseless > 0) {
        message = "no handle: " + count + ", " + std::to_string(poseless) + " without";
        return spot_phase_ ? Event::NO_CANDIDATES_SPOT : Event::NO_CANDIDATES_GRASP;
    }
    message = "no frames";
    return Event::FAILURE;
}

Event Pick::process(std::string &message) {
    std::vector<Frame> frames;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        frames.assign(frames_.begin(), frames_.end());
    }
    const double looked  = io_.now();
    const auto   started = std::chrono::steady_clock::now();
    CloudSettings s = c_.cloud;
    if (!spot_phase_) {  // cloud/margin is for where the vehicle may park; from here the arm reaches only this far, plus steering's shift
        s.crop_radius = arm_.reach(arm_.tipDistance()) + c_.track_max_shift + c_.park.link_radius;
    }
    scene_ = processFrames(frames, c_.camera, s);
    if (!frames.empty()) {
        scene_camera_to_arm_ = frames.back().camera_to_arm;
        look_                = frames.back().points;
        look_stamp_          = looked;
    }
    char took[32];
    std::snprintf(took, sizeof(took), " (%.2f s)", std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count());
    detail_ = scene_.summary + took;
    message = std::to_string(scene_.candidates.size()) + " candidates";
    io_.showMap(scene_.map, c_.arm_to_vehicle, spot_phase_);
    io_.showGrasps(scene_, c_.arm_to_vehicle, -1);
    if (scene_.candidates.empty()) {
        return spot_phase_ ? Event::NO_CANDIDATES_SPOT : Event::NO_CANDIDATES_GRASP;
    }
    return spot_phase_ ? Event::CANDIDATES_SPOT : Event::CANDIDATES_GRASP;
}

Event Pick::pickSpot(std::string &message) {
    Joints reported = c_.home;
    double stamp    = 0.0;
    freshJoints(reported, stamp);
    ParkChoice choice;
    try {
        choice = park_.choose(scene_.candidates, scene_.map, arm_.toModel(reported), [this] { return stop_requested_.load(); });
    } catch (const std::invalid_argument &e) {
        message = std::string("the obstacle map is not usable: ") + e.what();
        return Event::FAILURE;
    }
    detail_ = choice.summary;
    char line[120];
    std::snprintf(line, sizeof(line), "move %.2f %.2f %.2f m, %+.0f deg: %d holds", choice.move.x, choice.move.y, choice.move.z,
                  deg(choice.move.yaw), choice.held);
    if (choice.decision == ParkDecision::NOWHERE) {
        message = "no spot holds";
        return Event::NO_SPOT;
    }
    io_.showSpot(choice.move);
    message = choice.decision == ParkDecision::MOVE ? std::string("suggest ") + line + ", grasping from here" : "stay";
    return Event::SPOT_UNCHANGED;
}

Event Pick::pickGrasp(std::string &message) {
    Joints reported;
    double stamp = 0.0;
    if (!freshJoints(reported, stamp)) {
        message = "no fresh joints";
        return Event::FAILURE;
    }
    Joints start = arm_.toModel(reported);
    for (int j = 0; j < JOINT_COUNT; ++j) {
        if (start[j] < arm_.lower(j) - c_.start_tolerance || start[j] > arm_.upper(j) + c_.start_tolerance) {
            message = std::string("outside ") + JOINT_KEYS[j] + " limit";
            return Event::FAILURE;
        }
    }
    start = arm_.clamped(start);
    try {
        grid_.reset(new ObstacleGrid(scene_.map, c_.park.link_radius, bladeRadius(c_.park.blade_step)));
        Collision collision(arm_, *grid_, c_.hull, c_.park.link_step);
        plan_ = planGrasp(scene_.candidates, start, collision, c_.plan, [this] { return stop_requested_.load(); });
    } catch (const std::invalid_argument &e) {
        message = std::string("the obstacle map is not usable: ") + e.what();
        return Event::FAILURE;
    }
    attempted_ = aimed_ = replanned_ = Eigen::Vector3d::Zero();
    near_since_                      = 0.0;
    retargets_                       = 0;
    steer_tries_.clear();
    detail_ = plan_.summary;
    char line[120];
    if (plan_.ok) {
        std::snprintf(line, sizeof(line), "candidate %zu/%zu, %.1f s", plan_.candidate + 1, scene_.candidates.size(), plan_.time_s);
    } else {
        std::snprintf(line, sizeof(line), "no plan to %zu candidates", scene_.candidates.size());
    }
    message = line;
    io_.showGrasps(scene_, c_.arm_to_vehicle, plan_.ok ? static_cast<int>(plan_.candidate) : -1);
    publishPath();
    if (!plan_.ok) {
        return Event::NO_PLAN;
    }
    if (!c_.arm_move) {
        message += ", arm left where it is";
        return Event::PLAN_ONLY;
    }
    follower_.load(plan_.path);
    last_stamp_ = 0.0;
    return Event::PLAN_FOUND;
}

// While the mine is tracked, the goal and the obstacles shift with the tracked target (the scene taken as rigid) and the arm
// is sent to no posture that hits them there. Once the tracked target is track/retarget_m from
// where the arm is heading, or the next posture is blocked, it goes straight to the shifted goal if clear, else plans
// around for track/replan_budget_s. Returns true when the arm must hold: its next posture is blocked and nothing clear was
// found. Shifts past track/max_shift_m are taken as tracking failures.
bool Pick::steer() {
    Eigen::Vector3d offset;
    bool            ok = false;
    {
        std::lock_guard<std::mutex> lock(track_mutex_);
        if (!tracker_.active()) {
            return false;
        }
        ok     = track_.ok;
        offset = scene_camera_to_arm_.linear() * track_.offset;  // the last good shift while a frame finds no consensus
    }
    Joints now;
    double stamp = 0.0;
    if (offset.norm() > c_.track_max_shift) {
        noteSteer("ignoring a tracked shift past max_shift_m");
        return false;
    }
    if (!freshJoints(now, stamp)) {
        return false;
    }
    now = arm_.clamped(arm_.toModel(now));  // the planner takes no start past a limit
    if (offGoal(now) < c_.track_commit) {
        std::lock_guard<std::mutex> lock(track_mutex_);
        tracker_.stop();
        steer_note_ = "committed: open loop to the last goal";
        return false;
    }
    grid_->setQueryToMap(Eigen::Isometry3d(Eigen::Translation3d(-offset)));  // a point near the moved scene, in the map
    Collision     collision(arm_, *grid_, c_.hull, c_.park.link_step);
    Joints        next;
    const Verdict ahead   = follower_.upcoming(next) ? collision.check(next) : Verdict::CLEAR;
    const bool    blocked = ahead == Verdict::OBSTACLE || ahead == Verdict::HULL;
    const char   *hits    = outcomeName(outcomeOf(ahead));
    // Tried once per tracker frame (each moves the offset).
    if (offset == attempted_ || (!blocked && (!ok || (offset - aimed_).norm() < c_.track_retarget))) {
        return blocked;
    }
    attempted_         = offset;
    const auto give_up = [&](const std::string &why) {
        ++steer_tries_[why];
        noteSteer(blocked ? std::string("HOLDING, next posture hits the ") + hits + "; " + why : "staying on course; " + why);
        return blocked;
    };

    GraspPose target = scene_.candidates[plan_.candidate];
    target.point += offset;
    std::vector<GraspGoal> goals;
    const Outcome o = graspGoals(target, 0, plan_.path.back(), arm_.mountDistance() + c_.plan.grasp_point_from_mount, collision, goals);
    if (goals.empty()) {
        return give_up(std::string("no holding posture (") + outcomeName(o) + ")");
    }
    std::sort(goals.begin(), goals.end(), [](const GraspGoal &a, const GraspGoal &b) { return a.swing < b.swing; });
    std::vector<Joints> path;
    Verdict             v = Verdict::CLEAR;
    for (size_t i = 0; i < goals.size() && path.empty(); ++i) {
        v = collision.sweep(now, goals[i].joints, c_.plan.edge_step);
        if (v == Verdict::CLEAR) {
            path = {now, goals[i].joints};
        }
    }
    // Nothing straight: plan around it. The arm waits while that runs: every frame when blocked, else once per retarget_m.
    const char *how = "straight";
    if (path.empty() && (blocked || (offset - replanned_).norm() >= c_.track_retarget)) {
        replanned_      = offset;
        PlanSettings s  = c_.plan;
        s.budget_s      = c_.track_replan_s;
        s.goal_budget_s = 0.5 * c_.track_replan_s;
        const Plan p    = planGrasp({target}, now, collision, s, [this] { return stop_requested_.load(); });
        if (!p.ok) {
            return give_up("replan found no way");
        }
        path = p.path;
        how  = "replanned around";
    }
    if (path.empty()) {
        return give_up(std::string("straight way blocked (") + outcomeName(outcomeOf(v)) + ")");
    }
    ++steer_tries_[std::string(blocked ? "avoided: " : "retargeted: ") + how];
    plan_.path = path;
    aimed_     = offset;
    follower_.load(plan_.path);
    last_stamp_ = 0.0;
    ++retargets_;
    publishPath();
    char line[120];
    std::snprintf(line, sizeof(line), "%s%s to the target shifted %.0f mm", blocked ? "avoided the obstacle, " : "", how, 1000 * offset.norm());
    noteSteer(line);
    return false;
}

// What steering last did, logged when it changes.
void Pick::noteSteer(const std::string &note) {
    if (note != steer_note_) {
        io_.log(Level::DEBUG, "steer: " + note);
    }
    steer_note_ = note;
}

Event Pick::follow(std::string &message) {
    // Steering can hold the arm with no end (e.g. the camera stops mid-hold), so the whole motion has a deadline.
    if (io_.now() - entered_ > c_.motion_timeout_s) {
        io_.standby();
        message = "motion deadline passed, arm released";
        return Event::STALLED;
    }
    const bool hold = c_.track_enabled && c_.track_steer && steer();
    Joints     reported;
    double     stamp = 0.0;
    // Without readings a joint against something goes unnoticed, so the arm is released rather than left pushing blind.
    if (!freshJoints(reported, stamp)) {
        message = io_.standby() ? "joints stale mid-motion, arm released" : "joints stale mid-motion; standby failed, the arm may still be powered";
        return Event::FAILURE;
    }
    if (stamp != last_stamp_) {
        last_stamp_ = stamp;
        const Joints q = arm_.toModel(reported);
        follower_.measure(q);
        const double off = offGoal(q);
        if (off > c_.close_within) {
            near_since_ = 0.0;
        } else if (near_since_ == 0.0) {
            near_since_ = stamp;
        } else if (stamp - near_since_ >= c_.close_hold_s) {
            char line[120];
            std::snprintf(line, sizeof(line), "%.1f mm off for %.1f s", 1000 * off, c_.close_hold_s);
            message = line;
            detail_ = motionReport();
            return Event::REACHED;
        }
    }
    const Event event = advance(hold, message);
    if (event == Event::REACHED || event == Event::COLLIDED) {
        detail_ = motionReport();
    }
    return event;
}

// Sends the next waypoint unless holding, and says how the following is going.
Event Pick::advance(bool hold, std::string &message) {
    Joints          target;
    bool            send  = false;
    const Following state = hold ? Following::SENDING : follower_.tick(io_.now(), target, send);
    if (send) {
        io_.sendTargets(arm_.toReported(target));
    }
    switch (state) {
    case Following::REACHED:
        message = "path end";
        return Event::REACHED;
    case Following::STALLED:
        io_.standby();
        message = "timed out, arm released";
        return Event::STALLED;
    case Following::BLOCKED:
        message = c_.joint_names[follower_.blockedJoint()] + " blocked";
        return Event::COLLIDED;
    default:
        return Event::NONE;
    }
}

// A manual command: plans once from where the arm is, against the limits, the hull and the last look's map, then follows.
Event Pick::manual(std::string &message) {
    Joints reported;
    double stamp = 0.0;
    if (!freshJoints(reported, stamp)) {
        message = !manual_planned_ ? "no fresh joints"
                  : io_.standby()  ? "joints stale mid-motion, arm released"
                                   : "joints stale mid-motion; standby failed, the arm may still be powered";
        return Event::FAILURE;
    }
    if (manual_planned_) {
        if (stamp != last_stamp_) {
            last_stamp_ = stamp;
            follower_.measure(arm_.toModel(reported));
        }
        return advance(false, message);
    }
    bool            is_point = false;
    Eigen::Vector3d point;
    Joints          posture;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        is_point = manual_is_point_;
        point    = manual_point_;
        posture  = manual_posture_;
    }
    const Joints        start = arm_.clamped(arm_.toModel(reported));
    std::vector<Joints> goals;
    Outcome             refused = Outcome::UNREACHABLE;
    try {
        grid_.reset(new ObstacleGrid(scene_.map, c_.park.link_radius, bladeRadius(c_.park.blade_step)));
    } catch (const std::invalid_argument &e) {
        message = std::string("the obstacle map is not usable: ") + e.what();
        return Event::FAILURE;
    }
    Collision  collision(arm_, *grid_, c_.hull, c_.park.link_step);
    const auto offer = [&](const Joints &q) {
        const Outcome o = outcomeOf(collision.check(q));
        refused         = std::max(refused, o);
        if (o == Outcome::OK) {
            goals.push_back(q);
        }
    };
    if (is_point) {
        const double along = arm_.mountDistance() + c_.plan.grasp_point_from_mount;
        for (const bool elbow_up : {false, true}) {
            Joints q;
            const Ik ik = arm_.solve(point, along, elbow_up, start, q);
            if (ik == Ik::SOLVED) {
                offer(q);
            } else if (ik == Ik::JOINT_LIMIT) {
                refused = std::max(refused, Outcome::JOINT_LIMIT);
            }
        }
    } else {
        offer(arm_.toModel(posture));
    }
    plan_ = planTo(goals, start, collision, c_.plan, [this] { return stop_requested_.load(); });
    publishPath();
    if (!plan_.ok) {
        message = std::string("manual command refused: ") + (goals.empty() ? outcomeName(refused) : "no path there");
        return Event::FAILURE;
    }
    detail_ = plan_.summary;
    follower_.load(plan_.path);
    last_stamp_     = 0.0;
    manual_planned_ = true;
    return Event::NONE;
}

double Pick::offGoal(const Joints &q) const {
    const Eigen::Vector3d grasp = arm_.points(q).mount + arm_.axes(q).approach * c_.plan.grasp_point_from_mount;
    return (grasp - (scene_.candidates[plan_.candidate].point + aimed_)).norm();
}

// What the tracker and steering did on the way there.
std::string Pick::motionReport() {
    if (!c_.track_enabled) {
        return "";
    }
    std::lock_guard<std::mutex> lock(track_mutex_);
    char                        line[120];
    std::snprintf(line, sizeof(line), "the tracker saw the target move %.1f mm (%s)", 1000 * track_.offset.norm(),
                  !tracker_.active() ? "open loop" : track_.ok ? "tracking" : "held");
    std::string out = line;
    if (c_.track_steer) {
        const double behind = (scene_camera_to_arm_.linear() * track_.offset - aimed_).norm();
        std::snprintf(line, sizeof(line), "; %d retargets left the goal %.1f mm from it", retargets_, 1000 * behind);
        out += line;
        for (const auto &b : steer_tries_) {
            out += "; " + std::to_string(b.second) + " tries: " + b.first;
        }
    }
    return out;
}

// Settled once the jaw reading holds still; stopping short of closed means it holds something. With the jaw left open,
// where the handle sits between the blades decides instead.
Event Pick::checkJaw(std::string &message) {
    if (!c_.jaw_close) {
        grabbed_ = caught_;
        message  = std::string("jaw left open, ") + (grabbed_ ? "would hold: " : "would miss: ") + caught_how_;
        return Event::JAW_SETTLED;
    }
    double jaw = 0.0, stamp = 0.0;
    {
        std::lock_guard<std::mutex> lock(sensor_mutex_);
        jaw   = jaw_;
        stamp = jaw_stamp_;
    }
    char line[160];
    if (stamp > jaw_closed_at_ && stamp != jaw_last_seen_) {
        jaw_last_seen_ = stamp;
        if (jaw_still_since_ == 0.0 || std::fabs(jaw - jaw_still_) > c_.jaw_settle_tolerance) {
            jaw_still_       = jaw;
            jaw_still_since_ = stamp;
        } else if (stamp - jaw_still_since_ >= c_.jaw_settle_time_s) {
            grabbed_ = jaw > c_.jaw_closed + c_.jaw_grabbed_margin;
            std::snprintf(line, sizeof(line), "%s, jaw %.1f mm", grabbed_ ? "holding" : "empty", jaw * 1000.0);
            message = line;
            return Event::JAW_SETTLED;
        }
    }
    if (io_.now() - entered_ >= c_.jaw_timeout_s) {
        grabbed_ = false;
        message  = "jaw never settled";
        return Event::JAW_SETTLED;
    }
    return Event::NONE;
}

// The handle against the open jaw where the joints put it. The arm hides the handle, so the tracker's last word stands in.
bool Pick::handleCaught() {
    Joints reported;
    double stamp = 0.0;
    if (!freshJoints(reported, stamp)) {
        caught_how_ = "no fresh joints";
        return false;
    }
    const Joints q      = arm_.toModel(reported);
    GraspPose    handle = scene_.candidates[plan_.candidate];
    if (c_.track_enabled) {
        std::lock_guard<std::mutex> lock(track_mutex_);
        handle.point += scene_camera_to_arm_.linear() * track_.offset;
    }
    const HandleFit f = fitHandle(arm_, q, c_.plan.grasp_point_from_mount, handle, c_.cloud.handle_radius);
    miss_             = f.miss;
    char clear[48], line[200];
    if (!std::isfinite(f.clearance)) {
        std::snprintf(clear, sizeof(clear), "not between the blades");
    } else {
        std::snprintf(clear, sizeof(clear), f.clearance >= 0.0 ? "%.1f mm clear of the blades" : "%.1f mm into a blade", 1000 * std::fabs(f.clearance));
    }
    std::snprintf(line, sizeof(line), "tracked bar %.1f mm off the grasp point, crossing %.0f mm deep %+.1f mm aside, %s, leaning %.0f deg",
                  1000 * f.miss, 1000 * f.depth, 1000 * f.side, clear, deg(f.lean));
    caught_how_ = line;
    return f.holds(c_.catch_angle);
}

// ponytail: the bar is judged where it crosses the jaw's mid-plane only; a bar leaning past catch_angle is refused rather than swept
// along the hinge against the blades.
HandleFit fitHandle(const Arm &arm, const Joints &q, double grasp_from_mount, const GraspPose &handle, double radius) {
    const Eigen::Vector3d mount = arm.points(q).mount, bar = handle.bar.normalized();
    const JawAxes         a     = arm.axes(q);
    const Eigen::Vector3d off   = handle.point - (mount + a.approach * grasp_from_mount);
    HandleFit             f;
    f.miss = (off - bar * bar.dot(off)).norm();
    f.lean = std::asin(std::min(1.0, std::fabs(a.closing.dot(bar))));
    const double across = a.hinge.dot(bar);
    if (std::fabs(across) < 1e-6) {
        return f;  // the bar lies in the mid-plane: it runs through the blades, never across them
    }
    const Eigen::Vector3d from  = handle.point - mount;
    const Eigen::Vector3d cross = from - bar * (a.hinge.dot(from) / across);
    f.depth                     = a.approach.dot(cross);
    f.side                      = a.closing.dot(cross);
    double low = 0.0, high = 0.0;
    if (arm.gapAt(f.depth, low, high)) {
        f.clearance = std::min(f.side - radius - low, high - f.side - radius);
    }
    return f;
}

// --- output ---

void Pick::publishState(const std::string &message, State left, double spent) {
    io_.status({state_, detail_.empty() ? message : message + "; " + detail_, park_attempt_, grabbed_, miss_});
    char timing[80] = "";
    if (spent >= 0.0) {
        const bool over = state_ == State::SUCCESS || state_ == State::FAIL || state_ == State::ESTOP;
        std::snprintf(timing, sizeof(timing), over ? "%s -> %s (%.1f s, run %.1f s): " : "%s -> %s (%.1f s): ", stateName(left), stateName(state_),
                      spent, run_started_ == 0.0 ? 0.0 : io_.now() - run_started_);
    } else {
        std::snprintf(timing, sizeof(timing), "%s: ", stateName(state_));
    }
    const Level level = state_ == State::FAIL || state_ == State::ESTOP                                     ? Level::ERROR
                        : state_ == State::RESURVEY || state_ == State::RETARGET || state_ == State::REPARK ? Level::WARN
                                                                                                             : Level::INFO;
    const char *colour = state_ == State::SUCCESS                                   ? "\033[32m"
                         : state_ == State::GOTOGRASP ? "\033[34m"
                                                      : nullptr;
    io_.log(level, colour ? colour + (timing + message) + "\033[0m" : timing + message);
    if (!detail_.empty()) {
        io_.log(Level::DEBUG, detail_);
        detail_.clear();
    }
}

void Pick::publishPath() {
    std::vector<Eigen::Vector3d> tip;
    for (size_t k = 1; k < plan_.path.size(); ++k) {
        for (int n = 0; n <= 20; ++n) {
            tip.push_back(arm_.points(lerp(plan_.path[k - 1], plan_.path[k], n / 20.0)).tip);
        }
    }
    io_.showPath(tip);
}

void Pick::publishBody() {
    Joints reported;
    double stamp = 0.0;
    if (freshJoints(reported, stamp)) {
        Body body;
        arm_.body(arm_.toModel(reported), body, 4);
        io_.showBody(body);
    }
}

}  // namespace final_v1
