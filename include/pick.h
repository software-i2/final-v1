// Copyright by BeeX [2026]
#pragma once

#include <log.h>

#include <follow.h>
#include <fsm.h>
#include <park.h>
#include <plan.h>
#include <track.h>

#include <atomic>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>

namespace final_v1 {

struct PickConfig {
    std::string                          vehicle_frame, arm_frame, camera_frame, camera_source;
    std::array<std::string, JOINT_COUNT> joint_names;
    std::string                          jaw_name;
    double                               jaw_closed = 0.0;
    Joints                               home{};  // reported radians, assumed when no joint readings have arrived
    CameraModel                          camera;
    Eigen::Isometry3d                    camera_to_vehicle = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d                    arm_to_vehicle    = Eigen::Isometry3d::Identity();
    Hull                                 hull;
    CloudSettings                        cloud;
    int                                  frames_needed = 1, approach_column = 0, bar_column = 1;
    double                               frame_timeout_s = 0.0;
    double                               pair_gap = 0.0, pair_slop = 0.0, pair_wait = 0.0;
    ParkSettings                         park;
    PlanSettings                         plan;
    int                                  seed = 0;
    double                               start_tolerance = 0.0;
    FollowSettings                       follow;
    double                               motion_timeout_s = 0.0;  // the whole motion to the handle, steering holds included
    TrackSettings                        track;
    std::vector<MineModel>               mines;  // what the tracker fits, one per mine the pick may meet
    bool                                 track_enabled = false, track_steer = false;
    bool                                 park_skip = false;
    double                               track_retarget = 0.0, track_max_shift = 0.0, track_replan_s = 0.0, track_commit = 0.0;
    double                               loop_hz = 0.0, joint_state_timeout_s = 0.0;
    double                               stream_timeout_s = 0.0, spot_search_s = 0.0, repark_search_s = 0.0;
    int                                  retarget_attempts = 0, park_attempts = 0;
    double                               jaw_settle_tolerance = 0.0, jaw_settle_time_s = 0.0, jaw_grabbed_margin = 0.0, jaw_timeout_s = 0.0;
    double                               close_within = 0.0, close_hold_s = 0.0;
    bool                                 arm_move  = true;   // false: stop at the planned path and never move or home the arm
    bool                                 jaw_close = true;   // false: stop at the handle with the jaw open, for a dry run
    double                               catch_angle = 0.0;
};

// A handle's bar against the open jaw at one posture, taken where the bar crosses the jaw's mid-plane (the one the hinge
// runs through at right angles).
struct HandleFit {
    double miss      = 0.0;  // grasp point to the bar line, metres: the score, whether or not it holds
    double lean      = 0.0;  // bar into the closing line, radians; leaning along the approach still lets the blades squeeze it
    double depth     = 0.0;  // the crossing, metres down the approach from the mount
    double side      = 0.0;  // and along the closing line from the jaw's centre
    double clearance = -std::numeric_limits<double>::infinity();  // handle surface to the nearer blade; below 0 it hits one,
                                                                   // -inf: past the blades or running through them
    bool holds(double max_lean) const { return clearance >= 0.0 && lean <= max_lean; }
};

HandleFit fitHandle(const Arm &arm, const Joints &q, double grasp_from_mount, const GraspPose &handle, double radius);

struct PickStatus {
    State       state = State::READY;
    std::string message;
    uint32_t    park_attempt = 0;  // parking spots suggested this pick
    bool        grabbed      = false;
    double      miss         = std::numeric_limits<double>::quiet_NaN();  // HandleFit::miss when the handle was judged by eye
};

// Everything the pick needs from outside it. pick_node.cpp implements it over ROS1; the pick never sees a message type.
class PickIO {
public:
    virtual ~PickIO() = default;

    virtual double now()                                       = 0;  // seconds, on the clock the sensor stamps are on
    virtual void   log(Level level, const std::string &text) = 0;

    // The arm driver.
    virtual void sendTargets(const Joints &reported) = 0;  // the driver's radians, in joint order
    virtual bool closeJaw(std::string &why)          = 0;
    virtual bool standby()                           = 0;
    virtual bool home(std::string &message)          = 0;  // false when the driver did not answer

    // What the pick shows; nothing here feeds back into it.
    virtual void status(const PickStatus &s)                                                              = 0;
    virtual void showMap(const ObstacleMap &map, const Eigen::Isometry3d &map_to_vehicle, bool park)     = 0;
    virtual void showSpot(const VehiclePose &move)                                                        = 0;  // vehicle frame
    virtual void showGrasps(const Scene &scene, const Eigen::Isometry3d &scene_to_vehicle, int chosen)   = 0;
    virtual void showPath(const std::vector<Eigen::Vector3d> &tip)                                        = 0;  // arm frame; empty wipes it
    virtual void showBody(const Body &body)                                                               = 0;  // arm frame
    virtual void showTrack(const TrackResult &r, const std::vector<Eigen::Vector3d> &trail)               = 0;  // camera frame
    virtual void clearTrack()                                                                             = 0;
};

// Runs the pick after one start: survey, suggest a parking spot, survey again, plan, follow, close the jaw.
// tick() runs on one thread; the sensor and request calls may come from others.
class Pick {
public:
    // Adds the arm's reach to the crop radius and seeds the planner.
    Pick(const PickConfig &c, const Arm &arm, PickIO &io);

    const PickConfig &config() const { return c_; }

    void tick();

    // --- requests ---
    void requestStart(bool steer, bool skip_park);  // each replaces track/steer and park/skip for this run
    void requestStop() { stop_requested_ = true; }
    // Sends the arm home, unless the arm is not to move.
    bool reset(std::string &message);

    // --- sensors, stamps in seconds on io.now()'s clock ---
    void cloudSeen();
    void addFrame(Frame frame);  // its camera_to_arm is filled in here
    void addPoseless();          // a frame that carried no grasp poses
    void joints(const Joints &reported, double stamp);
    void jaw(double position, double stamp);
    bool tracking();
    // Tracks one organised cloud in the camera frame.
    void trackFrame(const std::vector<Eigen::Vector3f> &points);

private:
    bool freshJoints(Joints &q, double &stamp);

    void  apply(Event event, const std::string &message);
    void  enter(State next, std::string message);
    Event step(std::string &message);
    Event retry(double since, double budget_s, bool out_of_tries, Event again, Event done, std::string &message);
    Event checkStream(std::string &message);
    Event checkCollect(std::string &message);
    Event process(std::string &message);
    Event pickSpot(std::string &message);
    Event pickGrasp(std::string &message);
    bool  steer();
    void  noteSteer(const std::string &note);
    double offGoal(const Joints &q) const;
    Event follow(std::string &message);
    std::string motionReport();
    Event checkJaw(std::string &message);
    bool  handleCaught();

    void publishState(const std::string &message, State left = State::READY, double spent = -1.0);
    void publishPath();
    void publishBody();

    PickConfig   c_;
    const Arm   &arm_;
    PickIO      &io_;
    ParkSearch   park_;
    PathFollower follower_;

    std::mutex                   track_mutex_;
    MineTracker                  tracker_;
    TrackResult                  track_;
    std::vector<Eigen::Vector3d> trail_;  // the tracked target over this blind motion
    std::string                  detail_;
    std::string                  steer_note_;                                // what steering last did

    std::mutex        sensor_mutex_;
    std::deque<Frame> frames_;
    size_t            poseless_ = 0;
    double            frame_seen_ = 0.0, joints_stamp_ = 0.0, jaw_stamp_ = 0.0;  // 0: never
    Joints            joints_{};
    double            jaw_ = 0.0;

    std::atomic<bool> start_requested_{false}, stop_requested_{false}, working_{false}, steer_requested_{false}, skip_requested_{false};
    State             state_ = State::READY;
    bool              spot_phase_ = true, grabbed_ = false;
    uint32_t          park_attempt_ = 0, look_attempt_ = 0;
    long              ticks_ = 0;
    // Seconds on io.now()'s clock; 0 is unset.
    double            entered_ = 0.0, run_started_ = 0.0, surveying_since_ = 0.0, reparking_since_ = 0.0, last_stamp_ = 0.0;
    Scene             scene_;
    Plan              plan_;
    Eigen::Isometry3d scene_camera_to_arm_ = Eigen::Isometry3d::Identity();  // the newest frame of the look
    std::vector<Eigen::Vector3f> look_;         // that frame's cloud, which the tracker fits the mine to
    double            look_stamp_          = 0.0;
    Eigen::Vector3d   attempted_           = Eigen::Vector3d::Zero();      // the last tracked shift tried, arm frame
    Eigen::Vector3d   aimed_               = Eigen::Vector3d::Zero();      // the shift the goal actually took
    int               retargets_           = 0;
    Eigen::Vector3d   replanned_           = Eigen::Vector3d::Zero();      // the tracked shift last planned around
    std::map<std::string, int>    steer_tries_;                            // what steering tries came to this motion
    std::unique_ptr<ObstacleGrid> grid_;  // the grasp look's obstacles, inflated once; steering shifts it by a query offset
    double            jaw_closed_at_ = 0.0, jaw_last_seen_ = 0.0, jaw_still_since_ = 0.0, near_since_ = 0.0;
    double            jaw_still_ = 0.0;
    bool              caught_ = false;  // judged as the jaw starts to close, before the scene moves on
    std::string       caught_how_;
    double            miss_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace final_v1
