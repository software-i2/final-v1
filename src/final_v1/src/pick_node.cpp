// Copyright by BeeX [2026]
#include "rosparams.h"

#include <final_v1/PickState.h>
#include <pairs.h>
#include <pick.h>

#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <limits>
#include <mutex>

using namespace final_v1;

namespace {

inline geometry_msgs::Point toPoint(const Eigen::Vector3d &v) {
    geometry_msgs::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

inline geometry_msgs::Quaternion toQuaternion(const Eigen::Quaterniond &q) {
    geometry_msgs::Quaternion out;
    out.x = q.x();
    out.y = q.y();
    out.z = q.z();
    out.w = q.w();
    return out;
}

inline visualization_msgs::Marker marker(const std::string &frame, const std::string &ns, int type, float r, float g, float b,
                                         float a, double size) {
    visualization_msgs::Marker m;
    m.header.frame_id    = frame;
    m.header.stamp       = ros::Time::now();
    m.ns                 = ns;
    m.type               = type;
    m.action             = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.frame_locked       = true;  // follow the frame as the vehicle moves, not where it was when published
    m.scale.x = m.scale.y = m.scale.z = size;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
    return m;
}

// Occupied cells as points in `frame`, with intensity 1 on handle cells.
inline sensor_msgs::PointCloud2 mapCloud(const ObstacleMap &map, const Eigen::Isometry3d &map_to_frame, const std::string &frame) {
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = frame;
    cloud.header.stamp    = ros::Time::now();
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
                                  sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::FLOAT32);
    modifier.resize(map.obstacle.size() + map.handle.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z"), i(cloud, "intensity");
    for (const std::vector<uint32_t> *cells : {&map.obstacle, &map.handle}) {
        for (const uint32_t c : *cells) {
            const Eigen::Vector3d p = map_to_frame * map.box.centre(c);
            *x = static_cast<float>(p.x());
            *y = static_cast<float>(p.y());
            *z = static_cast<float>(p.z());
            *i = cells == &map.handle ? 1.0f : 0.0f;
            ++x;
            ++y;
            ++z;
            ++i;
        }
    }
    return cloud;
}

// Link capsules and blade samples of the collision body.
inline visualization_msgs::MarkerArray bodyMarkers(const Body &body, double link_radius, const std::string &frame) {
    visualization_msgs::MarkerArray out;
    int                             id = 0;
    for (const Segment &s : body.links) {
        visualization_msgs::Marker tube = marker(frame, "links", visualization_msgs::Marker::CYLINDER, 0.2f, 0.6f, 1.0f, 0.35f, 2.0 * link_radius);
        const Eigen::Vector3d      span = s.b - s.a;
        const Eigen::Quaterniond   turn = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), span.norm() > 0.0 ? span : Eigen::Vector3d::UnitZ());
        tube.id                         = id++;
        tube.pose.position              = toPoint(0.5 * (s.a + s.b));
        tube.pose.orientation   = toQuaternion(turn);
        tube.scale.z            = span.norm();
        out.markers.push_back(tube);
    }
    visualization_msgs::Marker blades = marker(frame, "blades", visualization_msgs::Marker::SPHERE_LIST, 0.2f, 0.6f, 1.0f, 0.35f, 0.003);
    for (const Eigen::Vector3d &p : body.blades) {
        blades.points.push_back(toPoint(p));
    }
    out.markers.push_back(blades);
    return out;
}

// The hull footprint as a flat plate at its floor height.
inline visualization_msgs::Marker hullMarker(const Hull &h, const std::string &frame) {
    visualization_msgs::Marker plate = marker(frame, "hull", visualization_msgs::Marker::TRIANGLE_LIST, 0.8f, 0.3f, 0.3f, 0.3f, 1.0);
    const Eigen::Vector3d      c[4]  = {{h.min_x, h.min_y, h.floor_z}, {h.max_x, h.min_y, h.floor_z}, {h.max_x, h.max_y, h.floor_z},
                                        {h.min_x, h.max_y, h.floor_z}};
    for (const int k : {0, 1, 2, 0, 2, 3}) {
        plate.points.push_back(toPoint(c[k]));
    }
    return plate;
}

std::vector<Eigen::Vector3f> cloudPoints(const sensor_msgs::PointCloud2 &cloud) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        points.emplace_back(*x, *y, *z);
    }
    return points;
}

ArmConfig loadArm(Params &robot, const std::string &urdf, PickConfig &c) {
    ArmConfig a;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        const std::string key = JOINT_KEYS[j];
        c.joint_names[j]      = robot.text("arm/joints/" + key);
        const std::vector<double> limits = robot.numbers("arm/limits/" + key, 2);
        a.min[j]    = rad(limits[0]);
        a.max[j]    = rad(limits[1]);
        a.offset[j] = rad(robot.number("arm/offset/" + key));
        a.sign[j]   = robot.number("arm/sign/" + key);
        c.home[j]   = rad(robot.number("arm/home/" + key));
    }
    a.shoulder_clear_of_housing = rad(robot.number("arm/clearance"));
    c.jaw_name   = robot.text("arm/joints/jaw");
    c.jaw_closed = robot.numbers("jaws/limits", 2)[0];
    JawShape &jaw             = a.jaw;
    jaw.open_len              = robot.number("jaws/open");
    jaw.mount_to_throat       = robot.number("jaws/throat");
    jaw.mount_to_tip          = robot.number("jaws/tip");
    jaw.palm_length           = robot.number("jaws/palm");
    jaw.hinge_roll_at_zero    = rad(robot.number("jaws/roll"));
    jaw.blade_rotation_per_m  = robot.number("jaws/swing");
    jaw.hinge_offset_closing  = robot.number("jaws/hinge/closing");
    jaw.hinge_offset_approach = robot.number("jaws/hinge/approach");
    for (const std::vector<double> &r : robot.rows("jaws/profile", 5)) {
        jaw.blades.push_back({r[0], r[1], r[2], r[3], r[4]});
    }
    a.geometry = geometryFromUrdf(urdf, c.joint_names, robot.text("arm/joints/mount"));
    return a;
}

void loadPick(Params &robot, Params &pick, PickConfig &c) {
    c.vehicle_frame = robot.text("frames/vehicle");
    c.arm_frame     = robot.text("frames/arm");
    c.camera_frame  = robot.text("camera/frame");
    c.camera_source = robot.text("camera/source");
    c.camera        = {robot.whole("camera/intrinsics/width"), robot.whole("camera/intrinsics/height"),
                       robot.number("camera/intrinsics/fx"),   robot.number("camera/intrinsics/fy"),
                       robot.number("camera/intrinsics/cx"),   robot.number("camera/intrinsics/cy")};
    c.camera_to_vehicle = fromXyzRpy(robot.vector3("camera/mount/xyz"), robot.vector3("camera/mount/rpy"))
                          * fromXyzRpy(Eigen::Vector3d::Zero(), robot.vector3("camera/optical"));
    c.arm_to_vehicle = fromXyzRpy(robot.vector3("arm/mount/xyz"), robot.vector3("arm/mount/rpy"));
    const std::vector<double> hull_x = robot.numbers("hull/x", 2), hull_y = robot.numbers("hull/y", 2);
    c.hull = {hull_x[0], hull_x[1], hull_y[0], hull_y[1], robot.number("hull/floor")};
    robot.require(c.camera.width > 0 && c.camera.height > 0, "camera/intrinsics", "a positive image size");
    robot.require(c.hull.max_x > c.hull.min_x && c.hull.max_y > c.hull.min_y, "hull", "x and y increasing");

    c.seed                  = pick.whole("plan/seed");
    c.loop_hz               = pick.number("rate");
    c.joint_state_timeout_s = pick.number("stale");
    c.stream_timeout_s      = pick.number("task/stream");
    c.spot_search_s         = pick.number("task/search");
    c.repark_search_s       = pick.number("task/repark");
    c.retarget_attempts     = pick.whole("task/looks");
    c.park_attempts         = pick.whole("task/parks");
    c.jaw_settle_tolerance  = pick.number("jaw/settle");
    c.jaw_settle_time_s     = pick.number("jaw/still");
    c.jaw_grabbed_margin    = pick.number("jaw/margin");
    c.jaw_timeout_s         = pick.number("jaw/timeout");
    c.close_within          = pick.number("jaw/near");
    c.close_hold_s          = pick.number("jaw/hold");
    c.arm_move              = pick.flag("arm/move");
    c.jaw_close             = pick.flag("jaw/close");
    c.catch_angle           = rad(pick.number("jaw/catch"));
    pick.require(c.loop_hz > 0.0 && c.joint_state_timeout_s > 0.0, "rate", "positive, with a positive stale");
    pick.require(c.retarget_attempts > 0 && c.park_attempts > 0, "task", "positive looks and parks");
    pick.require(c.jaw_timeout_s > c.jaw_settle_time_s, "jaw/timeout", "longer than jaw/still");
    pick.require(c.catch_angle > 0.0, "jaw/catch", "positive");

    CloudSettings &s        = c.cloud;
    const int frames        = pick.whole("cloud/frames");
    c.frames_needed         = frames;
    c.frame_timeout_s       = pick.number("cloud/timeout");
    c.pair_gap              = pick.number("cloud/period");
    c.pair_slop             = pick.number("cloud/slop");
    c.pair_wait             = pick.number("cloud/wait");
    s.voxel                 = pick.number("cloud/voxel");
    s.crop_radius           = pick.number("cloud/margin");  // the arm's reach is added once the arm is built
    c.approach_column       = pick.whole("cloud/columns/approach");
    c.bar_column            = pick.whole("cloud/columns/bar");
    s.depth_tolerance       = pick.number("cloud/flying/tolerance");
    s.min_agreeing_neighbours = pick.whole("cloud/flying/neighbours");
    s.slope_window_px       = pick.whole("cloud/flying/window");
    s.min_points_per_voxel  = pick.whole("cloud/occupancy/points");
    s.free_space_tolerance  = pick.number("cloud/occupancy/free");
    const double tilt       = pick.number("cloud/occupancy/tilt");
    s.max_ray_stretch       = 1.0 / std::max(std::cos(rad(tilt)), 1e-3);
    s.handle_radius         = pick.number("cloud/handle");
    s.bar_gap               = pick.number("cloud/gap");
    s.corridor_length       = pick.number("cloud/corridor/length");
    s.corridor_radius       = pick.number("cloud/corridor/radius");
    s.consensus_min_frames  = pick.whole("cloud/consensus/frames");
    s.match                 = pick.number("cloud/consensus/match");
    s.max_axis              = rad(pick.number("cloud/consensus/axis"));
    s.outlier               = pick.number("cloud/consensus/outlier");
    s.max_spread            = pick.number("cloud/consensus/spread");
    s.duplicate             = pick.number("cloud/consensus/duplicate");
    s.vote_min_frames       = pick.whole("cloud/vote/frames");
    s.vote_radius           = pick.whole("cloud/vote/radius");
    pick.require(frames > 0 && s.consensus_min_frames <= frames && s.vote_min_frames <= frames && s.vote_min_frames > 0,
                 "cloud/frames", "positive and at least consensus/frames and vote/frames");
    pick.require(s.voxel > 0.0 && c.frame_timeout_s > 0.0, "cloud/voxel", "positive, with a positive timeout");
    pick.require(c.pair_gap >= 0.0 && c.pair_slop >= 0.0 && c.pair_wait > c.pair_slop, "cloud/wait", "above slop, with period and slop at least 0");
    pick.require(s.slope_window_px >= 3 && s.slope_window_px % 2 == 1, "cloud/flying/window", "odd and at least 3");
    pick.require(tilt >= 0.0 && tilt < 90.0, "cloud/occupancy/tilt", "between 0 and 90");
    pick.require(c.approach_column >= 0 && c.approach_column <= 2 && c.bar_column >= 0 && c.bar_column <= 2
                         && c.bar_column != c.approach_column,
                 "cloud/columns/bar", "0, 1 or 2 and not the approach column");

    ParkSettings &p          = c.park;
    p.box_xy                 = pick.number("park/box/xy");
    p.box_z                  = pick.number("park/box/z");
    p.box_yaw                = rad(pick.number("park/box/yaw"));
    p.coarse_step            = pick.number("park/coarse/step");
    p.coarse_yaw             = rad(pick.number("park/coarse/yaw"));
    p.fine_step              = pick.number("park/fine/step");
    p.fine_yaw               = rad(pick.number("park/fine/yaw"));
    p.refine_count           = pick.whole("park/refine");
    const std::vector<double> standoff = pick.numbers("park/standoff", 2);
    p.standoff_min           = standoff[0];
    p.standoff_max           = standoff[1];
    p.screen_count           = pick.whole("park/screen/count");
    p.screen_budget_s        = pick.number("park/screen/budget");
    p.screen_blade_stride    = pick.whole("park/screen/stride");
    p.exact_count            = pick.whole("park/exact");
    p.transit_samples        = pick.whole("park/transit");
    p.reach_cell             = pick.number("park/cell");
    p.grasp_point_from_mount = pick.number("collision/grasp");
    p.link_radius            = pick.number("collision/radius");
    p.link_step              = pick.number("collision/step");
    p.blade_step             = pick.number("collision/blade");
    p.swing_band             = rad(pick.number("park/band"));
    pick.require(p.fine_step > 0.0 && p.fine_step <= p.coarse_step && p.fine_yaw > 0.0 && p.fine_yaw <= p.coarse_yaw,
                 "park/fine", "positive and no larger than the coarse steps");
    pick.require(p.standoff_min >= 0.0 && p.standoff_min < p.standoff_max, "park/standoff", "increasing and not negative");
    pick.require(p.screen_count > 0 && p.screen_blade_stride > 0 && p.exact_count > 0 && p.transit_samples > 0 && p.refine_count > 0,
                 "park", "positive counts");
    pick.require(p.reach_cell > 0.0, "park/cell", "positive");
    pick.require(p.swing_band >= 0.0, "park/band", "not negative");

    PlanSettings &plan          = c.plan;
    plan.grasp_point_from_mount = p.grasp_point_from_mount;
    plan.budget_s               = pick.number("plan/budget");
    plan.goal_budget_s          = pick.number("plan/goal");
    plan.range                  = rad(pick.number("plan/range"));
    plan.edge_step              = rad(pick.number("plan/step"));
    c.start_tolerance           = rad(pick.number("plan/tolerance"));
    pick.require(plan.goal_budget_s > 0.0 && plan.budget_s >= plan.goal_budget_s && plan.edge_step > 0.0 && plan.range > 0.0,
                 "plan", "positive budget, goal, range and step, with budget at least goal");

    FollowSettings &f          = c.follow;
    f.max_step                 = rad(pick.number("follow/step"));
    f.arrival_tolerance        = rad(pick.number("follow/tolerance"));
    f.arrival_timeout_s        = pick.number("follow/timeout");
    f.blocked_min_step         = rad(pick.number("follow/blocked/step"));
    f.blocked_follow_fraction  = pick.number("follow/blocked/fraction");
    f.blocked_strikes          = pick.whole("follow/blocked/strikes");
    c.motion_timeout_s         = pick.number("follow/deadline");
    pick.require(f.max_step > 0.0 && f.arrival_tolerance > 0.0 && f.blocked_strikes > 0, "follow", "positive step, tolerance and strikes");
    pick.require(c.motion_timeout_s > f.arrival_timeout_s, "follow/deadline", "longer than follow/timeout");
    plan.joint_speed = f.max_step * c.loop_hz;

    c.track_enabled   = pick.flag("track/enabled");
    c.track_steer     = pick.flag("track/steer");
    c.park_skip       = pick.flag("park/skip");
    c.track_retarget  = pick.number("track/retarget");
    c.track_max_shift = pick.number("track/shift");
    c.track_replan_s  = pick.number("track/replan");
    c.track_commit    = pick.number("track/commit");
    pick.require(c.track_commit >= 0.0, "track/commit", "at least 0");
    pick.require(c.track_replan_s > 0.0, "track/replan", "positive");
    pick.require(c.track_retarget > 0.0 && c.track_max_shift > c.track_retarget, "track/shift", "above a positive retarget");
    TrackSettings &t = c.track;
    t.gate           = pick.number("track/gate");
    t.min_fit        = pick.number("track/fit");
    t.min_seen       = pick.number("track/seen");
    t.max_speed      = pick.number("track/speed");
    pick.require(t.gate > 0.0 && t.max_speed > 0.0, "track", "positive gate and speed");
    pick.require(t.min_fit >= 0.0 && t.min_fit < 1.0 && t.min_seen >= 0.0 && t.min_seen < 1.0, "track", "fit and seen at least 0 and below 1");
    for (const std::string &name : pick.names("track/mines")) {
        const std::vector<double> size = pick.numbers("track/mines/" + name, 3);
        pick.require(size[0] > 0.0 && size[1] > 0.0 && size[2] > 0.0, "track/mines/" + name, "a positive face, base and height");
        c.mines.push_back(sampleMine({name, size[0], size[1], size[2]}));
    }
}

class RosPickIO : public PickIO {
public:
    explicit RosPickIO(const PickConfig &c) : c_(c) {
        ros::NodeHandle nh, pnh("~");
        targets_pub_  = nh.advertise<sensor_msgs::JointState>("driver/targets", 1);
        close_jaw_    = nh.serviceClient<std_srvs::Trigger>("driver/close");
        standby_      = nh.serviceClient<std_srvs::Trigger>("driver/standby");
        home_         = nh.serviceClient<std_srvs::Trigger>("driver/home");
        state_pub_    = pnh.advertise<PickState>("state", 1, true);
        park_map_pub_ = pnh.advertise<sensor_msgs::PointCloud2>("park_map", 1, true);
        spot_pub_     = pnh.advertise<visualization_msgs::Marker>("spot", 1, true);
        map_pub_      = pnh.advertise<sensor_msgs::PointCloud2>("pick_map", 1, true);
        grasp_pub_    = pnh.advertise<visualization_msgs::MarkerArray>("grasps", 1, true);
        path_pub_     = pnh.advertise<visualization_msgs::Marker>("path", 1, true);
        body_pub_     = pnh.advertise<visualization_msgs::MarkerArray>("body", 1);
        hull_pub_     = pnh.advertise<visualization_msgs::Marker>("hull", 1, true);
        if (c_.track_enabled) {
            track_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("track", 1, true);
        }
        hull_pub_.publish(hullMarker(c_.hull, c_.arm_frame));
    }

    double now() override { return ros::Time::now().toSec(); }
    void   log(Level level, const std::string &text) override { rosLog("pick", level, text); }

    void sendTargets(const Joints &reported) override {
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time::now();
        msg.name.assign(c_.joint_names.begin(), c_.joint_names.end());
        msg.position.assign(reported.begin(), reported.end());
        targets_pub_.publish(msg);
    }

    bool closeJaw(std::string &why) override {
        std_srvs::Trigger call;
        if (!close_jaw_.call(call)) {
            why = "the driver did not answer";
            return false;
        }
        why = call.response.message;
        return call.response.success;
    }

    bool standby() override {
        std_srvs::Trigger call;
        return standby_.call(call) && call.response.success;
    }

    bool home(std::string &message) override {
        std_srvs::Trigger call;
        if (!home_.call(call)) {
            return false;
        }
        message = call.response.message;
        return true;
    }

    void status(const PickStatus &s) override {
        PickState msg;
        msg.state        = stateName(s.state);
        msg.message      = s.message;
        msg.park_attempt = s.park_attempt;
        msg.grabbed      = s.grabbed;
        msg.miss_m       = s.miss;
        state_pub_.publish(msg);
    }

    void showMap(const ObstacleMap &map, const Eigen::Isometry3d &map_to_vehicle, bool park) override {
        (park ? park_map_pub_ : map_pub_).publish(mapCloud(map, map_to_vehicle, c_.vehicle_frame));
    }

    // Where parking would put the vehicle, as an arrow from here; the pilot drives, the pick does not.
    void showSpot(const VehiclePose &move) override {
        visualization_msgs::Marker m = marker(c_.vehicle_frame, "spot", visualization_msgs::Marker::ARROW, 0.2f, 0.6f, 1.0f, 1.0f, 0.03);
        m.scale.x                    = 0.15;
        m.pose.position              = toPoint(Eigen::Vector3d(move.x, move.y, move.z));
        m.pose.orientation           = toQuaternion(Eigen::Quaterniond(Eigen::AngleAxisd(move.yaw, Eigen::Vector3d::UnitZ())));
        spot_pub_.publish(m);
    }

    // Vision's poses, the candidates, and the chosen one if any, in the vehicle frame.
    void showGrasps(const Scene &scene, const Eigen::Isometry3d &scene_to_vehicle, int chosen) override {
        visualization_msgs::MarkerArray out;
        visualization_msgs::Marker      poses = marker(c_.vehicle_frame, "poses", visualization_msgs::Marker::SPHERE_LIST, 0.6f, 0.6f, 0.6f, 1.0f, 0.004);
        visualization_msgs::Marker      spots = marker(c_.vehicle_frame, "candidates", visualization_msgs::Marker::SPHERE_LIST, 1.0f, 0.8f, 0.1f, 1.0f, 0.007);
        for (const GraspPose &g : scene.poses) {
            poses.points.push_back(toPoint(scene_to_vehicle * g.point));
        }
        for (const GraspPose &g : scene.candidates) {
            spots.points.push_back(toPoint(scene_to_vehicle * g.point));
        }
        visualization_msgs::Marker pick = marker(c_.vehicle_frame, "chosen", visualization_msgs::Marker::LINE_LIST, 0.1f, 1.0f, 0.3f, 1.0f, 0.003);
        if (chosen >= 0) {
            const GraspPose &g = scene.candidates[chosen];
            pick.points.push_back(toPoint(scene_to_vehicle * Eigen::Vector3d(g.point - 0.03 * g.bar.normalized())));
            pick.points.push_back(toPoint(scene_to_vehicle * Eigen::Vector3d(g.point + 0.03 * g.bar.normalized())));
        } else {
            pick.action = visualization_msgs::Marker::DELETE;
        }
        out.markers = {poses, spots, pick};
        grasp_pub_.publish(out);
    }

    void showPath(const std::vector<Eigen::Vector3d> &tip) override {
        visualization_msgs::Marker line = marker(c_.arm_frame, "path", visualization_msgs::Marker::LINE_STRIP, 0.1f, 1.0f, 0.3f, 1.0f, 0.002);
        for (const Eigen::Vector3d &p : tip) {
            line.points.push_back(toPoint(p));
        }
        if (line.points.empty()) {
            line.action = visualization_msgs::Marker::DELETE;
        }
        path_pub_.publish(line);
    }

    void showBody(const Body &body) override { body_pub_.publish(bodyMarkers(body, c_.park.link_radius, c_.arm_frame)); }

    // From where the arm is aiming to where the tracker puts the target: the correction a closed loop would make.
    void showTrack(const TrackResult &r, const std::vector<Eigen::Vector3d> &trail) override {
        visualization_msgs::Marker shift = marker(c_.camera_frame, "shift", visualization_msgs::Marker::ARROW, 1.0f, 0.3f, 0.9f, 1.0f, 0.003);
        shift.scale.y = 0.007, shift.scale.z = 0.01;  // shaft and head width, head length
        shift.points  = {toPoint(r.target - r.offset), toPoint(r.target)};
        visualization_msgs::Marker line = marker(c_.camera_frame, "trail", visualization_msgs::Marker::LINE_STRIP, 0.3f, 0.9f, 1.0f, 0.8f, 0.0015);
        for (const Eigen::Vector3d &p : trail) {
            line.points.push_back(toPoint(p));
        }
        visualization_msgs::Marker ball = marker(c_.camera_frame, "target", visualization_msgs::Marker::SPHERE, 1.0f, 0.55f, 0.0f, 1.0f, 0.012);
        ball.pose.position              = toPoint(r.target);
        visualization_msgs::Marker on  = marker(c_.camera_frame, "mine on the cloud", visualization_msgs::Marker::POINTS, 0.2f, 0.8f, 0.2f, 1.0f, 0.003);
        visualization_msgs::Marker off = marker(c_.camera_frame, "mine off the cloud", visualization_msgs::Marker::POINTS, 0.9f, 0.2f, 0.2f, 1.0f, 0.003);
        for (size_t i = 0; i < r.shown.size(); ++i) {
            (r.matched[i] ? on : off).points.push_back(toPoint(r.shown[i].cast<double>()));
        }
        visualization_msgs::MarkerArray markers;
        markers.markers = {shift, line, ball, on, off};
        track_pub_.publish(markers);
    }

    void clearTrack() override {
        visualization_msgs::MarkerArray wipe;
        wipe.markers.push_back(marker(c_.camera_frame, "", visualization_msgs::Marker::ARROW, 0, 0, 0, 0, 0));
        wipe.markers[0].action = visualization_msgs::Marker::DELETEALL;
        track_pub_.publish(wipe);
    }

private:
    PickConfig         c_;
    ros::Publisher     targets_pub_, state_pub_, park_map_pub_, spot_pub_, map_pub_, grasp_pub_, path_pub_, body_pub_, hull_pub_, track_pub_;
    ros::ServiceClient close_jaw_, standby_, home_;
};

class PickNode {
public:
    PickNode(const PickConfig &config, const Arm &arm)
            : io_(config),
              pick_(config, arm, io_),
              pairs_(config.pair_gap, config.pair_slop, config.pair_wait) {
        ros::NodeHandle pnh("~");
        cloud_sub_  = nh_.subscribe("cloud", 1, &PickNode::onCloud, this);
        poses_sub_  = nh_.subscribe("poses", 10, &PickNode::onPoses, this);
        joints_sub_ = nh_.subscribe("joint_states", 1, &PickNode::onJoints, this);
        services_   = {pnh.advertiseService("start", &PickNode::onStart, this), pnh.advertiseService("stop", &PickNode::onStop, this),
                       pnh.advertiseService("reset", &PickNode::onReset, this)};
    }

    const PickConfig &config() const { return pick_.config(); }
    void              tick() { pick_.tick(); }

private:
    // The cloud's points in camera_link on the pixel grid. The vehicle's own frame for this camera looks along +z with y down.
    bool cameraPoints(const sensor_msgs::PointCloud2 &cloud, std::vector<Eigen::Vector3f> &points) const {
        const PickConfig &c      = pick_.config();
        const bool        source = cloud.header.frame_id == c.camera_source;
        if (!source && cloud.header.frame_id != c.camera_frame) {
            return false;
        }
        points = cloudPoints(cloud);
        if (source) {
            for (Eigen::Vector3f &p : points) {
                p = Eigen::Vector3f(p.x(), -p.y(), -p.z());
            }
        }
        if (source || points.size() != static_cast<size_t>(c.camera.width) * c.camera.height) {
            points = onPixelGrid(points, c.camera);
        }
        return true;
    }

    void handOut() {
        sensor_msgs::PointCloud2::ConstPtr   cloud;
        geometry_msgs::PoseArray::ConstPtr   poses;
        for (;;) {
            {
                std::lock_guard<std::mutex> lock(pairs_mutex_);
                if (!pairs_.next(cloud, poses)) {
                    return;
                }
            }
            onFrame(cloud, poses);
        }
    }

    void onCloud(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
        pick_.cloudSeen();
        {
            std::lock_guard<std::mutex> lock(pairs_mutex_);
            if (!pairs_.addCloud(static_cast<int64_t>(cloud->header.stamp.toNSec()), cloud)) {
                return;
            }
        }
        std::vector<Eigen::Vector3f> points;
        if (pick_.tracking() && cameraPoints(*cloud, points)) {
            pick_.trackFrame(points);
        }
        handOut();
    }

    void onPoses(const geometry_msgs::PoseArray::ConstPtr &poses) {
        {
            std::lock_guard<std::mutex> lock(pairs_mutex_);
            pairs_.addPoses(static_cast<int64_t>(poses->header.stamp.toNSec()), poses);
        }
        handOut();
    }

    void onFrame(const sensor_msgs::PointCloud2::ConstPtr &cloud, const geometry_msgs::PoseArray::ConstPtr &poses) {
        const PickConfig &c = pick_.config();
        Frame             frame;
        if (poses->header.frame_id != cloud->header.frame_id || !cameraPoints(*cloud, frame.points)) {
            ROS_WARN_THROTTLE(5.0, "[pick] frame dropped: cloud in %s, poses in %s; expected both in %s or %s", cloud->header.frame_id.c_str(),
                              poses->header.frame_id.c_str(), c.camera_frame.c_str(), c.camera_source.c_str());
            return;
        }
        if (poses->poses.empty()) {
            pick_.addPoseless();
            return;
        }
        const Eigen::Vector3d flip = cloud->header.frame_id == c.camera_source ? Eigen::Vector3d(1.0, -1.0, -1.0) : Eigen::Vector3d::Ones();
        for (const geometry_msgs::Pose &p : poses->poses) {
            const Eigen::Matrix3d r = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).normalized().toRotationMatrix();
            frame.poses.push_back({Eigen::Vector3d(p.position.x, p.position.y, p.position.z).cwiseProduct(flip),
                                   r.col(c.bar_column).cwiseProduct(flip), r.col(c.approach_column).cwiseProduct(flip)});
        }
        pick_.addFrame(std::move(frame));
    }

    void onJoints(const sensor_msgs::JointState::ConstPtr &msg) {
        const PickConfig &c  = pick_.config();
        const auto        at = [&](const std::string &name) {
            const size_t i = std::find(msg->name.begin(), msg->name.end(), name) - msg->name.begin();
            return i < msg->name.size() && i < msg->position.size() ? msg->position[i] : std::numeric_limits<double>::quiet_NaN();
        };
        Joints q;
        for (int j = 0; j < JOINT_COUNT; ++j) {
            q[j] = at(c.joint_names[j]);
        }
        if (std::all_of(q.begin(), q.end(), [](double v) { return std::isfinite(v); })) {
            pick_.joints(q, msg->header.stamp.toSec());
        }
        const double jaw = at(c.jaw_name);
        if (std::isfinite(jaw)) {
            pick_.jaw(jaw, msg->header.stamp.toSec());
        }
    }

    bool onStart(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        bool steer = pick_.config().track_steer;
        ros::param::get("~track/steer", steer);
        bool skip = pick_.config().park_skip;
        ros::param::get("~park/skip", skip);
        pick_.requestStart(steer, skip);
        res.success = true;
        res.message = "start requested";
        return true;
    }

    bool onStop(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        pick_.requestStop();
        res.success = true;
        res.message = "stop requested, the arm will be released";
        return true;
    }

    bool onReset(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        res.success = pick_.reset(res.message);
        return true;
    }

    RosPickIO io_;
    Pick      pick_;

    ros::NodeHandle                                                                        nh_;
    std::mutex                                                                             pairs_mutex_;
    FramePairs<sensor_msgs::PointCloud2::ConstPtr, geometry_msgs::PoseArray::ConstPtr>     pairs_;
    ros::Subscriber                                                                        cloud_sub_, poses_sub_, joints_sub_;
    std::vector<ros::ServiceServer>                                                        services_;
};

int run(int argc, char **argv) {
    ros::init(argc, argv, "pick");
    Params     robot("/robot", {"frames", "arm", "jaws", "camera", "hull"});
    Params     pick("/pick");
    PickConfig config;
    loadPick(robot, pick, config);

    std::string urdf;
    if (!ros::param::get("/robot_description", urdf)) {
        ROS_ERROR("[pick] /robot_description is not set");
        return 1;
    }
    std::unique_ptr<Arm> arm;
    try {
        arm.reset(new Arm(loadArm(robot, urdf, config), config.park.blade_step));
    } catch (const std::invalid_argument &e) {
        ROS_ERROR("[pick] the arm is not usable: %s", e.what());
        return 1;
    }
    const std::string problems = robot.problems() + pick.problems();
    if (!problems.empty()) {
        ROS_ERROR("[pick] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    PickNode node(config, *arm);
    std::string mines;
    for (const MineModel &m : node.config().mines) {
        mines += (mines.empty() ? "" : ", ") + m.name;
    }
    ROS_INFO("[pick] ready, crop %.2f m, tracking mines: %s", node.config().cloud.crop_radius, mines.c_str());
    ros::AsyncSpinner spinner(2);
    spinner.start();
    for (ros::Rate rate(node.config().loop_hz); ros::ok(); rate.sleep()) {
        node.tick();
    }
    return 0;
}

}  // namespace

int main(int argc, char **argv) { return run(argc, argv); }
