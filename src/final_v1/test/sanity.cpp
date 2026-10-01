// Copyright by BeeX [2026]
// Sanity: each part of the pick gives the right answer.
#include <bpl.h>
#include <driver.h>
#include <follow.h>
#include <fsm.h>
#include <park.h>
#include <pairs.h>
#include <pick.h>

#include "common.h"

#include <gtest/gtest.h>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <random>

using namespace final_v1;

namespace {

// The shipped arm with no calibration offsets, so it can be compared with the URDF directly.
ArmConfig testArm(const std::string &urdf) {
    ArmConfig c;
    c.geometry = geometryFromUrdf(urdf, kNames, "ee_joint");
    c.sign     = {{1, 1, 1, -1}};
    c.min      = {{0.0, 0.0, 0.0, 0.0}};
    c.max      = {{rad(349.6), rad(200.0), rad(184.5), rad(184.5)}};
    c.jaw.mount_to_tip = 0.0969;
    c.jaw.open_len     = 0.007;
    c.jaw.blades       = {{0.0, 0.05, -0.02, 0.005, 0.005}};
    return c;
}

Joints randomJoints(const Arm &arm, std::mt19937 &rng) {
    Joints q;
    for (int j = 0; j < JOINT_COUNT; ++j) {
        q[j] = std::uniform_real_distribution<double>(arm.lower(j), arm.upper(j))(rng);
    }
    return q;
}

}  // namespace

// Key state transitions; START is ignored mid-pick but restarts from ESTOP.
TEST(Fsm, WalksThePickAndLatchesStops) {
    EXPECT_EQ(nextState(State::READY, Event::START), State::STREAM);
    EXPECT_EQ(nextState(State::STREAM, Event::STREAMING), State::COLLECT);
    EXPECT_EQ(nextState(State::COLLECT, Event::NO_CANDIDATES_SPOT), State::RESURVEY);
    EXPECT_EQ(nextState(State::PICKSPOT, Event::SPOT_UNCHANGED), State::COLLECT);
    EXPECT_EQ(nextState(State::GOTOGRASP, Event::STALLED), State::RETARGET);
    EXPECT_EQ(nextState(State::GOTOGRASP, Event::COLLIDED), State::ESTOP);
    EXPECT_EQ(nextState(State::PICKGRASP, Event::START), State::PICKGRASP);
    EXPECT_EQ(nextState(State::PICKGRASP, Event::PLAN_ONLY), State::SUCCESS);
    EXPECT_EQ(nextState(State::READY, Event::FAILURE), State::READY);
    EXPECT_EQ(nextState(State::ESTOP, Event::START), State::STREAM);
    EXPECT_EQ(nextState(State::GOTOGRASP, Event::GOTO), State::MANUAL);
    EXPECT_EQ(nextState(State::ESTOP, Event::GOTO), State::MANUAL);
    EXPECT_EQ(nextState(State::MANUAL, Event::REACHED), State::READY);
    EXPECT_EQ(nextState(State::MANUAL, Event::STOP), State::ESTOP);
}

// The follower reaches its goal, and flags a joint whose fresh readings stop moving.
TEST(Follower, ReachesAndCatchesAStuckJoint) {
    FollowSettings s{rad(1.0), rad(0.5), 5.0, rad(0.2), 0.2, 3};
    PathFollower   f(s);
    const Joints   start{}, goal{{rad(10), 0, 0, 0}};
    Joints         target;
    bool           send = false;

    f.load({start, goal});
    Following state = Following::SENDING;
    for (int t = 0; t < 100 && state != Following::REACHED; ++t) {
        state = f.tick(t * 0.05, target, send);
        if (send) {
            f.measure(target);
        }
    }
    EXPECT_EQ(state, Following::REACHED);

    // The same reading judged every tick must not count as a joint that stopped.
    f.load({start, goal});
    f.measure(start);
    for (int t = 0; t < 30; ++t) {
        EXPECT_NE(f.tick(t * 0.05, target, send), Following::BLOCKED);
    }

    // New readings that never move are a joint against something.
    f.load({start, goal});
    for (int t = 0; t < 30 && state != Following::BLOCKED; ++t) {
        f.measure(start);
        state = f.tick(t * 0.05, target, send);
    }
    EXPECT_EQ(state, Following::BLOCKED);
    EXPECT_EQ(f.blockedJoint(), BASE);
}

// A joint jammed against something: the follower stops advancing the target at once, then calls it blocked.
// Before the fix it kept sending waypoints for every strike, pushing ~10 deg past where the arm actually was.
TEST(Follower, AJammedJointIsNotPushedFurther) {
    FollowSettings s{rad(0.5), rad(1.5), 15.0, rad(0.2), 0.2, 10};  // follow/ in config/pick.yaml
    PathFollower   f(s);
    const Joints   start{}, goal{{rad(30), 0, 0, 0}};
    Joints         target, furthest = start;
    bool           send  = false;
    Following      state = Following::SENDING;

    f.load({start, goal});
    for (int t = 0; t < 200 && state != Following::BLOCKED; ++t) {
        f.measure(start);  // the arm never moves
        state = f.tick(t * 0.05, target, send);
        if (send) {
            furthest = target;
        }
    }
    EXPECT_EQ(state, Following::BLOCKED);
    EXPECT_LE(deg(furthest[BASE]), 1.5) << "the target ran ahead of the jammed arm";  // two 0.5 deg steps before the first strike
}

// A joint that sticks for a moment and then moves again is not a collision: the follower waits, then carries on.
TEST(Follower, ABrieflyStuckJointStillArrives) {
    FollowSettings s{rad(0.5), rad(1.5), 15.0, rad(0.2), 0.2, 10};
    PathFollower   f(s);
    const Joints   start{}, goal{{rad(30), 0, 0, 0}};
    Joints         target, sent = start, arm = start;
    bool           send  = false;
    Following      state = Following::SENDING;

    f.load({start, goal});
    for (int t = 0; t < 400 && state != Following::REACHED && state != Following::BLOCKED; ++t) {
        f.measure(arm);
        state = f.tick(t * 0.05, target, send);
        if (send) {
            sent = target;
        }
        if (t < 20 || t > 25) {  // the arm reaches the last target each tick, except while stuck for ticks 20-25
            arm = sent;
        }
    }
    EXPECT_EQ(state, Following::REACHED);
}

// The largest move sets the step count; every joint covers its share per step, so all arrive on the same tick.
TEST(Follower, JointsArriveTogether) {
    FollowSettings s{rad(0.5), rad(0.5), 5.0, rad(0.2), 0.2, 3};
    PathFollower   f(s);
    const Joints   start{}, goal{{rad(10), rad(-4), rad(1), 0}};
    Joints         target, previous = start;
    bool           send = false;

    f.load({start, goal});
    int sent = 0;
    for (int t = 0; t < 100; ++t) {
        f.tick(t * 0.05, target, send);
        if (!send) {
            break;
        }
        ++sent;
        for (int j = 0; j < JOINT_COUNT; ++j) {
            EXPECT_NEAR(target[j] - previous[j], goal[j] / 20, 1e-9);  // 10 deg / 0.5 deg = 20 steps
        }
        previous = target;
    }
    EXPECT_EQ(sent, 20);
    for (int j = 0; j < JOINT_COUNT; ++j) {
        EXPECT_NEAR(target[j], goal[j], 1e-9);
    }
}

class StillArm : public Actuators {
public:
    bool position(uint8_t, float &value) override {
        value = static_cast<float>(rad(90.0));
        return true;
    }
    bool command(uint8_t, float) override { return true; }
    bool standby(uint8_t) override { return true; }
};

// The driver moves a joint only to a target inside its limits and close to where the joint is.
TEST(Driver, RefusesTargetsItCannotSafelyReach) {
    std::array<Axis, AXES> axes;
    const char            *names[AXES] = {"base", "shoulder", "elbow", "wrist", "jaw"};
    for (int a = 0; a < AXES; ++a) {
        axes[a] = {names[a], static_cast<uint8_t>(a + 1), 0.0, rad(300.0), rad(90.0), rad(20.0), 1.0};
    }
    Driver      driver(axes, std::make_shared<StillArm>(), 10.0, rad(5.0), [](Level, const std::string &) {});
    std::string why;

    EXPECT_FALSE(driver.moveTo({"base"}, {rad(91.0)}, why)) << "no reading yet, so nowhere is known to be close";
    AxisReadings r;
    ASSERT_TRUE(driver.poll(1.0, r));
    EXPECT_TRUE(driver.moveTo({"base"}, {rad(94.0)}, why)) << why;
    EXPECT_FALSE(driver.moveTo({"base"}, {rad(120.0)}, why)) << "a 30 deg jump";
    EXPECT_FALSE(driver.moveTo({"base"}, {rad(-1.0)}, why)) << "outside the limits";
    EXPECT_FALSE(driver.moveTo({"base"}, {NAN}, why)) << "not a number";
    EXPECT_FALSE(driver.moveTo({"base", "elbow"}, {rad(92.0), rad(150.0)}, why)) << "one bad target drops them all";
}

// BPL packets survive encode/decode back to back, and a flipped bit is dropped.
TEST(Bpl, FramesRoundTripAndRejectCorruption) {
    std::vector<uint8_t> stream = bpl::encode(5, bpl::POSITION, 1.5f);
    EXPECT_EQ(stream.back(), 0x00);
    EXPECT_EQ(std::count(stream.begin(), stream.end(), 0x00), 1);
    const std::vector<uint8_t> second = bpl::encode(2, bpl::MODE, std::vector<uint8_t>{0x00, 0x00});
    stream.insert(stream.end(), second.begin(), second.end());

    bpl::Reader              reader;
    std::vector<bpl::Packet> got = reader.feed(stream.data(), stream.size());
    ASSERT_EQ(got.size(), 2u);
    float value = 0.0f;
    std::memcpy(&value, got[0].data.data(), sizeof(float));
    EXPECT_EQ(got[0].device, 5);
    EXPECT_FLOAT_EQ(value, 1.5f);
    EXPECT_EQ(got[1].data, (std::vector<uint8_t>{0x00, 0x00}));

    std::vector<uint8_t> broken = bpl::encode(5, bpl::POSITION, 1.5f);
    broken[2] ^= 0x01;
    EXPECT_TRUE(reader.feed(broken.data(), broken.size()).empty());
}

// Our closed-form FK agrees with KDL on the URDF at 1000 random postures.
TEST(Arm, ForwardKinematicsMatchesTheUrdf) {
    const std::string urdf = expandedUrdf();
    ASSERT_FALSE(urdf.empty());
    KDL::Tree  tree;
    KDL::Chain chain;
    ASSERT_TRUE(kdl_parser::treeFromString(urdf, tree));
    ASSERT_TRUE(tree.getChain("arm_base", "ee_base_link", chain));
    ASSERT_EQ(chain.getNrOfJoints(), 4u);
    KDL::ChainFkSolverPos_recursive fk(chain);

    const Arm    arm(testArm(urdf), 0.005);
    std::mt19937 rng(7);
    for (int n = 0; n < 1000; ++n) {
        const Joints    reported = arm.toReported(randomJoints(arm, rng));
        KDL::JntArray   q(4);
        KDL::Frame      mount;
        for (int j = 0; j < JOINT_COUNT; ++j) {
            q(j) = reported[j];
        }
        ASSERT_GE(fk.JntToCart(q, mount), 0);
        const Joints          model = arm.toModel(reported);
        const Eigen::Vector3d ours  = arm.points(model).mount;
        const Eigen::Vector3d axis  = arm.axes(model).approach;
        EXPECT_NEAR((ours - Eigen::Vector3d(mount.p.x(), mount.p.y(), mount.p.z())).norm(), 0.0, 1e-5);
        EXPECT_NEAR(axis.dot(Eigen::Vector3d(mount.M.UnitZ().x(), mount.M.UnitZ().y(), mount.M.UnitZ().z())), 1.0, 1e-6);
    }
}

// Reach-back postures are left out on purpose: the IK never proposes them, they jam the upper arm on the housing.
TEST(Arm, InverseKinematicsRecoversEveryFrontFacingPosture) {
    const Arm    arm(testArm(expandedUrdf()), 0.005);
    const double along = arm.mountDistance() + 0.06;
    std::mt19937 rng(11);
    int          front = 0;
    for (int n = 0; n < 1000; ++n) {
        const Joints          q      = randomJoints(arm, rng);
        const Eigen::Vector3d target = arm.points(q).wrist + arm.axes(q).approach * along;
        if (target.x() * std::cos(q[BASE]) + target.y() * std::sin(q[BASE]) < 0.0) {
            continue;
        }
        ++front;
        bool matched = false;
        for (const bool up : {false, true}) {
            Joints out;
            if (arm.solve(target, along, up, q, out) == Ik::SOLVED) {
                matched |= (arm.points(out).wrist + arm.axes(out).approach * along - target).norm() < 1e-6;
            }
        }
        EXPECT_TRUE(matched) << "posture " << n;
    }
    EXPECT_GT(front, 200);
}

// Links and blades keep their clearances from obstacles; blades may touch only the handle's own cells.
TEST(Collision, InflatesObstaclesAndKeepsHandleExact) {
    ObstacleMap map;
    map.box.voxel  = 0.01;
    map.box.nx     = map.box.ny = map.box.nz = 21;
    map.obstacle   = {static_cast<uint32_t>(map.box.index(10, 10, 10))};
    map.handle     = {static_cast<uint32_t>(map.box.index(2, 2, 2))};
    ObstacleGrid grid(map, 0.03, 0.015);
    const Eigen::Vector3d obstacle = map.box.centre(map.box.index(10, 10, 10));
    EXPECT_TRUE(grid.linkBlocked(obstacle + Eigen::Vector3d(0.03, 0, 0)));
    EXPECT_FALSE(grid.linkBlocked(obstacle + Eigen::Vector3d(0.045, 0, 0)));
    EXPECT_TRUE(grid.bladeBlocked(obstacle + Eigen::Vector3d(0.01, 0, 0)));
    EXPECT_FALSE(grid.bladeBlocked(obstacle + Eigen::Vector3d(0.03, 0, 0)));
    const Eigen::Vector3d handle = map.box.centre(map.box.index(2, 2, 2));
    EXPECT_TRUE(grid.bladeBlocked(handle));
    EXPECT_FALSE(grid.bladeBlocked(handle + Eigen::Vector3d(0.01, 0, 0)));
    EXPECT_TRUE(grid.linkBlocked(handle + Eigen::Vector3d(0.02, 0, 0)));
}

// The reach map never rules out a point the IK can reach, so parking never skips a good spot.
TEST(Park, ReachMapNeverRefusesWhatTheIkSolves) {
    const Arm      arm(testArm(expandedUrdf()), 0.005);
    const double   along = arm.mountDistance() + 0.06;
    const ReachMap reach(arm, along, -0.05, 0.002);
    std::mt19937   rng(3);
    std::uniform_real_distribution<double> coord(-0.45, 0.45);
    int solvable = 0;
    for (int n = 0; n < 20000; ++n) {
        const Eigen::Vector3d p(coord(rng), coord(rng), coord(rng));
        Joints                q;
        const bool solved = arm.solve(p, along, false, Joints{}, q) == Ik::SOLVED || arm.solve(p, along, true, Joints{}, q) == Ik::SOLVED;
        if (solved && p.z() > -0.05) {
            ++solvable;
            EXPECT_TRUE(reach.reachable(std::hypot(p.x(), p.y()), p.z()) && reach.baseAngleFor(std::atan2(p.y(), p.x())));
        }
    }
    EXPECT_GT(solvable, 500);
}

// The limpet rendered from its shape, drifting and tilting, first seen three frames after the look, beside a squashed decoy:
// the tracker must choose the limpet and keep the handle point on it.
TEST(Track, ChoosesTheMineThatFitsAndFollowsItFromTheLook) {
    const CameraModel camera{320, 240, 250.0, 250.0, 160.0, 120.0};
    const MineShape   limpet{"limpet", 0.205, 0.350, 0.115};
    const MineModel   surface = sampleMine(limpet, 0.002), decoy = sampleMine({"decoy", 0.205, 0.350, 0.6 * 0.115});
    const auto pose = [](int k) {
        Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
        p.linear()      = (Eigen::AngleAxisd(rad(10.0 + 0.2 * k), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX())
                      * Eigen::AngleAxisd(rad(20.0), Eigen::Vector3d::UnitY())).toRotationMatrix();
        p.translation() = Eigen::Vector3d(0.02, -0.01, -0.6) + 0.001 * k * Eigen::Vector3d(1.5, -1.0, 0.5);
        return p;
    };
    const auto render = [&](int k) {
        std::vector<Eigen::Vector3f> cloud(static_cast<size_t>(camera.width) * camera.height, Eigen::Vector3f::Constant(NAN));
        for (const Eigen::Vector3f &s : surface.points) {
            const Eigen::Vector3d q = pose(k) * s.cast<double>();
            const int u = static_cast<int>(camera.cx + camera.fx * q.x() / -q.z()), v = static_cast<int>(camera.cy - camera.fy * q.y() / -q.z());
            for (int dv = 0; dv < 2; ++dv) {
                for (int du = 0; du < 2; ++du) {
                    if (u + du >= 0 && v + dv >= 0 && u + du < camera.width && v + dv < camera.height) {
                        Eigen::Vector3f &at = cloud[static_cast<size_t>(v + dv) * camera.width + u + du];
                        if (!(at.z() > static_cast<float>(q.z()))) {
                            at = q.cast<float>();
                        }
                    }
                }
            }
        }
        return cloud;
    };
    const Eigen::Vector3d handle(0.15, -0.07, 0.0);  // beside the side, in the mine's frame
    MineTracker           tracker({decoy, sampleMine(limpet)}, TrackSettings{0.01, 0.55, 0.3, 0.1}, camera);
    EXPECT_FALSE(tracker.start(std::vector<Eigen::Vector3f>(static_cast<size_t>(camera.width) * camera.height, Eigen::Vector3f::Constant(NAN)),
                               pose(0) * handle, 0.0));
    ASSERT_TRUE(tracker.start(render(0), pose(0) * handle, 0.0));
    EXPECT_EQ(tracker.mine(), "limpet");
    TrackResult r;
    for (int k = 3; k <= 20; ++k) {
        r = tracker.update(render(k), 0.1 * k);
        ASSERT_TRUE(r.ok) << "frame " << k;
    }
    EXPECT_LT((r.target - pose(20) * handle).norm(), 0.002) << "off by " << 1000 * (r.target - pose(20) * handle).transpose() << " mm";
}

// Grasp spots seen in enough frames are averaged; a single frame yields none.
TEST(Cloud, ConsensusKeepsWhatFramesAgreeOn) {
    CloudSettings s;
    s.bar_gap = 0.015;
    s.consensus_min_frames = 2;
    s.match = 0.01;
    s.max_axis = rad(20);
    s.outlier = 0.006;
    s.max_spread = 0.003;
    s.duplicate = 0.005;
    std::vector<std::vector<GraspPose>> frames(3);
    for (int k = 0; k < 3; ++k) {
        for (int i = 0; i < 10; ++i) {
            frames[k].push_back({Eigen::Vector3d(0.01 * i + 0.003 * k, 0.0005 * k, 0.3), Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ()});
        }
    }
    std::string                  summary;
    const std::vector<GraspPose> spots = agreeOnSpots(frames, s, summary);
    ASSERT_FALSE(spots.empty()) << summary;
    for (const GraspPose &g : spots) {
        EXPECT_NEAR(g.point.y(), 0.0005, 0.001);
        EXPECT_NEAR(g.point.z(), 0.3, 1e-9);
    }
    EXPECT_TRUE(agreeOnSpots({frames[0]}, s, summary).empty()) << "one frame cannot agree with itself";
}

// A tilted wall with scattered missing pixels and one pixel floating in front of it: the filter keeps the wall, drops the spike.
TEST(Cloud, FlyingPixelFilterKeepsASurfaceWithHoles) {
    const CameraModel camera{80, 60, 100.0, 100.0, 40.0, 30.0};
    CloudSettings     s;
    s.voxel                   = 0.0025;
    s.crop_radius             = 1.0;
    s.depth_tolerance         = 0.003;
    s.min_agreeing_neighbours = 5;
    s.slope_window_px         = 5;
    s.min_points_per_voxel    = 1;
    s.free_space_tolerance    = 0.005;
    s.max_ray_stretch         = 10.0;

    Frame        f;
    std::mt19937 rng(5);
    for (int v = 0; v < camera.height; ++v) {
        for (int u = 0; u < camera.width; ++u) {
            const double depth = u == 40 && v == 30 ? 0.35 : 0.40 + 0.0005 * u;
            const bool   hole  = std::uniform_real_distribution<double>(0.0, 1.0)(rng) < 0.05;
            f.points.push_back(hole ? Eigen::Vector3f::Constant(NAN)
                                    : Eigen::Vector3f(static_cast<float>((u - camera.cx) * depth / camera.fx),
                                                      static_cast<float>(-(v - camera.cy) * depth / camera.fy), static_cast<float>(-depth)));
        }
    }
    const Eigen::Vector3d spike = f.points[30 * camera.width + 40].cast<double>();
    ASSERT_TRUE(spike.allFinite());
    const auto obstacles = [&](bool filter) {
        s.min_agreeing_neighbours = filter ? 5 : 0;  // 0: every pixel is supported, so nothing is filtered
        return processFrames({f}, camera, s).map;
    };
    const ObstacleMap all = obstacles(false), kept = obstacles(true);
    const uint32_t    spike_cell = static_cast<uint32_t>(all.box.cellOf(spike));
    EXPECT_TRUE(std::binary_search(all.obstacle.begin(), all.obstacle.end(), spike_cell));
    EXPECT_FALSE(std::binary_search(kept.obstacle.begin(), kept.obstacle.end(), spike_cell)) << "the spike passed the filter";
    EXPECT_GT(kept.obstacle.size(), 0.95 * all.obstacle.size()) << "holes cost the wall around them";
}

TEST(Collision, BladeKeepOutCoversEveryVoxelSize) {
    const double                           step = 0.0025, sample_reach = 0.999 * step * std::sqrt(3.0) / 2.0;
    std::mt19937                           rng(9);
    std::uniform_real_distribution<double> half(-0.5, 0.5);
    std::normal_distribution<double>       normal;
    for (const double voxel : {0.002, 0.0025, 0.003, 0.005}) {
        ObstacleMap map;
        map.box.voxel = voxel;
        map.box.nx = map.box.ny = map.box.nz = 11;
        map.obstacle                        = {static_cast<uint32_t>(map.box.index(5, 5, 5))};
        const ObstacleGrid    grid(map, 0.0, bladeRadius(step));
        const Eigen::Vector3d centre = map.box.centre(map.obstacle[0]);
        int                   missed = 0;
        for (int n = 0; n < 20000; ++n) {
            const Eigen::Vector3d surface = centre + voxel * Eigen::Vector3d(half(rng), half(rng), half(rng));
            const Eigen::Vector3d sample  = surface + sample_reach * Eigen::Vector3d(normal(rng), normal(rng), normal(rng)).normalized();
            missed += !grid.bladeBlocked(sample);
        }
        EXPECT_EQ(missed, 0) << "voxel " << 1000 * voxel << " mm";
    }
}

TEST(Collision, HullKeepsLinksTheirRadiusAway) {
    const Hull hull{-0.80, 0.10, -0.40, 0.40, -1.0e-6};
    Body       body;
    body.throat = body.tip = Eigen::Vector3d(-0.25, 0.0, 0.10);
    body.links.fill({Eigen::Vector3d(-0.30, 0.0, 0.10), Eigen::Vector3d(-0.20, 0.0, 0.10)});
    EXPECT_FALSE(hitsHull(body, hull, 0.02));

    body.links[1] = {Eigen::Vector3d(-0.30, 0.0, 0.01), Eigen::Vector3d(-0.20, 0.0, 0.01)};
    EXPECT_TRUE(hitsHull(body, hull, 0.02));
    EXPECT_FALSE(hitsHull(body, hull, 0.005));
    body.links[1] = {Eigen::Vector3d(0.11, 0.0, -0.05), Eigen::Vector3d(0.20, 0.0, -0.05)};
    EXPECT_TRUE(hitsHull(body, hull, 0.02)) << "a link just past the footprint's edge";

    body.links[1] = body.links[0];
    body.blades   = {Eigen::Vector3d(-0.25, 0.0, 0.001)};
    EXPECT_FALSE(hitsHull(body, hull, 0.02)) << "blade samples are surface points";

    const Arm   arm(testArm(expandedUrdf()), 0.005);
    ObstacleMap empty;
    empty.box.voxel = 0.01;
    empty.box.nx = empty.box.ny = empty.box.nz = 1;
    const ObstacleGrid grid(empty, 0.02, 0.0);
    Collision          collision(arm, grid, hull, 0.002);
    EXPECT_EQ(collision.check({{0.0, rad(90.0), 0.0, 0.0}}), Verdict::CLEAR) << "home is folded clear of the padded hull";
}

TEST(Park, ParksOnlyWhereTheCameraSeesTheGrasp) {
    const Arm               arm(testArm(expandedUrdf()), 0.005);
    const CameraModel       camera{800, 600, 479.2662, 479.2662, 397.9827, 322.8405};
    const Eigen::Isometry3d camera_to_vehicle = fromXyzRpy({0.35, -0.05, -0.05}, {0, 90, 0}) * fromXyzRpy({0, 0, 0}, {90, 0, -90});
    const Eigen::Isometry3d vehicle_to_arm    = fromXyzRpy({0.35, 0.0, -0.05}, {180, 0, 0}).inverse();
    const Hull              hull{-0.80, 0.10, -0.40, 0.40, -1.0e-6};
    ParkSettings            p;
    p.box_xy = 1.0, p.box_z = 0.5, p.box_yaw = rad(45.0);
    p.coarse_step = 0.100, p.coarse_yaw = rad(9.0), p.fine_step = 0.025, p.fine_yaw = rad(3.0);
    p.refine_count = 4, p.standoff_min = 0.30, p.standoff_max = 0.40;
    p.screen_count = 3000, p.screen_budget_s = 10.0, p.screen_blade_stride = 20, p.exact_count = 10, p.transit_samples = 16;
    p.reach_cell = 0.002, p.grasp_point_from_mount = 0.050, p.link_radius = 0.020, p.link_step = 0.002, p.blade_step = 0.0025;
    p.swing_band = rad(5.0);
    const ParkSearch park(arm, hull, p, vehicle_to_arm, camera_to_vehicle, camera);

    const GraspPose grasp{Eigen::Vector3d(0.25, 0.0, 0.20), Eigen::Vector3d::UnitY(), Eigen::Vector3d::Zero()};
    const Eigen::Vector3d camera_now = vehicle_to_arm * camera_to_vehicle.translation();
    ASSERT_GT((grasp.point - camera_now).norm(), p.standoff_min);
    ASSERT_LT((grasp.point - camera_now).norm(), p.standoff_max);
    const auto seen_after = [&](const VehiclePose &move) {
        const Eigen::Vector3d q = (vehicle_to_arm * toIsometry(move) * camera_to_vehicle).inverse() * grasp.point;
        const double          u = camera.cx + camera.fx * q.x() / -q.z(), v = camera.cy - camera.fy * q.y() / -q.z();
        return q.z() < 0.0 && u >= 0.0 && u < camera.width && v >= 0.0 && v < camera.height;
    };
    ASSERT_FALSE(seen_after(VehiclePose())) << "the grasp must start out of view";

    ObstacleMap empty;
    empty.box.voxel = 0.01;
    empty.box.nx = empty.box.ny = empty.box.nz = 1;
    const ParkChoice choice = park.choose({grasp}, empty, arm.toModel({{0.0, rad(90.0), 0.0, 0.0}}), [] { return false; });
    EXPECT_NE(choice.decision, ParkDecision::STAY) << choice.summary;
    if (choice.decision == ParkDecision::MOVE) {
        EXPECT_TRUE(seen_after(choice.move)) << choice.summary;
    }
}

namespace {

class QuietIO : public PickIO {
public:
    double now() override { return clock; }
    void   log(Level, const std::string &) override {}
    void   sendTargets(const Joints &) override {}
    bool   closeJaw(std::string &) override { return true; }
    bool   standby() override { return true; }
    bool   home(std::string &) override { return true; }
    void   status(const PickStatus &s) override { state = s.state; }
    void   showMap(const ObstacleMap &, const Eigen::Isometry3d &, bool) override {}
    void   showSpot(const VehiclePose &) override {}
    void   showGrasps(const Scene &, const Eigen::Isometry3d &, int) override {}
    void   showPath(const std::vector<Eigen::Vector3d> &) override {}
    void   showBody(const Body &) override {}
    void   showTrack(const TrackResult &, const std::vector<Eigen::Vector3d> &) override {}
    void   clearTrack() override {}

    double clock = 1.0;
    State  state = State::READY;
};

}  // namespace

TEST(Pick, AShortLookGoesOnWithEnoughFramesAndLooksAgainWithout) {
    const Arm  arm(testArm(expandedUrdf()), 0.005);
    PickConfig c;
    c.loop_hz = 20.0, c.stream_timeout_s = 10.0, c.frame_timeout_s = 10.0, c.frames_needed = 5;
    c.cloud.consensus_min_frames = c.cloud.vote_min_frames = 2;
    c.park.reach_cell = 0.002;
    const auto look = [&](int with_poses, int poseless) {
        QuietIO io;
        Pick    pick(c, arm, io);
        pick.requestStart(false, false);
        pick.cloudSeen();
        pick.tick();
        EXPECT_EQ(io.state, State::COLLECT);
        for (int i = 0; i < with_poses; ++i) {
            pick.addFrame(Frame());
        }
        for (int i = 0; i < poseless; ++i) {
            pick.addPoseless();
        }
        io.clock += c.frame_timeout_s + 1.0;
        pick.tick();
        return io.state;
    };
    EXPECT_EQ(look(5, 0), State::PROCESS);
    EXPECT_EQ(look(2, 7), State::PROCESS);
    EXPECT_EQ(look(1, 7), State::RESURVEY);
    EXPECT_EQ(look(0, 7), State::RESURVEY);
    EXPECT_EQ(look(0, 0), State::FAIL);
}

// The handle holds only between the open blades with its radius clear of both; the miss is scored either way.
TEST(Pick, HandleHoldsOnlyBetweenTheBlades) {
    ArmConfig c = testArm(expandedUrdf());
    c.jaw.open_len              = 0.007;  // robot.yaml's jaw, three bands around the 50 mm grasp point
    c.jaw.blade_rotation_per_m  = 51.0;
    c.jaw.hinge_offset_closing  = 0.0155;
    c.jaw.hinge_offset_approach = 0.0069;
    c.jaw.blades                = {{0.035, 0.040, -0.0139, 0.0052, 0.0049}, {0.040, 0.045, -0.0145, 0.0043, 0.0049},
                                   {0.045, 0.050, -0.0156, 0.0030, 0.0049}};
    const Arm       arm(c, 0.0025);
    const Joints    q{{rad(180.0), rad(90.0), rad(90.0), rad(30.0)}};
    const JawAxes   a     = arm.axes(q);
    const double    along = 0.05, radius = 0.01;
    const auto      at    = [&](double deeper, double aside, const Eigen::Vector3d &bar) {
        return fitHandle(arm, q, along, {arm.points(q).mount + a.approach * (along + deeper) + a.closing * aside, bar, a.approach}, radius);
    };
    const HandleFit centred = at(0.0, 0.0, a.hinge);
    EXPECT_TRUE(centred.holds(rad(20.0)));
    EXPECT_NEAR(centred.miss, 0.0, 1e-9);
    EXPECT_NEAR(centred.depth, along, 1e-9);
    EXPECT_GT(centred.clearance, 0.003);                                                  // about 6 mm each side at 50 mm deep
    EXPECT_TRUE(at(0.0, 0.0, (a.hinge + a.approach).normalized()).holds(rad(20.0)));      // leaning down the approach still squeezes
    const HandleFit aside = at(0.0, 0.008, a.hinge);
    EXPECT_FALSE(aside.holds(rad(20.0)));                                                 // 8 mm aside puts it into a blade
    EXPECT_NEAR(aside.miss, 0.008, 1e-9);
    EXPECT_GT(aside.clearance, -0.01);
    EXPECT_FALSE(std::isfinite(at(0.04, 0.0, a.hinge).clearance));                        // past these blades' tips
    EXPECT_FALSE(at(0.0, 0.0, (a.hinge + a.closing).normalized()).holds(rad(20.0)));      // leaning 45 deg into the closing line
}

// A flat cloud in any order lands back on the pixels it was seen at, holes left as holes.
TEST(Cloud, AFlatCloudGoesBackOnItsPixels) {
    const CameraModel            camera{80, 60, 47.9, 47.9, 39.8, 32.3};
    std::vector<Eigen::Vector3f> organised(static_cast<size_t>(camera.width) * camera.height), flat;
    for (int v = 0; v < camera.height; ++v) {
        for (int u = 0; u < camera.width; ++u) {
            const double    depth = 0.4 + 0.001 * u;
            Eigen::Vector3f p((u - camera.cx) * depth / camera.fx, -(v - camera.cy) * depth / camera.fy, -depth);
            if ((u + v) % 7 == 0) {
                p.setConstant(std::numeric_limits<float>::quiet_NaN());
            } else {
                flat.push_back(p);
            }
            organised[camera.index(u, v)] = p;
        }
    }
    std::reverse(flat.begin(), flat.end());
    flat.push_back(Eigen::Vector3f(0.0f, 0.0f, 0.5f));
    const std::vector<Eigen::Vector3f> grid = onPixelGrid(flat, camera);
    ASSERT_EQ(grid.size(), organised.size());
    for (size_t i = 0; i < grid.size(); ++i) {
        EXPECT_EQ(grid[i].allFinite(), organised[i].allFinite()) << i;
        if (organised[i].allFinite()) {
            EXPECT_LT((grid[i] - organised[i]).norm(), 1e-6f) << i;
        }
    }
}

// Poses take the cloud stamped nearest them out of those held, whichever of the two arrives first; a pair goes out at most every gap.
TEST(Pairs, PosesTakeTheirCloudAndAPairGoesOutPerGap) {
    const int64_t        ms = 1000000;
    FramePairs<int, int> pairs(0.5, 0.010, 2.0);
    int                  cloud = 0, poses = 0;
    int64_t              nearest = 0;

    pairs.addPoses(1000 * ms, 10);
    EXPECT_FALSE(pairs.next(cloud, poses));
    pairs.addCloud(1000 * ms - 50, 1);
    ASSERT_TRUE(pairs.next(cloud, poses)) << "50 ns off its poses, as the vehicle stamps them";
    EXPECT_EQ(cloud, 1);
    EXPECT_EQ(poses, 10);

    pairs.addCloud(1200 * ms, 2);
    pairs.addPoses(1200 * ms, 20);
    EXPECT_FALSE(pairs.next(cloud, poses)) << "a pair sooner than the gap";

    pairs.addCloud(1400 * ms, 3);
    pairs.addCloud(1600 * ms, 4);
    pairs.addCloud(1800 * ms, 5);
    pairs.addPoses(1603 * ms, 40);
    ASSERT_TRUE(pairs.next(cloud, poses));
    EXPECT_EQ(cloud, 4) << "the frame at 1400 got no poses; these are the 1600 frame's, stamped 3 ms off";
    EXPECT_EQ(poses, 40);

    pairs.addPoses(2230 * ms, 50);
    pairs.addCloud(2200 * ms, 6);
    EXPECT_FALSE(pairs.next(cloud, poses)) << "30 ms apart is past the slop";
    EXPECT_FALSE(pairs.missed(nearest)) << "still waiting";
    pairs.addCloud(4300 * ms, 7);
    ASSERT_TRUE(pairs.missed(nearest)) << "poses 50 waited past `wait`";
    EXPECT_EQ(nearest, 30 * ms);
    EXPECT_FALSE(pairs.missed(nearest));
    pairs.addPoses(4300 * ms + 8, 70);
    ASSERT_TRUE(pairs.next(cloud, poses));
    EXPECT_EQ(cloud, 7);
    EXPECT_EQ(poses, 70);
    EXPECT_FALSE(pairs.next(cloud, poses));

    pairs.addCloud(100 * ms, 8);
    pairs.addPoses(100 * ms, 80);
    ASSERT_TRUE(pairs.next(cloud, poses)) << "the clock went back: start over rather than wait for it to catch up";
    EXPECT_EQ(cloud, 8);

    FramePairs<int, int> stuck(0.5, 0.010, 2.0);
    for (int i = 0; i < 100; ++i) {
        stuck.addCloud(5000 * ms, i);
    }
    stuck.addPoses(5000 * ms, 90);
    ASSERT_TRUE(stuck.next(cloud, poses));
    EXPECT_EQ(cloud, 88) << "a stamp that never advances: only the last 12 clouds are still held";
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
