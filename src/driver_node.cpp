// Copyright by BeeX [2026]
// The driver node: joint_states out, joint targets in, and the jaw, home and standby as services. Driver itself knows no ROS.
#include "rosparams.h"

#include <driver.h>

#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>

#include <algorithm>

using namespace final_v1;

namespace {

class DriverNode {
public:
    explicit DriverNode(Driver &driver) : driver_(driver) {
        ros::NodeHandle nh, pnh("~");
        pub_states_  = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        sub_targets_ = pnh.subscribe("targets", 1, &DriverNode::onTargets, this);
        services_    = {pnh.advertiseService("home", &DriverNode::onHome, this), pnh.advertiseService("open", &DriverNode::onOpenJaw, this),
                        pnh.advertiseService("close", &DriverNode::onCloseJaw, this),
                        pnh.advertiseService("standby", &DriverNode::onStandby, this)};
    }

    void poll() {
        AxisReadings r;
        if (!driver_.poll(ros::Time::now().toSec(), r)) {
            ROS_WARN_THROTTLE(5.0, "[driver] not every joint has answered yet");
            return;
        }
        sensor_msgs::JointState msg;
        msg.header.stamp = ros::Time(r.stamp);
        for (int a = 0; a < AXES; ++a) {
            msg.name.push_back(driver_.axes()[a].name);
            msg.position.push_back(r.position[a]);
        }
        pub_states_.publish(msg);
    }

private:
    void onTargets(const sensor_msgs::JointState::ConstPtr &msg) {
        std::string why;
        if (!driver_.moveTo(msg->name, msg->position, why)) {
            ROS_WARN("[driver] %s", why.c_str());
        }
    }

    bool onHome(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        res.success = driver_.home(res.message);
        return true;
    }

    bool onOpenJaw(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        res.success = driver_.openJaw(res.message);
        return true;
    }

    bool onCloseJaw(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        res.success = driver_.closeJaw(res.message);
        return true;
    }

    bool onStandby(std_srvs::Trigger::Request &, std_srvs::Trigger::Response &res) {
        res.success = driver_.standby(res.message);
        rosLog("driver", res.success ? Level::INFO : Level::ERROR, res.message);
        return true;
    }

    Driver                         &driver_;
    ros::Publisher                  pub_states_;
    ros::Subscriber                 sub_targets_;
    std::vector<ros::ServiceServer> services_;
};

int run(int argc, char **argv) {
    ros::init(argc, argv, "driver");
    Params robot("/robot", {"driver"});
    Params own("/driver");

    std::array<Axis, AXES> axes;
    const std::string      keys[AXES] = {"base", "shoulder", "elbow", "wrist", "jaw"};
    const double joint_speed          = rad(robot.number("driver/speed/joint"));
    for (int a = 0; a < AXES; ++a) {
        Axis &x  = axes[a];
        x.name   = robot.text("arm/joints/" + keys[a]);
        const int device = robot.whole("driver/ids/" + keys[a]);
        robot.require(device > 0 && device < 256, "driver/ids/" + keys[a], "between 1 and 255");
        x.device = static_cast<uint8_t>(device);
        if (a == JAW) {
            const std::vector<double> limits = robot.numbers("jaws/limits", 2);
            x.min           = limits[0];
            x.max           = limits[1];
            x.home          = robot.number("jaws/open");
            x.speed         = robot.number("driver/speed/jaw");
            x.wire_per_unit = 1000.0;  // the jaw speaks millimetres on the wire
        } else {
            const std::vector<double> limits = robot.numbers("arm/limits/" + keys[a], 2);
            x.min   = rad(limits[0]);
            x.max   = rad(limits[1]);
            x.home  = rad(robot.number("arm/home/" + keys[a]));
            x.speed = joint_speed;
        }
        robot.require(x.min < x.max && x.home >= x.min && x.home <= x.max, keys[a], "min below max with home inside");
    }
    SerialSettings serial;
    serial.port             = robot.text("driver/port");
    serial.baud             = robot.whole("driver/baud");
    serial.reply_timeout_s  = robot.number("driver/timeout");
    serial.connect_attempts = robot.whole("driver/attempts");
    serial.connect_retry_s  = robot.number("driver/retry");
    serial.wake_speed       = rad(robot.number("driver/wake"));
    const double poll_hz    = robot.number("driver/rate");
    const double max_jump   = rad(robot.number("driver/jump"));
    robot.require(poll_hz > 0.0 && serial.reply_timeout_s > 0.0 && serial.connect_attempts > 0 && max_jump > 0.0, "driver",
                  "positive rate, timeout, attempts and jump");
    const std::string problems = robot.problems() + own.problems();
    if (!problems.empty()) {
        ROS_ERROR("[driver] configuration is not usable:\n%s", problems.c_str());
        return 1;
    }

    const Log                        log = [](Level level, const std::string &text) { rosLog("driver", level, text); };
    std::string                      why;
    const std::shared_ptr<Actuators> actuators = connectArm(serial, axes, log, why);
    if (!actuators) {
        ROS_ERROR("[driver] %s", why.c_str());
        return 1;
    }

    Driver      driver(axes, actuators, poll_hz, max_jump, log);
    std::string released;
    driver.standby(released);
    DriverNode node(driver);
    ROS_INFO("[driver] talking to %s at %.1f Hz", serial.port.c_str(), poll_hz);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    for (ros::Rate rate(poll_hz); ros::ok(); rate.sleep()) {
        node.poll();
    }
    driver.standby(released);
    return 0;
}

}  // namespace

int main(int argc, char **argv) { return run(argc, argv); }
