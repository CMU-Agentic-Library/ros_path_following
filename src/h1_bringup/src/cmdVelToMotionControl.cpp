// Bridge between pathFollower and zeno_h1_motion_control (AUTO input).
//
// Subscribes  /cmd_vel           geometry_msgs/TwistStamped  (vehicle frame: x fwd, y left, yaw rate)
//             /state_estimation  nav_msgs/Odometry           (only used to detect stale odometry)
//             /chassis_estop     std_msgs/Bool               (true = stop and release AUTO, latched until false)
//             /zeno/h1/wheelarm/{torso,left_arm,right_arm}/joint_state, /zeno/h1/{left,right}_gripper/joint_state
//             /zeno/h1/motion/decision_state                 (which mode motion_control is in)
// Publishes   /zeno/h1/auto/wholebody/cmd  std_msgs/Float64MultiArray, 24 fields:
//               [0] mode (1 = AUTO, 0 = release), [1..18] torso + arms, [19..20] grippers, [21..23] vx vy wz
//
// AUTO messages also set the torso/arm/gripper targets, so this node fills them with the measured pose
// and freezes that pose while AUTO is in control (the arms hold still while the base drives).
//
// motion_control keeps the last AUTO chassis velocity if AUTO messages simply stop, so whenever there is
// nothing valid to send (cmd_vel or odometry stale, estop, shutdown) this node sends mode 0, which makes
// motion_control zero the chassis and go IDLE.
//
// AUTO is the lowest-priority mode in motion_control: joystick / VR / wholebody teleop override it.

#include <algorithm>
#include <array>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "zeno_h1_msgs/msg/motion_decision_state.hpp"

namespace {
constexpr int kNumJoints = 18;  // torso 4 + left arm 7 + right arm 7
constexpr int kNumFields = 24;
constexpr uint8_t kModeAuto = 1;
}  // namespace

class CmdVelToMotionControl : public rclcpp::Node
{
public:
  CmdVelToMotionControl() : Node("cmdVelToMotionControl")
  {
    cmdTimeout_ = declare_parameter<double>("cmdTimeout", 0.25);
    odomTimeout_ = declare_parameter<double>("odomTimeout", 0.3);
    jointTimeout_ = declare_parameter<double>("jointTimeout", 0.5);
    checkOdom_ = declare_parameter<bool>("checkOdom", true);
    maxLinear_ = declare_parameter<double>("maxLinear", 0.2);
    maxAngular_ = declare_parameter<double>("maxAngular", 1.0);
    releaseDuration_ = declare_parameter<double>("releaseDuration", 0.5);
    const double rate = declare_parameter<double>("publishRate", 50.0);
    const auto autoTopic = declare_parameter<std::string>("autoTopic", "/zeno/h1/auto/wholebody/cmd");

    subCmd_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 5, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          vx_ = msg->twist.linear.x;
          vy_ = msg->twist.linear.y;
          wz_ = msg->twist.angular.z;
          lastCmd_ = now();
          gotCmd_ = true;
        });

    subOdom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation", 5, [this](nav_msgs::msg::Odometry::ConstSharedPtr) {
          lastOdom_ = now();
          gotOdom_ = true;
        });

    subEstop_ = create_subscription<std_msgs::msg::Bool>(
        "/chassis_estop", 5, [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (msg->data != estop_) {
            RCLCPP_WARN(get_logger(), "Emergency stop %s.", msg->data ? "ON" : "released");
          }
          estop_ = msg->data;
        });

    subDecision_ = create_subscription<zeno_h1_msgs::msg::MotionDecisionState>(
        "/zeno/h1/motion/decision_state", 5, [this](zeno_h1_msgs::msg::MotionDecisionState::ConstSharedPtr msg) {
          if (msg->control_mode != decisionMode_) {
            RCLCPP_INFO(get_logger(), "motion_control mode %d -> %d.", decisionMode_, msg->control_mode);
          }
          decisionMode_ = msg->control_mode;
        });

    // Joint states in the same units motion_control uses for its targets.
    subJoint(0, "/zeno/h1/wheelarm/torso/joint_state", 0, 4);
    subJoint(1, "/zeno/h1/wheelarm/left_arm/joint_state", 4, 7);
    subJoint(2, "/zeno/h1/wheelarm/right_arm/joint_state", 11, 7);
    subGripper(0, "/zeno/h1/left_gripper/joint_state");
    subGripper(1, "/zeno/h1/right_gripper/joint_state");

    pubAuto_ = create_publisher<std_msgs::msg::Float64MultiArray>(autoTopic, 1);

    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() { tick(); });

    RCLCPP_INFO(get_logger(), "Publishing on %s, maxLinear %.2f m/s, maxAngular %.2f rad/s, cmdTimeout %.2f s.",
                autoTopic.c_str(), maxLinear_, maxAngular_, cmdTimeout_);
  }

  // Send mode 0: motion_control zeros the chassis and leaves AUTO (only acts if AUTO is active).
  void publishRelease()
  {
    pubAuto_->publish(makeMsg(0, 0, 0, 0));
  }

private:
  void subJoint(int group, const std::string& topic, int offset, int count)
  {
    jointSubs_.push_back(create_subscription<sensor_msgs::msg::JointState>(
        topic, 5, [this, group, offset, count](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (static_cast<int>(msg->position.size()) < count) return;
          for (int i = 0; i < count; i++) measured_[offset + i] = msg->position[i];
          lastJoint_[group] = now();
          gotJoint_[group] = true;
        }));
  }

  void subGripper(int side, const std::string& topic)
  {
    jointSubs_.push_back(create_subscription<sensor_msgs::msg::JointState>(
        topic, 5, [this, side](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
          if (msg->position.empty()) return;
          measuredGripper_[side] = msg->position[0];
        }));
  }

  bool jointsFresh(const rclcpp::Time& t) const
  {
    for (int g = 0; g < 3; g++) {
      if (!gotJoint_[g] || (t - lastJoint_[g]).seconds() > jointTimeout_) return false;
    }
    return true;
  }

  std_msgs::msg::Float64MultiArray makeMsg(double mode, double vx, double vy, double wz) const
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(kNumFields, 0.0);
    msg.data[0] = mode;
    for (int i = 0; i < kNumJoints; i++) msg.data[1 + i] = hold_[i];
    msg.data[19] = holdGripper_[0];
    msg.data[20] = holdGripper_[1];
    msg.data[21] = vx;
    msg.data[22] = vy;
    msg.data[23] = wz;
    return msg;
  }

  void tick()
  {
    const rclcpp::Time t = now();
    const bool cmdFresh = gotCmd_ && (t - lastCmd_).seconds() < cmdTimeout_;
    const bool odomFresh = !checkOdom_ || (gotOdom_ && (t - lastOdom_).seconds() < odomTimeout_);
    const bool jointOk = jointsFresh(t);
    const bool active = cmdFresh && odomFresh && jointOk && !estop_;

    // Track the measured pose until AUTO is actually in control, then hold it.
    if (!active || decisionMode_ != kModeAuto) {
      hold_ = measured_;
      holdGripper_ = measuredGripper_;
    }

    if (active) {
      double vx = vx_, vy = vy_;
      // Scale the planar velocity as a vector so the driving direction is preserved.
      const double v = std::hypot(vx, vy);
      if (v > maxLinear_) {
        vx *= maxLinear_ / v;
        vy *= maxLinear_ / v;
      }
      const double wz = std::clamp(wz_, -maxAngular_, maxAngular_);
      if (std::isfinite(vx) && std::isfinite(vy) && std::isfinite(wz)) {
        pubAuto_->publish(makeMsg(1, vx, vy, wz));
        lastActive_ = t;
        wasActive_ = true;
        return;
      }
    }

    if (cmdFresh && !estop_) {
      if (!odomFresh) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Odometry stale, releasing AUTO.");
      if (!jointOk) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                         "No joint states from motion_control, not sending AUTO.");
    }

    // Keep sending mode 0 for a short while after we stop, then go quiet so other inputs are not disturbed.
    if (wasActive_ && (t - lastActive_).seconds() < releaseDuration_) {
      publishRelease();
    }
  }

  double cmdTimeout_, odomTimeout_, jointTimeout_, maxLinear_, maxAngular_, releaseDuration_;
  bool checkOdom_;

  double vx_ = 0, vy_ = 0, wz_ = 0;
  rclcpp::Time lastCmd_, lastOdom_, lastActive_;
  std::array<rclcpp::Time, 3> lastJoint_;
  std::array<bool, 3> gotJoint_{false, false, false};
  bool gotCmd_ = false, gotOdom_ = false, estop_ = false, wasActive_ = false;
  int decisionMode_ = -1;

  std::array<double, kNumJoints> measured_{}, hold_{};
  std::array<double, 2> measuredGripper_{}, holdGripper_{};

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subCmd_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subEstop_;
  rclcpp::Subscription<zeno_h1_msgs::msg::MotionDecisionState>::SharedPtr subDecision_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> jointSubs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pubAuto_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelToMotionControl>();
  // Release AUTO on Ctrl-C so motion_control zeros the chassis (best effort: does not help on a hard kill).
  rclcpp::contexts::get_global_default_context()->add_pre_shutdown_callback([node]() {
    for (int i = 0; i < 3; i++) node->publishRelease();
  });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
