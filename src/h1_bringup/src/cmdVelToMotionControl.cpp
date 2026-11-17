// Pure translator: /cmd_vel -> zeno_h1_motion_control AUTO input.
//
// Subscribes  /cmd_vel  geometry_msgs/TwistStamped  (vehicle frame: x fwd, y left, yaw rate)
// Publishes   /zeno/h1/auto/wholebody/cmd  std_msgs/Float64MultiArray, 24 fields:
//               [0] mode (1 = AUTO), [1..20] torso + arms + grippers (all 0), [21..23] vx vy wz
//
// One output message per /cmd_vel message.

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class CmdVelToMotionControl : public rclcpp::Node
{
public:
  CmdVelToMotionControl() : Node("cmdVelToMotionControl")
  {
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/zeno/h1/auto/wholebody/cmd", 1);
    sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/cmd_vel", 5, [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr cmd) {
          std_msgs::msg::Float64MultiArray out;
          out.data.resize(24, 0.0);
          out.data[0] = 1.0;  // AUTO
          out.data[21] = cmd->twist.linear.x;
          out.data[22] = cmd->twist.linear.y;
          out.data[23] = cmd->twist.angular.z;
          pub_->publish(out);
        });
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToMotionControl>());
  rclcpp::shutdown();
  return 0;
}