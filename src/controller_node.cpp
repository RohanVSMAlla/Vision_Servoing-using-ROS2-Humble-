#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/twist.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode() : Node("controller_node")
  {
    kp_ = declare_parameter<double>("kp", 2.0);
    base_speed_ = declare_parameter<double>("base_speed", 0.8);

    offset_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/target_offset", 10,
      std::bind(&ControllerNode::offset_callback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  }

private:
  void offset_callback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    double offset = static_cast<double>(msg->data);  // -1 to +1 approx

    geometry_msgs::msg::Twist cmd;

    // Angular: turn to reduce offset (negative offset => turn left etc.)
    cmd.angular.z = -kp_ * offset;

    // Forward speed reduced when error is large
    double mag = std::abs(offset);
    if (mag < 1.0) {
      cmd.linear.x = base_speed_ * (1.0 - mag);
    } else {
      cmd.linear.x = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr offset_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  double kp_;
  double base_speed_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
