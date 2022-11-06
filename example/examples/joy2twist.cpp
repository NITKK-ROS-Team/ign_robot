#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <geometry_msgs/msg/twist.hpp>

namespace joy2twist
{
class Joy2Twist : public rclcpp::Node
{
public:
  Joy2Twist(const rclcpp::NodeOptions & options)
  : Node("joy2twist", options)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&Joy2Twist::joy_callback, this, std::placeholders::_1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = msg->axes[1];
        twist.angular.z = msg->axes[0];
        twist_pub_->publish(twist);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};

}  // namespace joy2twist

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joy2twist::Joy2Twist)
