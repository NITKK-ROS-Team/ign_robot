#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace joy2direct
{
class Joy2Direct : public rclcpp::Node
{
public:
  Joy2Direct(const rclcpp::NodeOptions & options)
  : Node("joy2direct", options)
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Direct::joy_callback, this, std::placeholders::_1));

    // /velocity_controller/commands
    twist_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "commands", 10);
  }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist_msg = std_msgs::msg::Float64MultiArray();
        // button[2] -> up
        // button[0] -> down
        if (msg->buttons[2] == 1) {
            twist_msg.data.push_back(10.0);
        } else if (msg->buttons[0] == 1) {
            twist_msg.data.push_back(-10.0);
        } else {
            twist_msg.data.push_back(0.0);
        }
        twist_pub_->publish(twist_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr twist_pub_;
};

}  // namespace joy2twist

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joy2direct::Joy2Direct)
