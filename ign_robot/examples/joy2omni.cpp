#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>
#include <cmath>

namespace joy2omni
{
class Joy2Omni : public rclcpp::Node
{
public:
  Joy2Omni(const rclcpp::NodeOptions & options)
  : Node("joy2omni", options)
  {
    this->declare_parameter("spd_boost", 10.0);
    this->spd_boost_ = this->get_parameter("spd_boost").as_double();

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Omni::joy_callback, this, std::placeholders::_1));

    // /velocity_controller/commands
    omni0_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("command0", 10);
    omni1_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("command1", 10);
    omni2_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("command2", 10);
    omni3_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("command3", 10);
  }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        auto twist_msg = std_msgs::msg::Float64MultiArray();
        // 1 --> y, 0 --> x (-1), 3 --> yaw
        this->four_omni(-msg->axes[0], msg->axes[1], msg->axes[3]);
    }

    void four_omni(const double x, const double y, const double yaw)
    {
        auto omni0_msg = std_msgs::msg::Float64MultiArray();
        auto omni1_msg = std_msgs::msg::Float64MultiArray();
        auto omni2_msg = std_msgs::msg::Float64MultiArray();
        auto omni3_msg = std_msgs::msg::Float64MultiArray();

        double spd0 = (x + y) / std::sqrt(2.0) + yaw;
        double spd1 = -(x - y) / std::sqrt(2.0) + yaw;
        double spd2 = (x - y) / std::sqrt(2.0) + yaw;
        double spd3 = -(x + y) / std::sqrt(2.0) + yaw;

        omni0_msg.data.push_back(spd0 * this->spd_boost_);
        omni1_msg.data.push_back(spd1 * this->spd_boost_);
        omni2_msg.data.push_back(spd2 * this->spd_boost_);
        omni3_msg.data.push_back(spd3 * this->spd_boost_);

        omni0_pub_->publish(omni0_msg);
        omni1_pub_->publish(omni1_msg);
        omni2_pub_->publish(omni2_msg);
        omni3_pub_->publish(omni3_msg);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni0_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr omni3_pub_;

    double spd_boost_ = 5.0;
};

}  // namespace joy2twist

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(joy2omni::Joy2Omni)
