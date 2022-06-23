#include "jackal_base/jackal_base.hpp"

using jackal_base::JackalBase;

JackalBase::JackalBase()
: Node("jackal_base")
{
  feedback_sub_ = create_subscription<jackal_msgs::msg::Feedback>(
    "/feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&JackalBase::feedback_callback, this, std::placeholders::_1));

  drive_pub_ = create_publisher<jackal_msgs::msg::Drive>(
    "/cmd_drive",
    rclcpp::SensorDataQoS());

  feedback_ = jackal_msgs::msg::Feedback::SharedPtr();
}

void JackalBase::feedback_callback(const jackal_msgs::msg::Feedback::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "Feedback received\n");
  feedback_ = msg;
}

void JackalBase::drive_command(float left_wheel, float right_wheel, int8_t mode)
{
  jackal_msgs::msg::Drive drive_msg;
  drive_msg.mode = mode;
  drive_msg.drivers[0] = left_wheel;
  drive_msg.drivers[1] = right_wheel;
  drive_pub_->publish(drive_msg);
}

jackal_msgs::msg::Feedback::SharedPtr JackalBase::get_feedback()
{
  return feedback_;
}