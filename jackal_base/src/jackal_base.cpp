#include "jackal_base/jackal_base.hpp"

using jackal_base::JackalBase;

/**
 * @brief Construct a new JackalBase object
 * 
 */
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
}

/**
 * @brief Feedback subscription callback
 * 
 * @param msg 
 */
void JackalBase::feedback_callback(const jackal_msgs::msg::Feedback::SharedPtr msg)
{
  std::lock_guard<std::mutex> guard(feedback_mutex_);
  feedback_ = *msg;
}

/**
 * @brief Publish Drive message
 * 
 * @param left_wheel Left wheel command
 * @param right_wheel Right wheel command
 * @param mode Command mode
 */
void JackalBase::drive_command(float left_wheel, float right_wheel, int8_t mode)
{
  jackal_msgs::msg::Drive drive_msg;
  drive_msg.mode = mode;
  drive_msg.drivers[0] = left_wheel;
  drive_msg.drivers[1] = right_wheel;
  drive_pub_->publish(drive_msg);
}

/**
 * @brief Get latest feedback message
 * 
 * @return jackal_msgs::msg::Feedback message 
 */
jackal_msgs::msg::Feedback JackalBase::get_feedback()
{
  jackal_msgs::msg::Feedback msg;

  {
    std::lock_guard<std::mutex> guard(feedback_mutex_);
    msg = feedback_;
  }

  return msg;
}
