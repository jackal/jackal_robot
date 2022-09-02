/**
 *
 *  \file
 *  \brief      Class representing Jackal base
 *  \author     Roni Kreinin <rkreinin@clearpathrobotics.com>
 *  \author     Tony Baltovski <tbaltovski@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2022, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#include "jackal_hardware/jackal_hardware_interface.hpp"

using jackal_hardware::JackalHardwareInterface;

/**
 * @brief Construct a new JackalHardwareInterface object
 * 
 */
JackalHardwareInterface::JackalHardwareInterface()
: Node("jackal_hardware")
{
  feedback_sub_ = create_subscription<jackal_msgs::msg::Feedback>(
    "feedback",
    rclcpp::SensorDataQoS(),
    std::bind(&JackalHardwareInterface::feedback_callback, this, std::placeholders::_1));

  drive_pub_ = create_publisher<jackal_msgs::msg::Drive>(
    "cmd_drive",
    rclcpp::SensorDataQoS());
}

/**
 * @brief Feedback subscription callback
 * 
 * @param msg 
 */
void JackalHardwareInterface::feedback_callback(const jackal_msgs::msg::Feedback::SharedPtr msg)
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
void JackalHardwareInterface::drive_command(float left_wheel, float right_wheel, int8_t mode)
{
  jackal_msgs::msg::Drive drive_msg;
  drive_msg.mode = mode;
  drive_msg.drivers[jackal_msgs::msg::Drive::LEFT] = left_wheel;
  drive_msg.drivers[jackal_msgs::msg::Drive::RIGHT] = right_wheel;
  drive_pub_->publish(drive_msg);
}

/**
 * @brief Get latest feedback message
 * 
 * @return jackal_msgs::msg::Feedback message 
 */
jackal_msgs::msg::Feedback JackalHardwareInterface::get_feedback()
{
  jackal_msgs::msg::Feedback msg;

  {
    std::lock_guard<std::mutex> guard(feedback_mutex_);
    msg = feedback_;
  }

  return msg;
}
