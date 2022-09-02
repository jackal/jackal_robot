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

#ifndef JACKAL_HARDWARE_JACKAL_HARDWARE_INTERFACE_H
#define JACKAL_HARDWARE_JACKAL_HARDWARE_INTERFACE_H

#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "jackal_msgs/msg/drive.hpp"
#include "jackal_msgs/msg/feedback.hpp"

namespace jackal_hardware
{

class JackalHardwareInterface
: public rclcpp::Node
{
  public:
  explicit JackalHardwareInterface();
  void drive_command(float left_wheel, float right_wheel, int8_t mode);
  jackal_msgs::msg::Feedback get_feedback();

  private:
  void feedback_callback(const jackal_msgs::msg::Feedback::SharedPtr msg);

  rclcpp::Publisher<jackal_msgs::msg::Drive>::SharedPtr drive_pub_;
  rclcpp::Subscription<jackal_msgs::msg::Feedback>::SharedPtr feedback_sub_;

  jackal_msgs::msg::Feedback feedback_;
  std::mutex feedback_mutex_;
};

}

#endif  // JACKAL_HARDWARE_JACKAL_HARDWARE_INTERFACE_H