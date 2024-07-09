/**
 *
 *  \file
 *  \brief      Class representing Jackal hardware
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

#include "jackal_hardware/jackal_hardware.hpp"
#include "clearpath_platform_msgs/msg/feedback.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace jackal_hardware
{

static const std::string HW_NAME = "JackalHardware";
static const std::string LEFT_CMD_JOINT_NAME = "front_left_wheel_joint";
static const std::string RIGHT_CMD_JOINT_NAME = "front_right_wheel_joint";
static const std::string LEFT_ALT_JOINT_NAME = "rear_left_wheel_joint";
static const std::string RIGHT_ALT_JOINT_NAME = "rear_right_wheel_joint";

/**
 * @brief Write commanded velocities to the MCU
 * 
 */
void JackalHardware::writeCommandsToHardware()
{
  double diff_speed_left = hw_commands_[wheel_joints_[LEFT_CMD_JOINT_NAME]];
  double diff_speed_right = hw_commands_[wheel_joints_[RIGHT_CMD_JOINT_NAME]];

  if (std::abs(diff_speed_left) < 0.01 && std::abs(diff_speed_right) < 0.01) {
    diff_speed_left = diff_speed_right = 0.0;
  }

  node_->drive_command(
    static_cast<float>(diff_speed_left),
    static_cast<float>(diff_speed_right),
    clearpath_platform_msgs::msg::Drive::MODE_VELOCITY);
}

/**
 * @brief Pull latest speed and travel measurements from MCU, 
 * and store in joint structure for ros_control
 * 
 */
void JackalHardware::updateJointsFromHardware()
{
  rclcpp::spin_some(node_);
  clearpath_platform_msgs::msg::Feedback msg = node_->get_feedback();
  RCLCPP_DEBUG(
    rclcpp::get_logger(HW_NAME),
    "Received linear distance information (L: %f, R: %f)",
    msg.drivers[0].measured_travel, msg.drivers[1].measured_travel);

  auto side = clearpath_platform_msgs::msg::Drive::LEFT;
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (i == wheel_joints_[RIGHT_ALT_JOINT_NAME] || i == wheel_joints_[RIGHT_CMD_JOINT_NAME]){
      side = clearpath_platform_msgs::msg::Drive::RIGHT;
    }
    else {
      side = clearpath_platform_msgs::msg::Drive::LEFT;
    }

    double delta = msg.drivers[side].measured_travel -
      hw_states_position_[i] - hw_states_position_offset_[i];

    // detect suspiciously large readings, possibly from encoder rollover
    if (std::abs(delta) < 1.0f) {
      hw_states_position_[i] += delta;
    } else {
      // suspicious! drop this measurement and update the offset for subsequent readings
      hw_states_position_offset_[i] += delta;
      RCLCPP_WARN(
        rclcpp::get_logger(HW_NAME), "Dropping overflow measurement from encoder");
    }

    // Velocities
    hw_states_velocity_[i] = msg.drivers[side].measured_velocity;
  }
}

hardware_interface::CallbackReturn JackalHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Name: %s", info_.name.c_str());

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Number of Joints %lu", info_.joints.size());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_position_offset_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  node_ = std::make_shared<JackalHardwareInterface>();

  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // JackalHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %ld command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %ld state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as first state interface. ",
          joint.name.c_str(), joint.state_interfaces[0].name.c_str());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as second state interface. .", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JackalHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> JackalHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Map wheel joint name to index
    wheel_joints_[info_.joints[i].name] = i;
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn JackalHardware::on_activate(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++) {
    if (std::isnan(hw_states_position_[i])) {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JackalHardware::on_deactivate(const rclcpp_lifecycle::State & )
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Stopping ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System successfully stopped!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type JackalHardware::read(const rclcpp::Time & , const rclcpp::Duration & )
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");

  updateJointsFromHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JackalHardware::write(const rclcpp::Time & , const rclcpp::Duration & )
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");

  writeCommandsToHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace jackal_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  jackal_hardware::JackalHardware, hardware_interface::SystemInterface)
