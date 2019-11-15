/*
 * Copyright (c) 2019, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef PHIDGETS_DC_MOTOR_DC_MOTOR_ROS_I_H
#define PHIDGETS_DC_MOTOR_DC_MOTOR_ROS_I_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <chrono>
#include <functional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <std_msgs/msg/float64.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include "phidgets_api/dc_motor.hpp"

namespace phidgets
{
class DCMotorRosI;

class RosDCMotor : public DCMotor
{
public:
  // explicit RosDCMotor(rclcpp::Node* node, const ChannelAddress& channel_address);
  explicit RosDCMotor(DCMotorRosI* node, const ChannelAddress& channel_address);

  void velocityUpdateHandler();
  void backEMFChangeHandler();

private:
  DCMotorRosI* node_;
  std::mutex ros_dc_motor_mutex_;
  enum
  {
    INTERFACE_NAME_LENGTH_MAX = 200
  };
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_emf_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscription_;

  void velocityCallback(const std_msgs::msg::Float64::SharedPtr msg);
};

class DCMotorRosI final : public rclcpp::Node
{
public:
  explicit DCMotorRosI(const rclcpp::NodeOptions& options);

  void publishJointStatesHandler();

private:
  std::unordered_map<std::string, std::unique_ptr<RosDCMotor>> ros_dc_motors_;
  double publish_rate_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  std::string frame_id_;
  void publishJointStates();

  rclcpp::TimerBase::SharedPtr timer_;
  void timerCallback();
};

}  // namespace phidgets

#endif  // PHIDGETS_DC_MOTOR_DC_MOTOR_ROS_I_H
