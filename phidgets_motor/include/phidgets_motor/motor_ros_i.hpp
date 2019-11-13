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

#ifndef PHIDGETS_MOTOR_MOTOR_ROS_I_H
#define PHIDGETS_MOTOR_MOTOR_ROS_I_H

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

#include "phidgets_api/motor.hpp"

namespace phidgets
{
class RosMotor : public Motor
{
public:
  explicit RosMotor(rclcpp::Node* node, const ChannelAddress& channel_address);

  void publishDutyCycle();
  void publishBackEMF();

private:
  std::mutex ros_motor_mutex_;
  enum
  {
    INTERFACE_NAME_LENGTH_MAX = 200
  };
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr duty_cycle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr back_emf_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr duty_cycle_subscription_;

  void dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr msg);
};

class MotorRosI final : public rclcpp::Node
{
public:
  explicit MotorRosI(const rclcpp::NodeOptions& options);

private:
  std::unordered_map<std::string, std::unique_ptr<RosMotor>> ros_motors_;

  rclcpp::TimerBase::SharedPtr timer_;
  double publish_rate_;

  void timerCallback();
};

}  // namespace phidgets

#endif  // PHIDGETS_MOTOR_MOTOR_ROS_I_H
