/*
 * Copyright (c) 2019, Howard Hughes Medical Institute
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

#ifndef PHIDGETS_STEPPER_STEPPER_ROS_I_H
#define PHIDGETS_STEPPER_STEPPER_ROS_I_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <chrono>
#include <functional>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <phidgets_msgs/srv/get_stepper_setting_ranges.hpp>
#include <phidgets_msgs/srv/get_stepper_settings.hpp>
#include <phidgets_msgs/srv/set_current_limit.hpp>
#include <phidgets_msgs/srv/set_enabled.hpp>
#include <phidgets_msgs/srv/set_float64.hpp>
#include <phidgets_msgs/srv/set_position.hpp>

#include "phidgets_api/stepper.hpp"

namespace phidgets
{
class RosStepper : public Stepper
{
public:
  explicit RosStepper(rclcpp::Node* node, const ChannelAddress& channel_address);

  // void positionChangeCallback(int channel, double position);

  // void velocityChangeCallback(int channel, double velocity);

  // void stoppedCallback(int channel);

private:
  std::mutex mutex_;
  enum
  {
    INTERFACE_NAME_LENGTH_MAX = 200
  };
  rclcpp::Service<phidgets_msgs::srv::SetEnabled>::SharedPtr set_enabled_service_;
  rclcpp::Service<phidgets_msgs::srv::SetFloat64>::SharedPtr set_target_position_service_;
  rclcpp::Service<phidgets_msgs::srv::SetFloat64>::SharedPtr set_velocity_limit_service_;
  rclcpp::Service<phidgets_msgs::srv::SetFloat64>::SharedPtr set_acceleration_service_;
  rclcpp::Service<phidgets_msgs::srv::SetCurrentLimit>::SharedPtr set_current_limit_service_;
  rclcpp::Service<phidgets_msgs::srv::SetCurrentLimit>::SharedPtr set_holding_current_limit_service_;
  rclcpp::Service<phidgets_msgs::srv::SetPosition>::SharedPtr set_position_service_;
  rclcpp::Service<phidgets_msgs::srv::GetStepperSettings>::SharedPtr get_settings_service_;
  rclcpp::Service<phidgets_msgs::srv::GetStepperSettingRanges>::SharedPtr get_setting_ranges_service_;

  void setEnabledCallback(const std::shared_ptr<phidgets_msgs::srv::SetEnabled::Request> req,
                          std::shared_ptr<phidgets_msgs::srv::SetEnabled::Response> res);
  void setTargetPositionCallback(const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
                                 std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res);
  void setVelocityLimitCallback(const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
                                std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res);
  void setAccelerationCallback(const std::shared_ptr<phidgets_msgs::srv::SetFloat64::Request> req,
                               std::shared_ptr<phidgets_msgs::srv::SetFloat64::Response> res);
  void setCurrentLimitCallback(const std::shared_ptr<phidgets_msgs::srv::SetCurrentLimit::Request> req,
                               std::shared_ptr<phidgets_msgs::srv::SetCurrentLimit::Response> res);
  void setHoldingCurrentLimitCallback(const std::shared_ptr<phidgets_msgs::srv::SetCurrentLimit::Request> req,
                                      std::shared_ptr<phidgets_msgs::srv::SetCurrentLimit::Response> res);
  void setPositionCallback(const std::shared_ptr<phidgets_msgs::srv::SetPosition::Request> req,
                           std::shared_ptr<phidgets_msgs::srv::SetPosition::Response> res);
  void getSettingsCallback(const std::shared_ptr<phidgets_msgs::srv::GetStepperSettings::Request> req,
                           std::shared_ptr<phidgets_msgs::srv::GetStepperSettings::Response> res);
  void getSettingRangesCallback(const std::shared_ptr<phidgets_msgs::srv::GetStepperSettingRanges::Request> req,
                                std::shared_ptr<phidgets_msgs::srv::GetStepperSettingRanges::Response> res);
};

class StepperRosI final : public rclcpp::Node
{
public:
  explicit StepperRosI(const rclcpp::NodeOptions& options);

private:
  std::unordered_map<std::string, std::unique_ptr<RosStepper>> ros_steppers_;
  enum
  {
    PARAMETER_NAME_LENGTH_MAX = 200
  };
  std::string frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  double publish_rate_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  void publishTimerCallback();

  void publishJointStates();

  void watchdogTimerCallback();

  void kickWatchdogTimers();

};

}  // namespace phidgets

#endif  // PHIDGETS_STEPPER_STEPPER_ROS_I_H
