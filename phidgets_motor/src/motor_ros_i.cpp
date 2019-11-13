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

#include "phidgets_motor/motor_ros_i.hpp"

namespace phidgets
{
RosMotor::RosMotor(rclcpp::Node* node, const ChannelAddress& channel_address)
  : Motor(channel_address, std::bind(&RosMotor::publishDutyCycle, this), std::bind(&RosMotor::publishBackEMF, this))

{
  std::lock_guard<std::mutex> lock(ros_motor_mutex_);

  char interface_name[INTERFACE_NAME_LENGTH_MAX];

  snprintf(interface_name, INTERFACE_NAME_LENGTH_MAX, "motor_duty_cycle");
  duty_cycle_publisher_ = node->create_publisher<std_msgs::msg::Float64>(interface_name, 1);

  snprintf(interface_name, INTERFACE_NAME_LENGTH_MAX, "motor_back_emf");
  back_emf_publisher_ = node->create_publisher<std_msgs::msg::Float64>(interface_name, 1);

  snprintf(interface_name, INTERFACE_NAME_LENGTH_MAX, "set_motor_duty_cycle");
  duty_cycle_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
      interface_name, rclcpp::QoS(1), std::bind(&RosMotor::dutyCycleCallback, this, std::placeholders::_1));
}

void RosMotor::publishDutyCycle()
{
  std::lock_guard<std::mutex> lock(ros_motor_mutex_);
  auto msg = std_msgs::msg::Float64();
  msg.data = getDutyCycle();
  duty_cycle_publisher_->publish(msg);
}

void RosMotor::publishBackEMF()
{
  std::lock_guard<std::mutex> lock(ros_motor_mutex_);
  if (backEMFSensingSupported())
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = getBackEMF();
    back_emf_publisher_->publish(msg);
  }
}

void RosMotor::dutyCycleCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  try
  {
    setDutyCycle(msg->data);
  }
  catch (const phidgets::Phidget22Error& err)
  {
    // If the data was wrong, the lower layers will throw an exception; just
    // catch and ignore here so we don't crash the node.
  }
}

MotorRosI::MotorRosI(const rclcpp::NodeOptions& options) : rclcpp::Node("phidgets_motor_node", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(get_logger(), "Starting Phidgets Motor");

  ChannelAddress channel_address;
  channel_address.serial_number = this->declare_parameter("serial", -1);  // default open any device

  channel_address.hub_port = this->declare_parameter("hub_port", 0);  // only used if the device is on a VINT hub_port

  int data_interval_ms = this->declare_parameter("data_interval_ms", 250);

  double braking_strength = this->declare_parameter("braking_strength", 0.0);

  publish_rate_ = this->declare_parameter("publish_rate", 0.0);
  if (publish_rate_ > 1000.0)
  {
    throw std::runtime_error("Publish rate must be <= 1000");
  }

  RCLCPP_INFO(get_logger(), "Connecting to Phidgets Motor serial %d, hub port %d ...", channel_address.serial_number,
              channel_address.hub_port);

  int n_motors;
  try
  {
    n_motors = 1;

    for (int i = 0; i < n_motors; i++)
    {
      std::string key = std::to_string(i);
      ros_motors_[key] = std::make_unique<RosMotor>(this, channel_address);
      RCLCPP_INFO(get_logger(), "Connected to serial %d, %d motor",
                  ros_motors_.at(key)->getChannelAddress().serial_number, n_motors);

      ros_motors_.at(key)->setDataInterval(data_interval_ms);
      ros_motors_.at(key)->setBraking(braking_strength);
    }
  }
  catch (const Phidget22Error& err)
  {
    RCLCPP_ERROR(get_logger(), "Motor: %s", err.what());
    throw;
  }

  if (publish_rate_ > 0.0)
  {
    double pub_msec = 1000.0 / publish_rate_;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
                                     std::bind(&MotorRosI::timerCallback, this));
  }
  else
  {
    // If we are *not* publishing periodically, then we are event driven and
    // will only publish when something changes (where "changes" is defined
    // by the libphidget22 library).  In that case, make sure to publish
    // once at the beginning to make sure there is *some* data.
    timerCallback();
  }
}

void MotorRosI::timerCallback()
{
  for (auto& ros_motor_pair : ros_motors_)
  {
    ros_motor_pair.second->publishDutyCycle();
    ros_motor_pair.second->publishBackEMF();
  }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::MotorRosI)
