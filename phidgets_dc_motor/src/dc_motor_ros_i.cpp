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

#include "phidgets_dc_motor/dc_motor_ros_i.hpp"

namespace phidgets
{
// RosDCMotor::RosDCMotor(rclcpp::Node* node, const ChannelAddress& channel_address)
RosDCMotor::RosDCMotor(DCMotorRosI* node, const ChannelAddress& channel_address)
: DCMotor(channel_address, std::bind(&RosDCMotor::velocityUpdateHandler, this), std::bind(&RosDCMotor::backEMFChangeHandler, this))

{
  std::lock_guard<std::mutex> lock(ros_dc_motor_mutex_);

  node_ = node;

  char interface_name[INTERFACE_NAME_LENGTH_MAX];

  snprintf(interface_name, INTERFACE_NAME_LENGTH_MAX, "motor_back_emf");
  back_emf_publisher_ = node_->create_publisher<std_msgs::msg::Float64>(interface_name, 1);

  snprintf(interface_name, INTERFACE_NAME_LENGTH_MAX, "set_motor_velocity");
  velocity_subscription_ = node_->create_subscription<std_msgs::msg::Float64>(
      interface_name, rclcpp::QoS(1), std::bind(&RosDCMotor::velocityCallback, this, std::placeholders::_1));
}

void RosDCMotor::velocityUpdateHandler()
{
  node_->publishJointStatesHandler();
}

void RosDCMotor::backEMFChangeHandler()
{
  std::lock_guard<std::mutex> lock(ros_dc_motor_mutex_);
  if (backEMFSensingSupported())
  {
    auto msg = std_msgs::msg::Float64();
    msg.data = getBackEMF();
    back_emf_publisher_->publish(msg);
  }
}

void RosDCMotor::velocityCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  try
  {
    setTargetVelocity(msg->data);
  }
  catch (const phidgets::Phidget22Error& err)
  {
    // If the data was wrong, the lower layers will throw an exception; just
    // catch and ignore here so we don't crash the node.
  }
}

DCMotorRosI::DCMotorRosI(const rclcpp::NodeOptions& options) : rclcpp::Node("phidgets_dc_motor_node", options)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  RCLCPP_INFO(get_logger(), "Starting Phidgets DCMotor");

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

  frame_id_ = this->declare_parameter("frame_id", "dc_motor");

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 100);

  RCLCPP_INFO(get_logger(), "Connecting to Phidgets DCMotor serial %d, hub port %d ...", channel_address.serial_number,
              channel_address.hub_port);

  int n_motors;
  try
  {
    n_motors = 1;

    for (int i = 0; i < n_motors; i++)
    {
      std::string key = std::to_string(i);
      ros_dc_motors_[key] = std::make_unique<RosDCMotor>(this, channel_address);
      RCLCPP_INFO(get_logger(), "Connected to serial %d, %d motor",
                  ros_dc_motors_.at(key)->getChannelAddress().serial_number, n_motors);

      ros_dc_motors_.at(key)->setDataInterval(data_interval_ms);
      ros_dc_motors_.at(key)->setBraking(braking_strength);
    }
  }
  catch (const Phidget22Error& err)
  {
    RCLCPP_ERROR(get_logger(), "DCMotor: %s", err.what());
    throw;
  }

  if (publish_rate_ > 0.0)
  {
    double pub_msec = 1000.0 / publish_rate_;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
                                     std::bind(&DCMotorRosI::timerCallback, this));
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

void DCMotorRosI::publishJointStatesHandler()
{
  if (publish_rate_ <= 0.0)
  {
    publishJointStates();
  }
}

void DCMotorRosI::publishJointStates()
{
  sensor_msgs::msg::JointState msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  for (auto& ros_motor_pair : ros_dc_motors_)
  {
    msg.name.push_back(ros_motor_pair.first);
    msg.velocity.push_back(ros_motor_pair.second->getVelocity());
  }
  joint_state_pub_->publish(msg);
}

void DCMotorRosI::timerCallback()
{
  publishJointStates();
  for (auto& ros_motor_pair : ros_dc_motors_)
  {
    ros_motor_pair.second->backEMFChangeHandler();
  }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::DCMotorRosI)
