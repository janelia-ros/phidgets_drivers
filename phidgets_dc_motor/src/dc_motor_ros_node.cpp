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

#include "phidgets_dc_motor/dc_motor_ros_node.hpp"

namespace phidgets
{
DcMotorRos::DcMotorRos(DcMotorRosNode* node, const ChannelAddress& channel_address)
  : DcMotor(channel_address, std::bind(&DcMotorRos::velocityUpdateHandler, this),
            std::bind(&DcMotorRos::backEmfChangeHandler, this))

{
  node_ = node;
}

void DcMotorRos::velocityUpdateHandler()
{
  node_->publishJointStateHandler();
}

void DcMotorRos::backEmfChangeHandler()
{
  node_->publishDcMotorStateHandler();
}

DcMotorRosNode::DcMotorRosNode(const rclcpp::NodeOptions& options) : rclcpp::Node("phidgets_dc_motor_node", options)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  RCLCPP_INFO(get_logger(), "Starting Phidgets DcMotor");

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

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_state", 100);
  dc_motor_state_pub_ = this->create_publisher<phidgets_msgs::msg::DcMotorState>("dc_motor_state", 100);
  joint_jog_sub_ = this->create_subscription<phidgets_msgs::msg::JointJog>(
      "joint_jog", rclcpp::QoS(1), std::bind(&DcMotorRosNode::jointJogCallback, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "Connecting to Phidgets DcMotor serial %d, hub port %d ...", channel_address.serial_number,
              channel_address.hub_port);

  int n_motors;
  try
  {
    n_motors = 1;

    for (int i = 0; i < n_motors; i++)
    {
      std::string name = std::to_string(i);
      dc_motors_[name] = std::make_unique<DcMotorRos>(this, channel_address);
      RCLCPP_INFO(get_logger(), "Connected to serial %d, %d motor",
                  dc_motors_.at(name)->getChannelAddress().serial_number, n_motors);

      dc_motors_.at(name)->setDataInterval(data_interval_ms);
      dc_motors_.at(name)->setBraking(braking_strength);
    }
  }
  catch (const Phidget22Error& err)
  {
    RCLCPP_ERROR(get_logger(), "DcMotor: %s", err.what());
    throw;
  }

  if (publish_rate_ > 0.0)
  {
    double pub_msec = 1000.0 / publish_rate_;
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int64_t>(pub_msec)),
                                     std::bind(&DcMotorRosNode::timerCallback, this));
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

void DcMotorRosNode::publishJointStateHandler()
{
  if (publish_rate_ <= 0.0)
  {
    publishJointState();
  }
}

void DcMotorRosNode::publishDcMotorStateHandler()
{
  if (publish_rate_ <= 0.0)
  {
    publishDcMotorState();
  }
}

void DcMotorRosNode::publishJointState()
{
  sensor_msgs::msg::JointState msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  for (const auto& dc_motor_pair : dc_motors_)
  {
    const std::string& name = dc_motor_pair.first;
    const std::unique_ptr<DcMotorRos>& dc_motor = dc_motor_pair.second;
    msg.name.push_back(name);
    msg.velocity.push_back(dc_motor->getVelocity());
  }
  joint_state_pub_->publish(msg);
}

void DcMotorRosNode::publishDcMotorState()
{
  phidgets_msgs::msg::DcMotorState msg;

  msg.header.stamp = this->now();
  msg.header.frame_id = frame_id_;

  for (const auto& dc_motor_pair : dc_motors_)
  {
    const std::string& name = dc_motor_pair.first;
    const std::unique_ptr<DcMotorRos>& dc_motor = dc_motor_pair.second;
    if (dc_motor->backEmfSensingSupported())
    {
      msg.name.push_back(name);
      msg.back_emf.push_back(dc_motor->getBackEmf());
    }
  }
  dc_motor_state_pub_->publish(msg);
}

void DcMotorRosNode::jointJogCallback(const phidgets_msgs::msg::JointJog::SharedPtr msg)
{
  try
  {
    if (msg->joint_names.size() != msg->velocities.size())
    {
      return;
    }
    for (size_t i = 0; i<msg->joint_names.size(); ++i)
    {
      const auto& name = msg->joint_names[i];
      const auto& velocity = msg->velocities[i];
      if (dc_motors_.count(name))
      {
        dc_motors_.at(name)->setTargetVelocity(velocity);
      }
    }
  }
  catch (const phidgets::Phidget22Error& err)
  {
    // If the data was wrong, the lower layers will throw an exception; just
    // catch and ignore here so we don't crash the node.
  }
}

void DcMotorRosNode::timerCallback()
{
  publishJointState();
  publishDcMotorState();
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::DcMotorRosNode)
