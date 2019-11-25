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

#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>

#include "phidgets_api/digital_output.hpp"
#include "phidgets_digital_output/digital_output_ros_i.hpp"
#include "phidgets_msgs/srv/set_digital_output.hpp"

namespace phidgets
{
DigitalOutputRosI::DigitalOutputRosI(const rclcpp::NodeOptions& options)
  : rclcpp::Node("phidgets_digital_output_node", options)
{
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  RCLCPP_INFO(get_logger(), "Starting Phidgets Digital Output");

  int serial_num = this->declare_parameter("serial", -1);  // default open any device

  int hub_port = this->declare_parameter("hub_port", 0);  // only used if the device is on a VINT hub_port

  // only used if the device is on a VINT hub_port
  bool is_hub_port_device = this->declare_parameter("is_hub_port_device", false);

  RCLCPP_INFO(get_logger(), "Connecting to Phidgets DigitalOutput serial %d, hub port %d ...", serial_num, hub_port);

  try
  {
    dos_ = std::make_unique<DigitalOutput>(serial_num, hub_port, is_hub_port_device);
  }
  catch (const Phidget22Error& err)
  {
    RCLCPP_ERROR(get_logger(), "DigitalOutput: %s", err.what());
    throw;
  }

  int n_out = dos_->getOutputCount();
  RCLCPP_INFO(get_logger(), "Connected to serial %d, %d output", dos_->getSerialNumber(), n_out);
  out_subs_.resize(n_out);
  for (int i = 0; i < n_out; i++)
  {
    char topicname[] = "digital_output00";
    snprintf(topicname, sizeof(topicname), "digital_output%02d", i);
    out_subs_[i] = std::make_unique<DigitalOutputetter>(dos_.get(), i, this, topicname);
  }
  out_srv_ = this->create_service<phidgets_msgs::srv::SetDigitalOutput>(
      "set_digital_output",
      std::bind(&DigitalOutputRosI::setSrvCallback, this, std::placeholders::_1, std::placeholders::_2));
}

void DigitalOutputRosI::setSrvCallback(const std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Request> req,
                                       std::shared_ptr<phidgets_msgs::srv::SetDigitalOutput::Response> res)
{
  dos_->setOutputtate(req->index, req->state);
  res->success = true;
}

DigitalOutputetter::DigitalOutputetter(DigitalOutput* dos, int index, DigitalOutputRosI* node,
                                       const std::string& topicname)
  : dos_(dos), index_(index)
{
  subscription_ = node->create_subscription<std_msgs::msg::Bool>(
      topicname, rclcpp::QoS(10), std::bind(&DigitalOutputetter::setMsgCallback, this, std::placeholders::_1));
}

void DigitalOutputetter::setMsgCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  dos_->setOutputtate(index_, msg->data);
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::DigitalOutputRosI)
