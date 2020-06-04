/*
 * Copyright (c) 2020, Howard Hughes Medical Institute
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

#include "phidgets_toggle_direct/toggle_direct_ros_i.hpp"

namespace phidgets {

ToggleDirectRosI::ToggleDirectRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_toggle_direct_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets ToggleDirect");

    int serial_num =
        this->declare_parameter("serial", -1);  // default open any device

    int input_hub_port = this->declare_parameter(
        "input_hub_port", 0);

    bool input_is_hub_port_device =
        this->declare_parameter("is_hub_port_device", false);

    constexpr int input_channel = 0;

    int output_hub_port = this->declare_parameter(
        "output_hub_port", 5);

    bool output_is_hub_port_device =
        this->declare_parameter("is_hub_port_device", true);

    constexpr int output_channel = 0;

    std::lock_guard<std::mutex> lock(mutex_);

    RCLCPP_INFO(
        get_logger(),
        "Connecting to Phidgets ToggleDirect serial %d, hub port %d ...",
        serial_num, input_hub_port);

    PhidgetReturnCode ret = PhidgetDigitalInput_create(&di_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
          "Failed to create DigitalInput handle ",
            ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(di_handle_),
                                   serial_num, input_hub_port, input_is_hub_port_device,
                                   input_channel);
    ret = PhidgetDigitalInput_setOnStateChangeHandler(di_handle_,
                                                      StateChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
          "Failed to set change handler for DigitalInput",
            ret);
    }

    RCLCPP_INFO(
        get_logger(),
        "Connecting to Phidgets DigitalInputs serial %d, hub port %d ...",
        serial_num, output_hub_port);

    ret = PhidgetDigitalOutput_create(&do_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
          "Failed to create DigitalOutput handle",
            ret);
    }

    helpers::openWaitForAttachment(reinterpret_cast<PhidgetHandle>(do_handle_),
                                   serial_num, output_hub_port, output_is_hub_port_device,
                                   output_channel);

}

ToggleDirectRosI::~ToggleDirectRosI()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(di_handle_);
    helpers::closeAndDelete(&handle);
}

void ToggleDirectRosI::stateChangeHandler(int state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "State: %d", state);
  setOutputState(state);
}

void ToggleDirectRosI::StateChangeHandler(
    PhidgetDigitalInputHandle /* input_handle */, void *ctx, int state)
{
    (reinterpret_cast<ToggleDirectRosI *>(ctx))->stateChangeHandler(state);
}

void ToggleDirectRosI::setOutputState(bool state) const
{
    PhidgetReturnCode ret = PhidgetDigitalOutput_setState(do_handle_, state);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set start for DigitalOutput", ret);
    }
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::ToggleDirectRosI)
