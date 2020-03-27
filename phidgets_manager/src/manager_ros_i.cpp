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

#include "phidgets_manager/manager_ros_i.hpp"

namespace phidgets {

ManagerRosI::ManagerRosI(const rclcpp::NodeOptions& options)
    : rclcpp::Node("phidgets_manager_node", options)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    RCLCPP_INFO(get_logger(), "Starting Phidgets Manager");

    // We take the mutex here and don't unlock until the end of the constructor
    // to prevent a callback from trying to use the publisher before we are
    // finished setting up.
    std::lock_guard<std::mutex> lock(manager_mutex_);

    try
    {
        manager_ =
            std::make_unique<Manager>(std::bind(&ManagerRosI::attachCallback,
                                                this, std::placeholders::_1),
                                      std::bind(&ManagerRosI::detachCallback,
                                                this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Connected to manager");

    } catch (const Phidget22Error& err)
    {
        RCLCPP_ERROR(get_logger(), "Manager: %s", err.what());
        throw;
    }
}

void ManagerRosI::attachCallback(PhidgetHandle phidget_handle)
{
    std::lock_guard<std::mutex> lock(manager_mutex_);
    std::stringstream ss;
    int serial_number;
    const char* name;

    manager_->getDeviceName(phidget_handle, &name);
    manager_->getDeviceSerialNumber(phidget_handle, &serial_number);

    ss << "Attached Phidget: " << name << ", Serial Number: " << serial_number
       << "\n";
    RCLCPP_INFO(get_logger(), ss.str().c_str());
}

void ManagerRosI::detachCallback(PhidgetHandle phidget_handle)
{
    std::lock_guard<std::mutex> lock(manager_mutex_);
    std::stringstream ss;
    int serial_number;
    const char* name;

    manager_->getDeviceName(phidget_handle, &name);
    manager_->getDeviceSerialNumber(phidget_handle, &serial_number);

    ss << "Detached Phidget: " << name << ", Serial Number: " << serial_number
       << "\n";
    RCLCPP_INFO(get_logger(), ss.str().c_str());
}

}  // namespace phidgets

RCLCPP_COMPONENTS_REGISTER_NODE(phidgets::ManagerRosI)
