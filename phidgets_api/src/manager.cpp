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

#include <functional>
#include <string>

#include <libphidget22/phidget22.h>

#include "phidgets_api/manager.hpp"
#include "phidgets_api/phidget22.hpp"

namespace phidgets {

Manager::Manager(std::function<void(PhidgetHandle)> attach_handler,
                 std::function<void(PhidgetHandle)> detach_handler)
    : attach_handler_(attach_handler), detach_handler_(detach_handler)
{
    PhidgetReturnCode ret = PhidgetDCManager_create(&manager_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Manager handle for channel " +
                                 std::to_string(channel),
                             ret);
    }

    helpers::openWaitForAttachment(
        reinterpret_cast<PhidgetHandle>(manager_handle_), serial_number,
        hub_port, is_hub_port_device, channel);

    ret = PhidgetDCManager_setOnVelocityUpdateHandler(
        manager_handle_, DutyCycleChangeHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set duty cycle update handler for Manager channel " +
                std::to_string(channel),
            ret);
    }

    back_emf_sensing_supported_ = true;
    ret = PhidgetDCManager_setBackEMFSensingState(manager_handle_, 1);
    if (ret == EPHIDGET_UNSUPPORTED)
    {
        back_emf_sensing_supported_ = false;
    } else if (ret == EPHIDGET_OK)
    {
        ret = PhidgetDCManager_setOnBackEMFChangeHandler(
            manager_handle_, BackEMFChangeHandler, this);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to set back EMF update handler for Manager channel " +
                    std::to_string(channel),
                ret);
        }
    } else
    {
        throw Phidget22Error(
            "Failed to set back EMF sensing state Manager channel " +
                std::to_string(channel),
            ret);
    }

    if (serial_number_ == -1)
    {
        ret = Phidget_getDeviceSerialNumber(
            reinterpret_cast<PhidgetHandle>(manager_handle_), &serial_number_);
        if (ret != EPHIDGET_OK)
        {
            throw Phidget22Error(
                "Failed to get serial number for manager channel " +
                    std::to_string(channel),
                ret);
        }
    }
}

Manager::~Manager()
{
    PhidgetHandle handle = reinterpret_cast<PhidgetHandle>(manager_handle_);
    helpers::closeAndDelete(&handle);
}

int32_t Manager::getSerialNumber() const noexcept
{
    return serial_number_;
}

double Manager::getDutyCycle() const
{
    double duty_cycle;
    PhidgetReturnCode ret =
        PhidgetDCManager_getVelocity(manager_handle_, &duty_cycle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get duty cycle for Manager channel " +
                                 std::to_string(channel_),
                             ret);
    }
    return duty_cycle;
}

void Manager::setDutyCycle(double duty_cycle) const
{
    PhidgetReturnCode ret =
        PhidgetDCManager_setTargetVelocity(manager_handle_, duty_cycle);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set duty cycle for Manager channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

double Manager::getAcceleration() const
{
    double accel;
    PhidgetReturnCode ret =
        PhidgetDCManager_getAcceleration(manager_handle_, &accel);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get acceleration for Manager channel " +
                                 std::to_string(channel_),
                             ret);
    }
    return accel;
}

void Manager::setAcceleration(double acceleration) const
{
    PhidgetReturnCode ret =
        PhidgetDCManager_setAcceleration(manager_handle_, acceleration);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set acceleration for Manager channel " +
                                 std::to_string(channel_),
                             ret);
    }
}

bool Manager::backEMFSensingSupported() const
{
    return back_emf_sensing_supported_;
}

double Manager::getBackEMF() const
{
    if (!back_emf_sensing_supported_)
    {
        throw Phidget22Error("Back EMF sensing not supported",
                             EPHIDGET_UNSUPPORTED);
    }
    double backemf;
    PhidgetReturnCode ret =
        PhidgetDCManager_getBackEMF(manager_handle_, &backemf);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to get back EMF for Manager channel " +
                                 std::to_string(channel_),
                             ret);
    }
    return backemf;
}

void Manager::setDataInterval(uint32_t data_interval_ms) const
{
    PhidgetReturnCode ret =
        PhidgetDCManager_setDataInterval(manager_handle_, data_interval_ms);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set data interval for Manager channel " +
                std::to_string(channel_),
            ret);
    }
}

double Manager::getBraking() const
{
    double braking;
    PhidgetReturnCode ret =
        PhidgetDCManager_getBrakingStrength(manager_handle_, &braking);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to get braking strength for Manager channel " +
                std::to_string(channel_),
            ret);
    }
    return braking;
}

void Manager::setBraking(double braking) const
{
    PhidgetReturnCode ret =
        PhidgetDCManager_setTargetBrakingStrength(manager_handle_, braking);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error(
            "Failed to set braking strength for Manager channel " +
                std::to_string(channel_),
            ret);
    }
}

void Manager::dutyCycleChangeHandler(double duty_cycle) const
{
    duty_cycle_change_handler_(channel_, duty_cycle);
}

void Manager::backEMFChangeHandler(double back_emf) const
{
    back_emf_change_handler_(channel_, back_emf);
}

void Manager::DutyCycleChangeHandler(PhidgetManagerHandle /* manager_handle */,
                                     void *ctx, double duty_cycle)
{
    (reinterpret_cast<Manager *>(ctx))->dutyCycleChangeHandler(duty_cycle);
}

void Manager::BackEMFChangeHandler(PhidgetManagerHandle /* manager_handle */,
                                   void *ctx, double back_emf)
{
    (reinterpret_cast<Manager *>(ctx))->backEMFChangeHandler(back_emf);
}

}  // namespace phidgets
