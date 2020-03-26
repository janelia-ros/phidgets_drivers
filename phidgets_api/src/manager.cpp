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

#include "phidgets_api/manager.hpp"

namespace phidgets {

Manager::Manager(std::function<void(PhidgetHandle)> attach_handler,
                 std::function<void(PhidgetHandle)> detach_handler)
    : attach_handler_(attach_handler), detach_handler_(detach_handler)
{
    PhidgetReturnCode ret = PhidgetManager_create(&manager_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to create Manager handle", ret);
    }

    ret =
        PhidgetManager_setOnAttachHandler(manager_handle_, AttachHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set attach handler for Manager", ret);
    }

    ret =
        PhidgetManager_setOnDetachHandler(manager_handle_, DetachHandler, this);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to set detach handler for Manager", ret);
    }

    ret = PhidgetManager_open(manager_handle_);
    if (ret != EPHIDGET_OK)
    {
        throw Phidget22Error("Failed to open Manager", ret);
    }
}

Manager::~Manager()
{
    PhidgetManager_close(manager_handle_);
    PhidgetManager_delete(&manager_handle_);
}

void Manager::attachHandler(PhidgetHandle phidget_handle) const
{
    attach_handler_(phidget_handle);
}

void Manager::detachHandler(PhidgetHandle phidget_handle) const
{
    detach_handler_(phidget_handle);
}

void Manager::AttachHandler(PhidgetManagerHandle /* manager_handle */,
                            void *ctx, PhidgetHandle phidget_handle)
{
    (reinterpret_cast<Manager *>(ctx))->attachHandler(phidget_handle);
}

void Manager::DetachHandler(PhidgetManagerHandle /* manager_handle */,
                            void *ctx, PhidgetHandle phidget_handle)
{
    (reinterpret_cast<Manager *>(ctx))->detachHandler(phidget_handle);
}

}  // namespace phidgets
