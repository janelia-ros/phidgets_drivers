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

#ifndef PHIDGETS_API_MANAGER_H
#define PHIDGETS_API_MANAGER_H

#include <cstddef>
#include <functional>

#include <libphidget22/phidget22.h>

namespace phidgets {

class Manager final
{
  public:
    PHIDGET22_NO_COPY_NO_MOVE_NO_ASSIGN(Manager)

    explicit Manager(std::function<void(PhidgetHandle)> attach_handler,
                     std::function<void(PhidgetHandle)> detach_handler);

    ~Manager();

    void attachHandler(PhidgetHandle phidget_handle) const;

    void detachHandler(PhidgetHandle phidget_handle) const;

  private:
    std::function<void(PhidgetHandle)> attach_handler_;
    std::function<void(PhidgetHandle)> detach_handler_;
    PhidgetManagerHandle manager_handle_{nullptr};
    bool back_emf_sensing_supported_;

    static void AttachHandler(PhidgetManagerHandle manager_handle, void *ctx,
                              PhidgetHandle phidget_handle);
    static void DetachHandler(PhidgetManagerHandle manager_handle, void *ctx,
                              PhidgetHandle phidget_handle);
};

}  // namespace phidgets

#endif  // PHIDGETS_API_MANAGER_H
