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

#include "phidgets_api/stepper.hpp"

namespace phidgets
{
Stepper::Stepper(const ChannelAddress& channel_address,
                 std::function<void()> position_change_handler,
                 std::function<void()> velocity_change_handler, std::function<void()> stopped_handler)
: PhidgetChannel(channel_address)
  , position_change_handler_(position_change_handler)
  , velocity_change_handler_(velocity_change_handler)
  , stopped_handler_(stopped_handler)
{
  PhidgetReturnCode ret = PhidgetStepper_create(&handle_);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to create Stepper handle for channel " + std::to_string(getChannelAddress().channel), ret);
  }

  openWaitForAttachment(reinterpret_cast<PhidgetHandle>(handle_), getChannelAddress());

  ret = PhidgetStepper_setOnPositionChangeHandler(handle_, PositionChangeHandler, this);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set position change handler for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }

  ret = PhidgetStepper_setOnVelocityChangeHandler(handle_, VelocityChangeHandler, this);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set velocity change handler for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }

  ret = PhidgetStepper_setOnStoppedHandler(handle_, StoppedHandler, this);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set stopped handler for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

Stepper::~Stepper()
{
  close(reinterpret_cast<PhidgetHandle>(handle_));
  PhidgetStepper_delete(&handle_);
}

double Stepper::getAcceleration() const
{
  double acceleration;
  PhidgetReturnCode ret = PhidgetStepper_getAcceleration(handle_, &acceleration);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get acceleration for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return acceleration;
}

void Stepper::setAcceleration(double acceleration) const
{
  PhidgetReturnCode ret = PhidgetStepper_setAcceleration(handle_, acceleration);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set acceleration for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getMinAcceleration() const
{
  double min_acceleration;
  PhidgetReturnCode ret = PhidgetStepper_getMinAcceleration(handle_, &min_acceleration);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get min acceleration for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return min_acceleration;
}

double Stepper::getMaxAcceleration() const
{
  double max_acceleration;
  PhidgetReturnCode ret = PhidgetStepper_getMaxAcceleration(handle_, &max_acceleration);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get max acceleration for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return max_acceleration;
}

bool Stepper::stepControlMode() const
{
  PhidgetStepper_ControlMode control_mode;
  PhidgetReturnCode ret = PhidgetStepper_getControlMode(handle_, &control_mode);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get control mode for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return (control_mode == CONTROL_MODE_STEP);
}

void Stepper::setStepControlMode(bool step_control_mode) const
{
  PhidgetReturnCode ret;
  if (step_control_mode)
  {
    ret = PhidgetStepper_setControlMode(handle_, CONTROL_MODE_STEP);
  }
  else
  {
    ret = PhidgetStepper_setControlMode(handle_, CONTROL_MODE_RUN);
  }
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set step control mode for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getCurrentLimit() const
{
  double current_limit;
  PhidgetReturnCode ret = PhidgetStepper_getCurrentLimit(handle_, &current_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get current limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return current_limit;
}

void Stepper::setCurrentLimit(double current_limit) const
{
  PhidgetReturnCode ret = PhidgetStepper_setCurrentLimit(handle_, current_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set current limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getMinCurrentLimit() const
{
  double min_current_limit;
  PhidgetReturnCode ret = PhidgetStepper_getMinCurrentLimit(handle_, &min_current_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get min current limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return min_current_limit;
}

double Stepper::getMaxCurrentLimit() const
{
  double max_current_limit;
  PhidgetReturnCode ret = PhidgetStepper_getMaxCurrentLimit(handle_, &max_current_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get max current limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return max_current_limit;
}

uint32_t Stepper::getDataInterval() const
{
  uint32_t data_interval_ms;
  PhidgetReturnCode ret = PhidgetStepper_getDataInterval(handle_, &data_interval_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get data interval for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return data_interval_ms;
}

void Stepper::setDataInterval(uint32_t data_interval_ms) const
{
  PhidgetReturnCode ret = PhidgetStepper_setDataInterval(handle_, data_interval_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set data interval for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

uint32_t Stepper::getMinDataInterval() const
{
  uint32_t min_data_interval_ms;
  PhidgetReturnCode ret = PhidgetStepper_getMinDataInterval(handle_, &min_data_interval_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get min data interval for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return min_data_interval_ms;
}

uint32_t Stepper::getMaxDataInterval() const
{
  uint32_t max_data_interval_ms;
  PhidgetReturnCode ret = PhidgetStepper_getMaxDataInterval(handle_, &max_data_interval_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get max data interval for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return max_data_interval_ms;
}

bool Stepper::getEngaged() const
{
  int engaged;
  PhidgetReturnCode ret = PhidgetStepper_getEngaged(handle_, &engaged);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get engaged for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return engaged;
}

void Stepper::setEngaged(bool engaged) const
{
  PhidgetReturnCode ret = PhidgetStepper_setEngaged(handle_, (int)engaged);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set engaged for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

void Stepper::enableFailsafe(uint32_t failsafe_time_ms) const
{
  PhidgetReturnCode ret = PhidgetStepper_enableFailsafe(handle_, failsafe_time_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to enable failsafe for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

uint32_t Stepper::getMinFailsafeTime() const
{
  uint32_t min_failsafe_time_ms;
  PhidgetReturnCode ret = PhidgetStepper_getMinFailsafeTime(handle_, &min_failsafe_time_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get min failsafe time for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return min_failsafe_time_ms;
}

uint32_t Stepper::getMaxFailsafeTime() const
{
  uint32_t max_failsafe_time_ms;
  PhidgetReturnCode ret = PhidgetStepper_getMaxFailsafeTime(handle_, &max_failsafe_time_ms);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get max failsafe time for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return max_failsafe_time_ms;
}

double Stepper::getHoldingCurrentLimit() const
{
  double holding_current_limit;
  PhidgetReturnCode ret = PhidgetStepper_getHoldingCurrentLimit(handle_, &holding_current_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get holding current limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return holding_current_limit;
}

void Stepper::setHoldingCurrentLimit(double holding_current_limit) const
{
  PhidgetReturnCode ret = PhidgetStepper_setHoldingCurrentLimit(handle_, holding_current_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set holding current limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getPosition()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return position_;
}

double Stepper::getMinPosition() const
{
  double min_position;
  PhidgetReturnCode ret = PhidgetStepper_getMinPosition(handle_, &min_position);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get min position for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return min_position;
}

double Stepper::getMaxPosition() const
{
  double max_position;
  PhidgetReturnCode ret = PhidgetStepper_getMaxPosition(handle_, &max_position);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get max position for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return max_position;
}

void Stepper::addPositionOffset(double position_offset) const
{
  PhidgetReturnCode ret = PhidgetStepper_addPositionOffset(handle_, position_offset);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to add position offset for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getRescaleFactor() const
{
  double rescale_factor;
  PhidgetReturnCode ret = PhidgetStepper_getRescaleFactor(handle_, &rescale_factor);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get rescale factor for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return rescale_factor;
}

void Stepper::setRescaleFactor(double rescale_factor) const
{
  PhidgetReturnCode ret = PhidgetStepper_setRescaleFactor(handle_, rescale_factor);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set rescale factor for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

void Stepper::resetFailsafe() const
{
  PhidgetReturnCode ret = PhidgetStepper_resetFailsafe(handle_);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to reset failsafe for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getTargetPosition() const
{
  double target_position;
  PhidgetReturnCode ret = PhidgetStepper_getTargetPosition(handle_, &target_position);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get target position for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return target_position;
}

void Stepper::setTargetPosition(double target_position) const
{
  PhidgetReturnCode ret = PhidgetStepper_setTargetPosition(handle_, target_position);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set target position for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getVelocity()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return velocity_;
}

double Stepper::getVelocityLimit() const
{
  double velocity_limit;
  PhidgetReturnCode ret = PhidgetStepper_getVelocityLimit(handle_, &velocity_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get velocity limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return velocity_limit;
}

void Stepper::setVelocityLimit(double velocity_limit) const
{
  PhidgetReturnCode ret = PhidgetStepper_setVelocityLimit(handle_, velocity_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to set velocity limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
}

double Stepper::getMinVelocityLimit() const
{
  double min_velocity_limit;
  PhidgetReturnCode ret = PhidgetStepper_getMinVelocityLimit(handle_, &min_velocity_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get min velocity limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return min_velocity_limit;
}

double Stepper::getMaxVelocityLimit() const
{
  double max_velocity_limit;
  PhidgetReturnCode ret = PhidgetStepper_getMaxVelocityLimit(handle_, &max_velocity_limit);
  if (ret != EPHIDGET_OK)
  {
    throw Phidget22Error("Failed to get max velocity limit for Stepper channel " + std::to_string(getChannelAddress().channel), ret);
  }
  return max_velocity_limit;
}

void Stepper::positionChangeHandler(double position) const
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = position;
  }
  position_change_handler_();
}

void Stepper::velocityChangeHandler(double velocity) const
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    velocity_ = velocity;
  }
  velocity_change_handler_();
}

void Stepper::stoppedHandler() const
{
  stopped_handler_();
}

void Stepper::PositionChangeHandler(PhidgetStepperHandle /* stepper_handle */, void* ctx, double position)
{
  ((Stepper*)ctx)->positionChangeHandler(position);
}

void Stepper::VelocityChangeHandler(PhidgetStepperHandle /* stepper_handle */, void* ctx, double velocity)
{
  ((Stepper*)ctx)->velocityChangeHandler(velocity);
}

void Stepper::StoppedHandler(PhidgetStepperHandle /* stepper_handle */, void* ctx)
{
  ((Stepper*)ctx)->stoppedHandler();
}

}  // namespace phidgets
