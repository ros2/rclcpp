// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/clock.hpp"

#include <memory>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace rclcpp
{
bool
JumpThreshold::is_exceeded(const TimeJump & jump)
{
  if (on_clock_change_ &&
    (jump.jump_type_ == TimeJump::ClockChange_t::ROS_TIME_ACTIVATED ||
    jump.jump_type_ == TimeJump::ClockChange_t::ROS_TIME_DEACTIVATED))
  {
    return true;
  }
  if ((uint64_t)jump.delta_.nanoseconds > min_forward_ ||
    (uint64_t)jump.delta_.nanoseconds < min_backward_)
  {
    return true;
  }
  return false;
}

JumpHandler::JumpHandler(
  std::function<void()> pre_callback,
  std::function<void(TimeJump)> post_callback,
  JumpThreshold & threshold)
: pre_callback(pre_callback),
  post_callback(post_callback),
  notice_threshold(threshold)
{}

Clock::Clock(rcl_clock_type_t clock_type)
{
  auto ret = rcl_clock_init(clock_type, &rcl_clock_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not get current time stamp");
  }
}

Clock::~Clock()
{
  auto ret = rcl_clock_fini(&rcl_clock_);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
  }
}

Time
Clock::now()
{
  Time now(0, 0, rcl_clock_.type);

  auto ret = rcl_clock_get_now(&rcl_clock_, &now.rcl_time_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not get current time stamp");
  }

  return now;
}

bool
Clock::ros_time_is_active()
{
  if (!rcl_clock_valid(&rcl_clock_)) {
    RCUTILS_LOG_ERROR("ROS time not valid!");
    return false;
  }

  bool is_enabled;
  auto ret = rcl_is_enabled_ros_time_override(&rcl_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "Failed to check ros_time_override_status");
  }
  return is_enabled;
}

rcl_clock_type_t
Clock::get_clock_type()
{
  return rcl_clock_.type;
}

rclcpp::JumpHandler::SharedPtr
Clock::create_jump_callback(
  std::function<void()> pre_callback,
  std::function<void(const TimeJump &)> post_callback,
  JumpThreshold & threshold)
{
  // JumpHandler jump_callback;
  auto jump_callback =
    std::make_shared<rclcpp::JumpHandler>(pre_callback, post_callback, threshold);
  {
    std::lock_guard<std::mutex> guard(callback_list_mutex_);
    active_jump_handlers_.push_back(jump_callback);
  }
  return jump_callback;
}

std::vector<JumpHandler::SharedPtr>
Clock::get_triggered_callbacks(const TimeJump & jump)
{
  std::vector<JumpHandler::SharedPtr> callbacks;
  std::lock_guard<std::mutex> guard(callback_list_mutex_);
  for (auto wjcb = active_jump_handlers_.begin(); wjcb != active_jump_handlers_.end(); wjcb++) {
    if (auto jcb = wjcb->lock()) {
      if (jcb->notice_threshold.is_exceeded(jump)) {
        callbacks.push_back(jcb);
      }
    } else {
      active_jump_handlers_.erase(wjcb);
    }
  }
  return callbacks;
}

void
Clock::invoke_prejump_callbacks(const std::vector<JumpHandler::SharedPtr> & callbacks)
{
  for (const auto cb : callbacks) {
    cb->pre_callback();
  }
}

void
Clock::invoke_postjump_callbacks(const std::vector<JumpHandler::SharedPtr> & callbacks,
  const TimeJump & jump)
{
  for (auto cb : callbacks) {
    cb->post_callback(jump);
  }
}

}  // namespace rclcpp
