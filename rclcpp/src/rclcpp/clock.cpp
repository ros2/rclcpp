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

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace rclcpp
{

JumpHandler::JumpHandler(
  pre_callback_t pre_callback,
  post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
: pre_callback(pre_callback),
  post_callback(post_callback),
  notice_threshold(threshold)
{}

Clock::Clock(rcl_clock_type_t clock_type)
{
  allocator_ = rcl_get_default_allocator();
  auto ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "failed to initialize rcl clock");
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

  auto ret = rcl_clock_get_now(&rcl_clock_, &now.rcl_time_.nanoseconds);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "could not get current time stamp");
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

  bool is_enabled = false;
  auto ret = rcl_is_enabled_ros_time_override(&rcl_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(
      ret, "Failed to check ros_time_override_status");
  }
  return is_enabled;
}

rcl_clock_t *
Clock::get_clock_handle() noexcept
{
  return &rcl_clock_;
}

rcl_clock_type_t
Clock::get_clock_type() const noexcept
{
  return rcl_clock_.type;
}

void
Clock::on_time_jump(
  const struct rcl_time_jump_t * time_jump,
  bool before_jump,
  void * user_data)
{
  const auto * handler = static_cast<JumpHandler *>(user_data);
  if (nullptr == handler) {
    return;
  }
  if (before_jump && handler->pre_callback) {
    handler->pre_callback();
  } else if (!before_jump && handler->post_callback) {
    handler->post_callback(*time_jump);
  }
}

JumpHandler::SharedPtr
Clock::create_jump_callback(
  JumpHandler::pre_callback_t pre_callback,
  JumpHandler::post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
{
  // Allocate a new jump handler
  JumpHandler::UniquePtr handler(new JumpHandler(pre_callback, post_callback, threshold));
  if (nullptr == handler) {
    throw std::bad_alloc{};
  }

  // Try to add the jump callback to the clock
  rcl_ret_t ret = rcl_clock_add_jump_callback(&rcl_clock_, threshold,
      Clock::on_time_jump, handler.get());
  if (RCL_RET_OK != ret) {
    exceptions::throw_from_rcl_error(ret, "Failed to add time jump callback");
  }

  // *INDENT-OFF*
  // create shared_ptr that removes the callback automatically when all copies are destructed
  // TODO(dorezyuk) UB, if the clock leaves scope before the JumpHandler
  return JumpHandler::SharedPtr(handler.release(), [this](JumpHandler * handler) noexcept {
    rcl_ret_t ret = rcl_clock_remove_jump_callback(&rcl_clock_, Clock::on_time_jump,
        handler);
    delete handler;
    handler = NULL;
    if (RCL_RET_OK != ret) {
      RCUTILS_LOG_ERROR("Failed to remove time jump callback");
    }
  });
  // *INDENT-ON*
}

}  // namespace rclcpp
