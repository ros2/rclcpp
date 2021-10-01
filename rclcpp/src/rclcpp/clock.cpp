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
#include <thread>

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace rclcpp
{

class Clock::Impl
{
public:
  explicit Impl(rcl_clock_type_t clock_type)
  : allocator_{rcl_get_default_allocator()}
  {
    rcl_ret_t ret = rcl_clock_init(clock_type, &rcl_clock_, &allocator_);
    if (ret != RCL_RET_OK) {
      exceptions::throw_from_rcl_error(ret, "failed to initialize rcl clock");
    }
  }

  ~Impl()
  {
    rcl_ret_t ret = rcl_clock_fini(&rcl_clock_);
    if (ret != RCL_RET_OK) {
      RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
    }
  }

  rcl_clock_t rcl_clock_;
  rcl_allocator_t allocator_;
  std::mutex clock_mutex_;
};

JumpHandler::JumpHandler(
  pre_callback_t pre_callback,
  post_callback_t post_callback,
  const rcl_jump_threshold_t & threshold)
: pre_callback(pre_callback),
  post_callback(post_callback),
  notice_threshold(threshold)
{}

Clock::Clock(rcl_clock_type_t clock_type)
: impl_(new Clock::Impl(clock_type)) {}

Clock::~Clock() {}

Time
Clock::now()
{
  Time now(0, 0, impl_->rcl_clock_.type);

  auto ret = rcl_clock_get_now(&impl_->rcl_clock_, &now.rcl_time_.nanoseconds);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(ret, "could not get current time stamp");
  }

  return now;
}

bool
Clock::ros_time_is_active()
{
  if (!rcl_clock_valid(&impl_->rcl_clock_)) {
    RCUTILS_LOG_ERROR("ROS time not valid!");
    return false;
  }

  bool is_enabled = false;
  auto ret = rcl_is_enabled_ros_time_override(&impl_->rcl_clock_, &is_enabled);
  if (ret != RCL_RET_OK) {
    exceptions::throw_from_rcl_error(
      ret, "Failed to check ros_time_override_status");
  }
  return is_enabled;
}

rcl_clock_t *
Clock::get_clock_handle() noexcept
{
  return &impl_->rcl_clock_;
}

rcl_clock_type_t
Clock::get_clock_type() const noexcept
{
  return impl_->rcl_clock_.type;
}

std::mutex &
Clock::get_clock_mutex() noexcept
{
  return impl_->clock_mutex_;
}

void
Clock::on_time_jump(
  const rcl_time_jump_t * time_jump,
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

  {
    std::lock_guard<std::mutex> clock_guard(impl_->clock_mutex_);
    // Try to add the jump callback to the clock
    rcl_ret_t ret = rcl_clock_add_jump_callback(
      &impl_->rcl_clock_, threshold, Clock::on_time_jump,
      handler.get());
    if (RCL_RET_OK != ret) {
      exceptions::throw_from_rcl_error(ret, "Failed to add time jump callback");
    }
  }

  std::weak_ptr<Clock::Impl> weak_impl = impl_;
  // *INDENT-OFF*
  // create shared_ptr that removes the callback automatically when all copies are destructed
  return JumpHandler::SharedPtr(handler.release(), [weak_impl](JumpHandler * handler) noexcept {
    auto shared_impl = weak_impl.lock();
    if (shared_impl) {
      std::lock_guard<std::mutex> clock_guard(shared_impl->clock_mutex_);
      rcl_ret_t ret = rcl_clock_remove_jump_callback(&shared_impl->rcl_clock_,
          Clock::on_time_jump, handler);
      if (RCL_RET_OK != ret) {
        RCUTILS_LOG_ERROR("Failed to remove time jump callback");
      }
    }
    delete handler;
  });
  // *INDENT-ON*
}

}  // namespace rclcpp
