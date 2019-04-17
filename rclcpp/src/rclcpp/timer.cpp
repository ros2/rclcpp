// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/timer.hpp"

#include <memory>

#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"


#include "rcl/error_handling.h"

namespace rclcpp
{

TimerBase::TimerBase(
  Clock::SharedPtr clock,
  std::chrono::nanoseconds period,
  Context::SharedPtr context)
: clock_(clock), timer_handle_(nullptr)
{
  if (nullptr == context) {
    context = contexts::default_context::get_global_default_context();
  }

  auto rcl_context = context->get_rcl_context();

  timer_handle_ = std::shared_ptr<rcl_timer_t>(
    new rcl_timer_t, [ = ](rcl_timer_t * timer) mutable noexcept
    {
      if (rcl_timer_fini(timer) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "Failed to clean up rcl timer handle: %s", rcl_get_error_string().str);
        rcl_reset_error();
      }
      delete timer;
      // Captured shared pointers by copy, reset to make sure timer is finalized before clock
      clock.reset();
      rcl_context.reset();
    });

  *timer_handle_.get() = rcl_get_zero_initialized_timer();

  rcl_clock_t * clock_handle = clock_->get_clock_handle();
  if (rcl_timer_init(
      timer_handle_.get(), clock_handle, rcl_context.get(), period.count(), nullptr,
      rcl_get_default_allocator()) != RCL_RET_OK)
  {
    RCUTILS_LOG_ERROR_NAMED(
      "rclcpp",
      "Couldn't initialize rcl timer handle: %s\n", rcl_get_error_string().str);
    rcl_reset_error();
  }
}

void
TimerBase::cancel()
{
  const auto res = rcl_timer_cancel(timer_handle_.get());
  if (RCL_RET_OK != res) {
    exceptions::throw_from_rcl_error(res, "Couldn't cancel timer");
  }
}

bool
TimerBase::is_canceled()
{
  bool is_canceled = false;
  rcl_ret_t ret = rcl_timer_is_canceled(timer_handle_.get(), &is_canceled);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "Couldn't get timer cancelled state");
  }
  return is_canceled;
}

void
TimerBase::reset()
{
  const auto res = rcl_timer_reset(timer_handle_.get());
  if (RCL_RET_OK != res) {
    exceptions::throw_from_rcl_error(res, "Couldn't reset timer");
  }
}

bool
TimerBase::is_ready()
{
  bool ready = false;
  const auto res = rcl_timer_is_ready(timer_handle_.get(), &ready);
  if (RCL_RET_OK != res) {
    exceptions::throw_from_rcl_error(res, "Failed to check timer");
  }
  return ready;
}

std::chrono::nanoseconds
TimerBase::time_until_trigger()
{
  int64_t time_until_next_call = 0;
  const auto res = rcl_timer_get_time_until_next_call(timer_handle_.get(),
      &time_until_next_call);
  if (RCL_RET_OK != res) {
    exceptions::throw_from_rcl_error(res,
      "Timer could not get time until next call");
  }
  return std::chrono::nanoseconds(time_until_next_call);
}

std::shared_ptr<const rcl_timer_t>
TimerBase::get_timer_handle()
{
  return timer_handle_;
}

}  // namespace rclcpp
