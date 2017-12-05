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

#include <chrono>
#include <string>

using rclcpp::TimerBase;

TimerBase::TimerBase(std::chrono::nanoseconds period)
{
  if (rcl_timer_init(
      &timer_handle_, period.count(), nullptr,
      rcl_get_default_allocator()) != RCL_RET_OK)
  {
    fprintf(stderr, "Couldn't initialize rcl timer handle: %s\n", rcl_get_error_string_safe());
  }
}

TimerBase::~TimerBase()
{}

void
TimerBase::cancel()
{
  if (rcl_timer_cancel(&timer_handle_) != RCL_RET_OK) {
    throw std::runtime_error(std::string("Couldn't cancel timer: ") + rcl_get_error_string_safe());
  }
}

void
TimerBase::reset()
{
  if (rcl_timer_reset(&timer_handle_) != RCL_RET_OK) {
    throw std::runtime_error(std::string("Couldn't reset timer: ") + rcl_get_error_string_safe());
  }
}

bool
TimerBase::is_ready()
{
  bool ready = false;
  if (rcl_timer_is_ready(&timer_handle_, &ready) != RCL_RET_OK) {
    throw std::runtime_error(std::string("Failed to check timer: ") + rcl_get_error_string_safe());
  }
  return ready;
}

std::chrono::nanoseconds
TimerBase::time_until_trigger()
{
  int64_t time_until_next_call = 0;
  if (rcl_timer_get_time_until_next_call(&timer_handle_, &time_until_next_call) != RCL_RET_OK) {
    throw std::runtime_error(
            std::string("Timer could not get time until next call: ") +
            rcl_get_error_string_safe());
  }
  return std::chrono::nanoseconds(time_until_next_call);
}

const rcl_timer_t *
TimerBase::get_timer_handle()
{
  return &timer_handle_;
}
