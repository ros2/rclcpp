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

#ifndef RCLCPP__CLOCK_HPP_
#define RCLCPP__CLOCK_HPP_

#include <functional>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"

namespace rclcpp
{

class TimeSource;

class JumpHandler
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(JumpHandler)
  JumpHandler(
    std::function<void()> pre_callback,
    std::function<void(const rcl_time_jump_t &)> post_callback,
    const rcl_jump_threshold_t & threshold);

  std::function<void()> pre_callback;
  std::function<void(const rcl_time_jump_t &)> post_callback;
  rcl_jump_threshold_t notice_threshold;
};

class Clock
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Clock)

  RCLCPP_PUBLIC
  explicit Clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  ~Clock();

  RCLCPP_PUBLIC
  Time
  now();

  RCLCPP_PUBLIC
  bool
  ros_time_is_active();

  RCLCPP_PUBLIC
  rcl_clock_t *
  get_clock_handle();

  RCLCPP_PUBLIC
  rcl_clock_type_t
  get_clock_type();

  // Add a callback to invoke if the jump threshold is exceeded.
  /**
   * These callback functions must remain valid as long as the
   * returned shared pointer is valid.
   */
  RCLCPP_PUBLIC
  JumpHandler::SharedPtr
  create_jump_callback(
    std::function<void()> pre_callback,
    std::function<void(const rcl_time_jump_t &)> post_callback,
    const rcl_jump_threshold_t & threshold);

private:
  // Invoke time jump callback
  RCLCPP_PUBLIC
  static void
  on_time_jump(
    const struct rcl_time_jump_t * time_jump,
    bool before_jump,
    void * user_data);

  /// Internal storage backed by rcl
  rcl_clock_t rcl_clock_;
  friend TimeSource;  /// Allow TimeSource to access the rcl_clock_ datatype.
  rcl_allocator_t allocator_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLOCK_HPP_
