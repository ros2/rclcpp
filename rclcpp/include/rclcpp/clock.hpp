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

class TimeJump
{
public:
  typedef enum ClockChange_t
  {
    ROS_TIME_NO_CHANGE,
    ROS_TIME_ACTIVATED,
    ROS_TIME_DEACTIVATED,
    SYSTEM_TIME_NO_CHANGE
  } ClockChange_t;

  ClockChange_t jump_type_;
  rcl_duration_t delta_;
};

class JumpThreshold
{
public:
  uint64_t min_forward_;
  uint64_t min_backward_;
  bool on_clock_change_;

  // Test if the threshold is exceeded by a TimeJump
  RCLCPP_PUBLIC
  bool
  is_exceeded(const TimeJump & jump);
};

class JumpCallback : public std::enable_shared_from_this<JumpCallback>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(JumpCallback)
  JumpCallback(
    std::function<void()> pre_callback,
    std::function<void(TimeJump)> post_callback,
    JumpThreshold & threshold);

  std::function<void()> pre_callback;
  std::function<void(TimeJump)> post_callback;
  JumpThreshold notice_threshold;
};

class Clock : public std::enable_shared_from_this<Clock>
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
  isROSTimeActive();

  RCLCPP_PUBLIC
  rcl_clock_type_t
  getClockType();

  // Add a callback to invoke if the jump threshold is exceeded.
  RCLCPP_PUBLIC
  JumpCallback::SharedPtr
  create_jump_callback(
    std::function<void()> pre_callback,
    std::function<void(TimeJump)> post_callback,
    JumpThreshold & threshold);

private:
  // A method for TimeSource to get a list of callbacks to invoke while updating
  RCLCPP_PUBLIC
  std::vector<JumpCallback::SharedPtr>
  get_triggered_callbacks(const TimeJump & jump);


  // Invoke callbacks that are valid and outside threshold.
  RCLCPP_PUBLIC
  static void invoke_prejump_callbacks(const std::vector<JumpCallback::SharedPtr> & callbacks);

  RCLCPP_PUBLIC
  static void invoke_postjump_callbacks(const std::vector<JumpCallback::SharedPtr> & callbacks, const TimeJump & jump);

  // Internal storage backed by rcl
  rcl_clock_t rcl_clock_;
  friend TimeSource;  // Allow TimeSource to access the rcl_clock_ datatype.

  std::mutex callback_list_mutex_;
  std::vector<std::weak_ptr<JumpCallback>> active_jump_callbacks_;
};

}  // namespace rclcpp

#endif  // RCLCPP__CLOCK_HPP_
