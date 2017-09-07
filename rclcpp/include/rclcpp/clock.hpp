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

#include "rclcpp/time.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"

namespace rclcpp
{

class TimeSource;

class Clock
{
public:
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

private:
  rcl_clock_t rcl_clock_;
  friend TimeSource;  // Allow TimeSource to access the rcl_clock_ datatype.
};

}  // namespace rclcpp

#endif  // RCLCPP__CLOCK_HPP_
