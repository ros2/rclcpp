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

#ifndef RCLCPP__TIME_HPP_
#define RCLCPP__TIME_HPP_

#include "builtin_interfaces/msg/time.hpp"

#include "rclcpp/visibility_control.hpp"

#include "rcl/time.h"

namespace rclcpp
{

class Clock;
class TimeSource;

class Time
{
public:
  RCLCPP_PUBLIC
  Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  Time(int32_t seconds, uint32_t nanoseconds, rcl_time_source_type_t clock = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  explicit Time(uint64_t nanoseconds = 0, rcl_time_source_type_t clock = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  Time(const Time & rhs);

  RCLCPP_PUBLIC
  Time(const builtin_interfaces::msg::Time & time_msg);  // NOLINT

  RCLCPP_PUBLIC
  explicit Time(const rcl_time_point_t & time_point);

  RCLCPP_PUBLIC
  virtual ~Time();

  RCLCPP_PUBLIC
  operator builtin_interfaces::msg::Time() const;

  RCLCPP_PUBLIC
  Time &
  operator=(const Time & rhs);

  RCLCPP_PUBLIC
  Time &
  operator=(const builtin_interfaces::msg::Time & time_msg);

  RCLCPP_PUBLIC
  bool
  operator==(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator!=(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<=(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>=(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  Time
  operator+(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  Time
  operator-(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  uint64_t
  nanoseconds() const;

private:
  rcl_time_point_t rcl_time_;
  friend Clock;  // Allow clock to manipulate internal data
};

// TODO(tfoote) move to own file
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

private:
  rcl_clock_t rcl_clock_;
  friend TimeSource;  // Allow TimeSource to acces the rcl_clock_ datatype.
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_
