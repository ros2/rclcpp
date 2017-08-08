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

#include "rcl/time.h"

namespace rclcpp
{

class Time
{
public:
  static
  Time
  now(rcl_time_source_type_t clock = RCL_SYSTEM_TIME);

  Time(int32_t seconds, uint32_t nanoseconds, rcl_time_source_type_t clock = RCL_SYSTEM_TIME);

  explicit Time(uint64_t nanoseconds, rcl_time_source_type_t clcok = RCL_SYSTEM_TIME);

  Time(const builtin_interfaces::msg::Time & time_msg);  // NOLINT

  virtual ~Time();

  operator builtin_interfaces::msg::Time() const;

  void
  operator=(const builtin_interfaces::msg::Time & time_msg);

  bool
  operator==(const rclcpp::Time & rhs) const;

  bool
  operator<(const rclcpp::Time & rhs) const;

  bool
  operator<=(const rclcpp::Time & rhs) const;

  bool
  operator>=(const rclcpp::Time & rhs) const;

  bool
  operator>(const rclcpp::Time & rhs) const;

  Time
  operator+(const rclcpp::Time & rhs) const;

  Time
  operator-(const rclcpp::Time & rhs) const;

  uint64_t
  nanoseconds() const;

private:
  rcl_time_source_t rcl_time_source_;
  rcl_time_point_t rcl_time_;
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_
