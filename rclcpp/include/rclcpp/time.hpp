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

#include "rclcpp/duration.hpp"

namespace rclcpp
{

class Clock;

class Time
{
public:
  RCLCPP_PUBLIC
  Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  explicit Time(int64_t nanoseconds = 0, rcl_clock_type_t clock = RCL_SYSTEM_TIME);

  RCLCPP_PUBLIC
  Time(const Time & rhs);

  RCLCPP_PUBLIC
  Time(
    const builtin_interfaces::msg::Time & time_msg,
    rcl_clock_type_t ros_time = RCL_ROS_TIME);

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
  operator+(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator-(const rclcpp::Time & rhs) const;

  RCLCPP_PUBLIC
  Time
  operator-(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  rcl_time_point_value_t
  nanoseconds() const;

  /// \return the seconds since epoch as a floating point number.
  RCLCPP_PUBLIC
  double
  seconds() const;

  RCLCPP_PUBLIC
  rcl_clock_type_t
  get_clock_type() const;

private:
  rcl_time_point_t rcl_time_;
  friend Clock;  // Allow clock to manipulate internal data
};

Time
operator+(const rclcpp::Duration & lhs, const rclcpp::Time & rhs);

}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_
