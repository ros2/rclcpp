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

#ifndef RCLCPP__DURATION_HPP_
#define RCLCPP__DURATION_HPP_

#include <chrono>

#include "builtin_interfaces/msg/duration.hpp"
#include "rcl/time.h"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
class Duration
{
public:
  RCLCPP_PUBLIC
  Duration(int32_t seconds, uint32_t nanoseconds);

  RCLCPP_PUBLIC
  explicit Duration(
    rcl_duration_value_t nanoseconds);

  RCLCPP_PUBLIC
  explicit Duration(
    std::chrono::nanoseconds nanoseconds);

  RCLCPP_PUBLIC
  Duration(
    const builtin_interfaces::msg::Duration & duration_msg);

  RCLCPP_PUBLIC
  explicit Duration(const rcl_duration_t & duration);

  RCLCPP_PUBLIC
  Duration(const Duration & rhs);

  RCLCPP_PUBLIC
  virtual ~Duration();

  RCLCPP_PUBLIC
  operator builtin_interfaces::msg::Duration() const;

  RCLCPP_PUBLIC
  Duration &
  operator=(const Duration & rhs);

  RCLCPP_PUBLIC
  Duration &
  operator=(const builtin_interfaces::msg::Duration & Duration_msg);

  RCLCPP_PUBLIC
  bool
  operator==(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator<=(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>=(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  bool
  operator>(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator+(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  Duration
  operator-(const rclcpp::Duration & rhs) const;

  RCLCPP_PUBLIC
  static Duration
  max();

  RCLCPP_PUBLIC
  Duration
  operator*(double scale) const;

  RCLCPP_PUBLIC
  rcl_duration_value_t
  nanoseconds() const;

  /// \return the duration in seconds as a floating point number.
  /// \warning Depending on sizeof(double) there could be significant precision loss.
  /// When an exact time is required use nanoseconds() instead.
  RCLCPP_PUBLIC
  double
  seconds() const;

private:
  rcl_duration_t rcl_duration_;
};

}  // namespace rclcpp

#endif  // RCLCPP__DURATION_HPP_
