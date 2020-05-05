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
class RCLCPP_PUBLIC Duration
{
public:
  /// Duration constructor.
  /**
   * Initializes the time values for seconds and nanoseconds individually.
   * Large values for nsecs are wrapped automatically with the remainder added to secs.
   * Both inputs must be integers.
   * Seconds can be negative.
   *
   * \param seconds time in seconds
   * \param nanoseconds time in nanoseconds
   */
  Duration(int32_t seconds, uint32_t nanoseconds);

  // This constructor matches any numeric value - ints or floats.
  explicit Duration(rcl_duration_value_t nanoseconds);

  // This constructor matches std::chrono::nanoseconds.
  explicit Duration(std::chrono::nanoseconds nanoseconds);

  // This constructor matches any std::chrono value other than nanoseconds
  // intentionally not using explicit to create a conversion constructor
  template<class Rep, class Period>
  // cppcheck-suppress noExplicitConstructor
  Duration(const std::chrono::duration<Rep, Period> & duration)  // NOLINT(runtime/explicit)
  : Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(duration))
  {}

  // cppcheck-suppress noExplicitConstructor
  Duration(const builtin_interfaces::msg::Duration & duration_msg);  // NOLINT(runtime/explicit)

  /// Time constructor
  /**
   * \param duration rcl_duration_t structure to copy.
   */
  explicit Duration(const rcl_duration_t & duration);

  Duration(const Duration & rhs);

  virtual ~Duration() = default;

  operator builtin_interfaces::msg::Duration() const;

  // cppcheck-suppress operatorEq // this is a false positive from cppcheck
  Duration &
  operator=(const Duration & rhs);

  Duration &
  operator=(const builtin_interfaces::msg::Duration & Duration_msg);

  bool
  operator==(const rclcpp::Duration & rhs) const;

  bool
  operator<(const rclcpp::Duration & rhs) const;

  bool
  operator<=(const rclcpp::Duration & rhs) const;

  bool
  operator>=(const rclcpp::Duration & rhs) const;

  bool
  operator>(const rclcpp::Duration & rhs) const;

  Duration
  operator+(const rclcpp::Duration & rhs) const;

  Duration
  operator-(const rclcpp::Duration & rhs) const;

  /// Get the maximum representable value.
  /**
   * \return the maximum representable value
   */
  static
  Duration
  max();

  Duration
  operator*(double scale) const;

  /// Get duration in nanosecods
  /**
   * \return the duration in nanoseconds as a rcl_duration_value_t.
   */
  rcl_duration_value_t
  nanoseconds() const;

  /// Get duration in seconds
  /**
   * \warning Depending on sizeof(double) there could be significant precision loss.
   *   When an exact time is required use nanoseconds() instead.
   * \return the duration in seconds as a floating point number.
   */
  double
  seconds() const;

  /// Create a duration object from a floating point number representing seconds
  static Duration
  from_seconds(double seconds);

  /// Convert Duration into a std::chrono::Duration.
  template<class DurationT>
  DurationT
  to_chrono() const
  {
    return std::chrono::duration_cast<DurationT>(std::chrono::nanoseconds(this->nanoseconds()));
  }

  /// Convert Duration into rmw_time_t.
  rmw_time_t
  to_rmw_time() const;

private:
  rcl_duration_t rcl_duration_;
};

}  // namespace rclcpp

#endif  // RCLCPP__DURATION_HPP_
