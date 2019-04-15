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

#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#include "rclcpp/duration.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/time_utils.hpp"
#include "rclcpp/utilities.hpp"

namespace rclcpp
{

Duration::Duration(int32_t seconds, uint32_t nanoseconds)
: rcl_duration_{join_to_nano(seconds, nanoseconds)} {}

Duration::Duration(int64_t nanoseconds)
: rcl_duration_{nanoseconds} {}

Duration::Duration(std::chrono::nanoseconds nanoseconds)
: rcl_duration_{nanoseconds.count()} {}

Duration::Duration(const Duration & rhs)
: rcl_duration_(rhs.rcl_duration_) {}

Duration::Duration(const builtin_interfaces::msg::Duration & duration_msg)
: rcl_duration_{join_to_nano(duration_msg.sec, duration_msg.nanosec)} {}

Duration::Duration(const rcl_duration_t & duration)
: rcl_duration_(duration) {}

Duration::~Duration()
{
}

Duration::operator builtin_interfaces::msg::Duration() const
{
  builtin_interfaces::msg::Duration msg_duration;
  split_from_nano(rcl_duration_.nanoseconds, msg_duration.sec, msg_duration.nanosec);
  return msg_duration;
}

Duration &
Duration::operator=(const Duration & rhs)
{
  rcl_duration_.nanoseconds = rhs.rcl_duration_.nanoseconds;
  return *this;
}

Duration &
Duration::operator=(const builtin_interfaces::msg::Duration & duration_msg)
{
  rcl_duration_.nanoseconds = join_to_nano(duration_msg.sec, duration_msg.nanosec);
  return *this;
}

bool
Duration::operator==(const rclcpp::Duration & rhs) const
{
  return rcl_duration_.nanoseconds == rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator<(const rclcpp::Duration & rhs) const
{
  return rcl_duration_.nanoseconds < rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator<=(const rclcpp::Duration & rhs) const
{
  return rcl_duration_.nanoseconds <= rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator>=(const rclcpp::Duration & rhs) const
{
  return rcl_duration_.nanoseconds >= rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator>(const rclcpp::Duration & rhs) const
{
  return rcl_duration_.nanoseconds > rhs.rcl_duration_.nanoseconds;
}

Duration
Duration::operator+(const rclcpp::Duration & rhs) const
{
  // will throw if addition over/under_flows
  check_add(rcl_duration_.nanoseconds, rhs.nanoseconds());
  return Duration(rcl_duration_.nanoseconds + rhs.rcl_duration_.nanoseconds);
}

Duration
Duration::operator-(const rclcpp::Duration & rhs) const
{
  // will throw if subtraction over/under_flows
  check_sub(rcl_duration_.nanoseconds, rhs.nanoseconds());
  return Duration(rcl_duration_.nanoseconds - rhs.rcl_duration_.nanoseconds);
}

Duration
Duration::operator*(double scale) const
{
  if (!std::isfinite(scale)) {
    throw std::runtime_error("abnormal scale in rclcpp::Duration");
  }
  bounds_check_duration_scale(
    this->rcl_duration_.nanoseconds,
    scale,
    std::numeric_limits<rcl_duration_value_t>::max());
  return Duration(static_cast<rcl_duration_value_t>(rcl_duration_.nanoseconds * scale));
}

rcl_duration_value_t
Duration::nanoseconds() const
{
  return rcl_duration_.nanoseconds;
}

Duration
Duration::max()
{
  return Duration(std::numeric_limits<int32_t>::max(), 999999999);
}

double
Duration::seconds() const
{
  return std::chrono::duration<double>(std::chrono::nanoseconds(rcl_duration_.nanoseconds)).count();
}

}  // namespace rclcpp
