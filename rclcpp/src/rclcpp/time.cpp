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

#include <limits>
#include <utility>

#include "rclcpp/time.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/overflow.hpp"
#include "rclcpp/time_utils.hpp"

namespace rclcpp
{

Time::Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_time_{join_to_nano(seconds, nanoseconds), clock_type}
{
  // will throw if seconds are negative
  __positive_timepoint(seconds);
}

Time::Time(int64_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_time_{nanoseconds, clock_type} {}

Time::Time(const Time & rhs)
: rcl_time_(rhs.rcl_time_) {}

Time::Time(
  const builtin_interfaces::msg::Time & time_msg,
  rcl_clock_type_t clock_type)
: rcl_time_{join_to_nano(time_msg.sec, time_msg.nanosec), clock_type}
{
  // will throw if seconds are negative
  __positive_timepoint(time_msg.sec);
}

Time::Time(const rcl_time_point_t & time_point)
: rcl_time_(time_point) {}

Time::operator builtin_interfaces::msg::Time() const
{
  builtin_interfaces::msg::Time msg_time;
  split_from_nano(rcl_time_.nanoseconds, msg_time.sec, msg_time.nanosec);
  return msg_time;
}

Time &
Time::operator=(const Time & rhs)
{
  rcl_time_ = rhs.rcl_time_;
  return *this;
}

Time &
Time::operator=(const builtin_interfaces::msg::Time & time_msg)
{
  *this = Time(time_msg);
  return *this;
}

bool
Time::operator==(const rclcpp::Time & rhs) const
{
  // will throw if not comparable
  __comparable(rcl_time_.clock_type, rhs.rcl_time_.clock_type);
  return rcl_time_.nanoseconds == rhs.rcl_time_.nanoseconds;
}

bool
Time::operator!=(const rclcpp::Time & rhs) const
{
  return !(*this == rhs);
}

bool
Time::operator<(const rclcpp::Time & rhs) const
{
  // will throw if not comparable
  __comparable(rcl_time_.clock_type, rhs.rcl_time_.clock_type);
  return rcl_time_.nanoseconds < rhs.rcl_time_.nanoseconds;
}

bool
Time::operator<=(const rclcpp::Time & rhs) const
{
  // will throw if not comparable
  __comparable(rcl_time_.clock_type, rhs.rcl_time_.clock_type);
  return rcl_time_.nanoseconds <= rhs.rcl_time_.nanoseconds;
}

bool
Time::operator>=(const rclcpp::Time & rhs) const
{
  // will throw if not comparable
  __comparable(rcl_time_.clock_type, rhs.rcl_time_.clock_type);
  return rcl_time_.nanoseconds >= rhs.rcl_time_.nanoseconds;
}

bool
Time::operator>(const rclcpp::Time & rhs) const
{
  // will throw if not comparable
  __comparable(rcl_time_.clock_type, rhs.rcl_time_.clock_type);
  return rcl_time_.nanoseconds > rhs.rcl_time_.nanoseconds;
}

Time
Time::operator+(const rclcpp::Duration & rhs) const
{
  rcl_time_point_value_t res = 0;
  // will throw if addition over/underflows
  check_add_overflow(rcl_time_.nanoseconds, rhs.nanoseconds(), &res);
  return Time(res, this->get_clock_type());
}

Duration
Time::operator-(const rclcpp::Time & rhs) const
{
  // will throw if not comparable
  __comparable(rcl_time_.clock_type, rhs.rcl_time_.clock_type);
  rcl_time_point_value_t res = 0;
  // will throw if subtraction over/underflows
  check_sub_overflow(rcl_time_.nanoseconds, rhs.nanoseconds(), &res);
  return Duration(res);
}

Time
Time::operator-(const rclcpp::Duration & rhs) const
{
  rcl_time_point_value_t res = 0;
  // will throw if subtraction over/underflows
  check_sub_overflow(rcl_time_.nanoseconds, rhs.nanoseconds(), &res);
  return Time(res, rcl_time_.clock_type);
}

int64_t
Time::nanoseconds() const
{
  return rcl_time_.nanoseconds;
}

double
Time::seconds() const
{
  return std::chrono::duration<double>(std::chrono::nanoseconds(rcl_time_.nanoseconds)).count();
}

rcl_clock_type_t
Time::get_clock_type() const
{
  return rcl_time_.clock_type;
}

Time
operator+(const rclcpp::Duration & lhs, const rclcpp::Time & rhs)
{
  rcl_time_point_value_t res = 0;
  // will throw if addition over/underflows
  check_add_overflow(lhs.nanoseconds(), rhs.nanoseconds(), &res);
  return Time(res, rhs.get_clock_type());
}

Time
Time::max()
{
  return Time(std::numeric_limits<int32_t>::max(), 999999999);
}

}  // namespace rclcpp
