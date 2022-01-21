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
#include <string>
#include <utility>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

#include "rclcpp/utilities.hpp"

namespace
{

rcl_time_point_t
init_time_point(rcl_clock_type_t & clock_type)
{
  rcl_time_point_t time_point;
  time_point.clock_type = clock_type;

  return time_point;
}

}  // namespace

namespace rclcpp
{

Time::Time(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_time_(init_time_point(clock_type))
{
  if (seconds < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(seconds));
  rcl_time_.nanoseconds += nanoseconds;
}

Time::Time(int64_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_time_(init_time_point(clock_type))
{
  rcl_time_.nanoseconds = nanoseconds;
}

Time::Time(const Time & rhs) = default;

Time::Time(
  const builtin_interfaces::msg::Time & time_msg,
  rcl_clock_type_t clock_type)
: rcl_time_(init_time_point(clock_type))
{
  if (time_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(time_msg.sec));
  rcl_time_.nanoseconds += time_msg.nanosec;
}

Time::Time(const rcl_time_point_t & time_point)
: rcl_time_(time_point)
{
  // noop
}

Time::~Time()
{
}

Time::operator builtin_interfaces::msg::Time() const
{
  builtin_interfaces::msg::Time msg_time;
  constexpr rcl_time_point_value_t kRemainder = RCL_S_TO_NS(1);
  const auto result = std::div(rcl_time_.nanoseconds, kRemainder);
  if (result.rem >= 0) {
    msg_time.sec = static_cast<std::int32_t>(result.quot);
    msg_time.nanosec = static_cast<std::uint32_t>(result.rem);
  } else {
    msg_time.sec = static_cast<std::int32_t>(result.quot - 1);
    msg_time.nanosec = static_cast<std::uint32_t>(kRemainder + result.rem);
  }
  return msg_time;
}

Time &
Time::operator=(const Time & rhs) = default;

Time &
Time::operator=(const builtin_interfaces::msg::Time & time_msg)
{
  *this = Time(time_msg);
  return *this;
}

bool
Time::operator==(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

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
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds < rhs.rcl_time_.nanoseconds;
}

bool
Time::operator<=(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds <= rhs.rcl_time_.nanoseconds;
}

bool
Time::operator>=(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds >= rhs.rcl_time_.nanoseconds;
}

bool
Time::operator>(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds > rhs.rcl_time_.nanoseconds;
}

Time
Time::operator+(const rclcpp::Duration & rhs) const
{
  if (rclcpp::add_will_overflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (rclcpp::add_will_underflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }
  return Time(this->nanoseconds() + rhs.nanoseconds(), this->get_clock_type());
}

Duration
Time::operator-(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error(
            std::string("can't subtract times with different time sources [") +
            std::to_string(rcl_time_.clock_type) + " != " +
            std::to_string(rhs.rcl_time_.clock_type) + "]");
  }

  if (rclcpp::sub_will_overflow(rcl_time_.nanoseconds, rhs.rcl_time_.nanoseconds)) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }

  if (rclcpp::sub_will_underflow(rcl_time_.nanoseconds, rhs.rcl_time_.nanoseconds)) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  return Duration::from_nanoseconds(rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds);
}

Time
Time::operator-(const rclcpp::Duration & rhs) const
{
  if (rclcpp::sub_will_overflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }
  if (rclcpp::sub_will_underflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  return Time(rcl_time_.nanoseconds - rhs.nanoseconds(), rcl_time_.clock_type);
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
  if (rclcpp::add_will_overflow(rhs.nanoseconds(), lhs.nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (rclcpp::add_will_underflow(rhs.nanoseconds(), lhs.nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }
  return Time(lhs.nanoseconds() + rhs.nanoseconds(), rhs.get_clock_type());
}

Time &
Time::operator+=(const rclcpp::Duration & rhs)
{
  if (rclcpp::add_will_overflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::overflow_error("addition leads to int64_t overflow");
  }
  if (rclcpp::add_will_underflow(rhs.nanoseconds(), this->nanoseconds())) {
    throw std::underflow_error("addition leads to int64_t underflow");
  }

  rcl_time_.nanoseconds += rhs.nanoseconds();

  return *this;
}

Time &
Time::operator-=(const rclcpp::Duration & rhs)
{
  if (rclcpp::sub_will_overflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::overflow_error("time subtraction leads to int64_t overflow");
  }
  if (rclcpp::sub_will_underflow(rcl_time_.nanoseconds, rhs.nanoseconds())) {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  rcl_time_.nanoseconds -= rhs.nanoseconds();

  return *this;
}

Time
Time::max()
{
  return Time(std::numeric_limits<int32_t>::max(), 999999999);
}


}  // namespace rclcpp
