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

#include <utility>
#include <limits>

#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

#include "builtin_interfaces/msg/duration.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace
{

rcl_duration_t
init_duration(rcl_clock_type_t & clock_type)
{
  rcl_duration_t duration;
  duration.clock_type = clock_type;

  return duration;
}

}  // namespace


namespace rclcpp
{

Duration::Duration(int32_t seconds, uint32_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_duration_(init_duration(clock_type))
{
  rcl_duration_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(seconds));
  rcl_duration_.nanoseconds += nanoseconds;
}

Duration::Duration(int64_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_duration_(init_duration(clock_type))
{
  rcl_duration_.nanoseconds = nanoseconds;
}

Duration::Duration(const Duration & rhs)
{
  rcl_duration_.nanoseconds = rhs.rcl_duration_.nanoseconds;
  rcl_duration_.clock_type = rhs.rcl_duration_.clock_type;
}

Duration::Duration(const builtin_interfaces::msg::Duration & duration_msg)  // NOLINT
{
  rcl_clock_type_t ros_duration = RCL_ROS_TIME;  // TODO(tfoote) hard coded ROS here
  rcl_duration_ = init_duration(ros_duration);
  if (duration_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative duration point in rclcpp::Duration");
  }

  rcl_duration_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(duration_msg.sec));
  rcl_duration_.nanoseconds += duration_msg.nanosec;
}

Duration::Duration(const rcl_duration_t & duration)
: rcl_duration_(duration)
{
  // noop
}

Duration::~Duration()
{
}

Duration::operator builtin_interfaces::msg::Duration() const
{
  builtin_interfaces::msg::Duration msg_duration;
  msg_duration.sec = static_cast<std::int32_t>(RCL_NS_TO_S(rcl_duration_.nanoseconds));
  msg_duration.nanosec =
    static_cast<std::uint32_t>(rcl_duration_.nanoseconds % (1000 * 1000 * 1000));
  return msg_duration;
}

Duration &
Duration::operator=(const Duration & rhs)
{
  rcl_duration_.nanoseconds = rhs.rcl_duration_.nanoseconds;
  rcl_duration_.clock_type = rhs.rcl_duration_.clock_type;
  return *this;
}

Duration &
Duration::operator=(const builtin_interfaces::msg::Duration & duration_msg)
{
  if (duration_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative duration point in rclcpp::Duration");
  }


  rcl_clock_type_t ros_duration = RCL_ROS_TIME;
  rcl_duration_ = init_duration(ros_duration);  // TODO(tfoote) hard coded ROS here

  rcl_duration_.nanoseconds = RCL_S_TO_NS(static_cast<int64_t>(duration_msg.sec));
  rcl_duration_.nanoseconds += duration_msg.nanosec;
  return *this;
}

bool
Duration::operator==(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't compare durations with different duration sources");
  }

  return rcl_duration_.nanoseconds == rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator<(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't compare durations with different duration sources");
  }

  return rcl_duration_.nanoseconds < rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator<=(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't compare durations with different duration sources");
  }

  return rcl_duration_.nanoseconds <= rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator>=(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't compare durations with different duration sources");
  }

  return rcl_duration_.nanoseconds >= rhs.rcl_duration_.nanoseconds;
}

bool
Duration::operator>(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't compare durations with different duration sources");
  }

  return rcl_duration_.nanoseconds > rhs.rcl_duration_.nanoseconds;
}

void
bounds_check_duration_sum(int64_t lhsns, int64_t rhsns, uint64_t max)
{
  auto abs_lhs = (uint64_t)std::abs(lhsns);
  auto abs_rhs = (uint64_t)std::abs(rhsns);

  if (lhsns > 0 && rhsns > 0) {
    if (abs_lhs + abs_rhs > (uint64_t) max) {
      throw std::overflow_error("addition leads to int64_t overflow");
    }
  } else if (lhsns < 0 && rhsns < 0) {
    if (abs_lhs + abs_rhs > (uint64_t) max) {
      throw std::underflow_error("addition leads to int64_t underflow");
    }
  }
}

Duration
Duration::operator+(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't add durations with different duration sources");
  }

  bounds_check_duration_sum(
    this->rcl_duration_.nanoseconds,
    rhs.rcl_duration_.nanoseconds,
    std::numeric_limits<rcl_duration_value_t>::max());
  return Duration(
    rcl_duration_.nanoseconds + rhs.rcl_duration_.nanoseconds,
    rcl_duration_.clock_type);
}

void
bounds_check_duration_difference(int64_t lhsns, int64_t rhsns, uint64_t max)
{
  auto abs_lhs = (uint64_t)std::abs(lhsns);
  auto abs_rhs = (uint64_t)std::abs(rhsns);

  if (lhsns > 0 && rhsns < 0) {
    if (abs_lhs + abs_rhs > (uint64_t) max) {
      throw std::overflow_error("duration subtraction leads to int64_t overflow");
    }
  } else if (lhsns < 0 && rhsns > 0) {
    if (abs_lhs + abs_rhs > (uint64_t) max) {
      throw std::underflow_error("duration subtraction leads to int64_t underflow");
    }
  }
}

Duration
Duration::operator-(const rclcpp::Duration & rhs) const
{
  if (rcl_duration_.clock_type != rhs.rcl_duration_.clock_type) {
    throw std::runtime_error("can't subtract durations with different duration sources");
  }

  bounds_check_duration_difference(
    this->rcl_duration_.nanoseconds,
    rhs.rcl_duration_.nanoseconds,
    std::numeric_limits<rcl_duration_value_t>::max());

  return Duration(
    rcl_duration_.nanoseconds - rhs.rcl_duration_.nanoseconds,
    rcl_duration_.clock_type);
}

rcl_duration_value_t
Duration::nanoseconds() const
{
  return rcl_duration_.nanoseconds;
}

rcl_clock_type_t
Duration::get_clock_type() const
{
  return rcl_duration_.clock_type;
}

}  // namespace rclcpp
