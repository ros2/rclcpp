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

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

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

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(seconds));
  rcl_time_.nanoseconds += nanoseconds;
}

Time::Time(uint64_t nanoseconds, rcl_clock_type_t clock_type)
: rcl_time_(init_time_point(clock_type))
{
  rcl_time_.nanoseconds = nanoseconds;
}

Time::Time(const Time & rhs)
: rcl_time_source_(init_time_source(rhs.rcl_time_source_.type)),
  rcl_time_(init_time_point(rcl_time_source_))
{
  rcl_time_.nanoseconds = rhs.rcl_time_.nanoseconds;
}

Time::Time(const builtin_interfaces::msg::Time & time_msg)  // NOLINT
{
  rcl_clock_type_t ros_time = RCL_ROS_TIME;  // TODO(tfoote) hard coded ROS here
  rcl_time_ = init_time_point(ros_time);
  if (time_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(time_msg.sec));
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
  msg_time.sec = static_cast<std::int32_t>(RCL_NS_TO_S(rcl_time_.nanoseconds));
  msg_time.nanosec = static_cast<std::uint32_t>(rcl_time_.nanoseconds % (1000 * 1000 * 1000));
  return msg_time;
}

Time &
Time::operator=(const Time & rhs)
{
  rcl_time_source_ = init_time_source(rhs.rcl_time_source_.type);
  rcl_time_ = init_time_point(rcl_time_source_);
  rcl_time_.nanoseconds = rhs.rcl_time_.nanoseconds;
  return *this;
}

Time &
Time::operator=(const builtin_interfaces::msg::Time & time_msg)
{
  if (time_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }


  rcl_clock_type_t ros_time = RCL_ROS_TIME;
  rcl_time_ = init_time_point(ros_time);  // TODO(tfoote) hard coded ROS here

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(time_msg.sec));
  rcl_time_.nanoseconds += time_msg.nanosec;
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

Duration
Time::operator+(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't add times with different time sources");
  }

  if (rcl_time_.nanoseconds >
    std::numeric_limits<rcl_time_point_value_t>::max() - rhs.rcl_time_.nanoseconds)
  {
    throw std::overflow_error("addition leads to uint64_t overflow");
  }
  return Duration(rcl_time_.nanoseconds + rhs.rcl_time_.nanoseconds, rcl_time_.clock_type);
}

Duration
Time::operator-(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't subtract times with different time sources");
  }

  if (rcl_time_.nanoseconds >
    (uint64_t)std::numeric_limits<rcl_duration_value_t>::max() + rhs.rcl_time_.nanoseconds)
  {
    throw std::underflow_error("time subtraction leads to int64_t overflow");
  }
  // TODO(tfoote) the below check won't work for larger times
  if ((int64_t)rcl_time_.nanoseconds <
    (int64_t)rhs.rcl_time_.nanoseconds -
    (int64_t) std::abs(std::numeric_limits<rcl_duration_value_t>::min()))
  {
    throw std::underflow_error("time subtraction leads to int64_t underflow");
  }

  return Duration(rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds, rcl_time_.clock_type);
}

uint64_t
Time::nanoseconds() const
{
  return rcl_time_.nanoseconds;
}

}  // namespace rclcpp
