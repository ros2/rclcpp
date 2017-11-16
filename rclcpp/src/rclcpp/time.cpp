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
: rcl_time_(rhs.rcl_time_)
{
  rcl_time_.nanoseconds = rhs.rcl_time_.nanoseconds;
}

Time::Time(
  const builtin_interfaces::msg::Time & time_msg,
  rcl_clock_type_t ros_time)
{
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
  rcl_time_ = rhs.rcl_time_;
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

Time
Time::operator+(const rclcpp::Duration & rhs) const
{
  if (this->get_clock_type() != rhs.get_clock_type()) {
    throw std::runtime_error("can't add times with different time sources");
  }

  if (rhs.nanoseconds() > 0 && (uint64_t)rhs.nanoseconds() >
    std::numeric_limits<rcl_time_point_value_t>::max() -
    (rcl_time_point_value_t)this->nanoseconds())
  {
    throw std::overflow_error("addition leads to uint64_t overflow");
  }
  return Time(this->nanoseconds() + rhs.nanoseconds(), this->get_clock_type());
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

  if (rcl_time_.nanoseconds < rhs.rcl_time_.nanoseconds) {
    rcl_time_point_value_t negative_delta = rhs.rcl_time_.nanoseconds - rcl_time_.nanoseconds;
    if (negative_delta > (uint64_t) std::numeric_limits<rcl_duration_value_t>::min()) {
      throw std::underflow_error("time subtraction leads to int64_t underflow");
    }
  }

  return Duration(rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds, rcl_time_.clock_type);
}

Time
Time::operator-(const rclcpp::Duration & rhs) const
{
  if (rcl_time_.clock_type != rhs.get_clock_type()) {
    throw std::runtime_error("can't subtract times with different time sources");
  }

  if (rhs.nanoseconds() > 0 && rcl_time_.nanoseconds >
    std::numeric_limits<rcl_time_point_value_t>::max() - (uint64_t)rhs.nanoseconds())
  {
    throw std::underflow_error("time subtraction leads to uint64_t overflow");
  }
  if (rcl_time_.nanoseconds < (uint64_t) std::numeric_limits<rcl_duration_value_t>::max() &&
    (int64_t)rcl_time_.nanoseconds < (int64_t)rhs.nanoseconds())
  {
    throw std::underflow_error("time subtraction leads to uint64_t underflow");
  }

  return Time(rcl_time_.nanoseconds - rhs.nanoseconds(), rcl_time_.clock_type);
}

uint64_t
Time::nanoseconds() const
{
  return rcl_time_.nanoseconds;
}

rcl_clock_type_t
Time::get_clock_type() const
{
  return rcl_time_.clock_type;
}

Time
operator+(const rclcpp::Duration & lhs, const rclcpp::Time & rhs)
{
  if (lhs.get_clock_type() != rhs.get_clock_type()) {
    throw std::runtime_error("can't add times with different time sources");
  }

  if (rhs.nanoseconds() >
    std::numeric_limits<rcl_time_point_value_t>::max() - (rcl_time_point_value_t)lhs.nanoseconds())
  {
    throw std::overflow_error("addition leads to uint64_t overflow");
  }
  return Time(lhs.nanoseconds() + rhs.nanoseconds(), lhs.get_clock_type());
}


}  // namespace rclcpp
