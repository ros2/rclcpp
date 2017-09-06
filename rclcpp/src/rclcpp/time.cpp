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

#include "rclcpp/time.hpp"

#include <utility>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace
{

rcl_clock_t
init_clock(rcl_clock_type_t clock_type = RCL_SYSTEM_TIME)
{
  rcl_clock_t clock;
  auto ret = rcl_clock_init(clock_type, &clock);

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not initialize time source");
  }

  return clock;
}

rcl_time_point_t
init_time_point(rcl_clock_type_t & clock)
{
  rcl_time_point_t time_point;
  auto ret = rcl_time_point_init(&time_point, &clock);

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not initialize time point");
  }

  return time_point;
}

}  // namespace

namespace rclcpp
{

Time
Clock::now()
{
  Time now(0, 0, rcl_clock_.type);

  auto ret = rcl_time_point_get_now(&rcl_clock_, &now.rcl_time_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not get current time stamp");
  }

  return now;
}

Clock::Clock(rcl_clock_type_t clock_type)
{
  auto ret = rcl_clock_init(clock_type, &rcl_clock_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not get current time stamp");
  }
}

Clock::~Clock()
{
  auto ret = rcl_clock_fini(&rcl_clock_);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("Failed to fini rcl clock.");
  }
}

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
  if (rcl_time_source_fini(&rcl_time_source_) != RCL_RET_OK) {
    RCUTILS_LOG_FATAL("failed to reclaim rcl_time_source_t in destructor of rclcpp::Time")
  }
  if (rcl_time_point_fini(&rcl_time_) != RCL_RET_OK) {
    RCUTILS_LOG_FATAL("failed to reclaim rcl_time_point_t in destructor of rclcpp::Time")
  }
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

Time
Time::operator+(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't add times with different time sources");
  }

  auto ns = rcl_time_.nanoseconds + rhs.rcl_time_.nanoseconds;
  if (ns < rcl_time_.nanoseconds) {
    throw std::overflow_error("addition leads to uint64_t overflow");
  }

  return Time(rcl_time_.nanoseconds + rhs.rcl_time_.nanoseconds);
}

Time
Time::operator-(const rclcpp::Time & rhs) const
{
  if (rcl_time_.clock_type != rhs.rcl_time_.clock_type) {
    throw std::runtime_error("can't add times with different time sources");
  }

  auto ns = rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds;
  if (ns > rcl_time_.nanoseconds) {
    throw std::underflow_error("subtraction leads to uint64_t underflow");
  }

  return Time(rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds);
}

uint64_t
Time::nanoseconds() const
{
  return rcl_time_.nanoseconds;
}

}  // namespace rclcpp
