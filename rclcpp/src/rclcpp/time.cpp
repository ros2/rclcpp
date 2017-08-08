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

rcl_time_source_t
init_time_source(rcl_time_source_type_t clock = RCL_SYSTEM_TIME)
{
  rcl_time_source_t time_source;
  auto ret = rcl_time_source_init(clock, &time_source);

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not initialize time source");
  }

  return time_source;
}

rcl_time_point_t
init_time_point(rcl_time_source_t & time_source)
{
  rcl_time_point_t time_point;
  auto ret = rcl_time_point_init(&time_point, &time_source);

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
Time::now(rcl_time_source_type_t clock)
{
  // TODO(karsten1987): This impl throws explicitely on RCL_ROS_TIME
  // we have to do this, because rcl_time_source_init returns RCL_RET_OK
  // for RCL_ROS_TIME, however defaults to system time under the hood.
  // ref: https://github.com/ros2/rcl/blob/master/rcl/src/rcl/time.c#L66-L77
  // This section can be removed when rcl supports ROS_TIME correctly.
  if (clock == RCL_ROS_TIME) {
    throw std::runtime_error("RCL_ROS_TIME is currently not implemented.");
  }

  Time now(0, 0, clock);

  auto ret = rcl_time_point_get_now(&now.rcl_time_);
  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not get current time stamp");
  }

  return now;
}

Time::Time(int32_t seconds, uint32_t nanoseconds, rcl_time_source_type_t clock)
: rcl_time_source_(init_time_source(clock)),
  rcl_time_(init_time_point(rcl_time_source_))
{
  if (seconds < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(seconds));
  rcl_time_.nanoseconds += nanoseconds;
}

Time::Time(uint64_t nanoseconds, rcl_time_source_type_t clock)
: rcl_time_source_(init_time_source(clock)),
  rcl_time_(init_time_point(rcl_time_source_))
{
  rcl_time_.nanoseconds = nanoseconds;
}

Time::Time(const builtin_interfaces::msg::Time & time_msg)  // NOLINT
: rcl_time_source_(init_time_source(RCL_SYSTEM_TIME)),
  rcl_time_(init_time_point(rcl_time_source_))
{
  if (time_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(time_msg.sec));
  rcl_time_.nanoseconds += time_msg.nanosec;
}

Time::~Time()
{
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

void
Time::operator=(const builtin_interfaces::msg::Time & time_msg)
{
  if (time_msg.sec < 0) {
    throw std::runtime_error("cannot store a negative time point in rclcpp::Time");
  }

  this->rcl_time_source_ = init_time_source();
  this->rcl_time_ = init_time_point(this->rcl_time_source_);

  rcl_time_.nanoseconds = RCL_S_TO_NS(static_cast<uint64_t>(time_msg.sec));
  rcl_time_.nanoseconds += time_msg.nanosec;
}

bool
Time::operator==(const rclcpp::Time & rhs) const
{
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds == rhs.rcl_time_.nanoseconds;
}

bool
Time::operator<(const rclcpp::Time & rhs) const
{
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds < rhs.rcl_time_.nanoseconds;
}

bool
Time::operator<=(const rclcpp::Time & rhs) const
{
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds <= rhs.rcl_time_.nanoseconds;
}

bool
Time::operator>=(const rclcpp::Time & rhs) const
{
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds >= rhs.rcl_time_.nanoseconds;
}

bool
Time::operator>(const rclcpp::Time & rhs) const
{
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
    throw std::runtime_error("can't compare times with different time sources");
  }

  return rcl_time_.nanoseconds > rhs.rcl_time_.nanoseconds;
}

Time
Time::operator+(const rclcpp::Time & rhs) const
{
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
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
  if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
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
