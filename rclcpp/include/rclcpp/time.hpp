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

#ifndef RCLCPP__TIME_HPP_
#define RCLCPP__TIME_HPP_

#include <utility>

#include "builtin_interfaces/msg/time.hpp"

#include "rcl/time.h"

#include "rclcpp/exceptions.hpp"

#include "rcutils/logging_macros.h"

namespace rclcpp
{

namespace
{
template<rcl_time_source_type_t ClockT = RCL_SYSTEM_TIME>
rcl_time_point_t
get_empty_time_point()
{
  rcl_time_source_t time_source;
  auto ret = rcl_time_source_init(ClockT, &time_source);

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not initialize time source: ");
  }
  rcl_time_point_t time_point;
  ret = rcl_time_point_init(&time_point, &time_source);

  if (ret != RCL_RET_OK) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "could not initialize time point: ");
  }

  return time_point;
}
}  // namespace

class Time
{
public:
  template<rcl_time_source_type_t ClockT = RCL_SYSTEM_TIME>
  static Time
  now()
  {
    // TODO(karsten1987): This impl throws explicitely on RCL_ROS_TIME
    // we have to do this, because rcl_time_source_init returns RCL_RET_OK
    // for RCL_ROS_TIME, however defaults to system time under the hood.
    // ref: https://github.com/ros2/rcl/blob/master/rcl/src/rcl/time.c#L66-L77
    // This section can be removed when rcl supports ROS_TIME correctly.
    if (ClockT == RCL_ROS_TIME) {
      throw std::runtime_error("RCL_ROS_TIME is currently not implemented.");
    }

    auto time_point = get_empty_time_point<ClockT>();

    auto ret = rcl_time_point_get_now(&time_point);
    if (ret != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "could not get current time stamp: ");
    }

    return Time(std::move(time_point));
  }

  template<rcl_time_source_type_t ClockT = RCL_SYSTEM_TIME>
  Time(int32_t seconds, uint32_t nanoseconds)
  : rcl_time_(get_empty_time_point())
  {
    uint64_t ns = RCL_S_TO_NS(seconds);
    ns += nanoseconds;
    rcl_time_.nanoseconds = ns;
  }

  template<rcl_time_source_type_t ClockT = RCL_SYSTEM_TIME>
  explicit Time(uint64_t nanoseconds)
  : rcl_time_(get_empty_time_point())
  {
    rcl_time_.nanoseconds = nanoseconds;
  }

  Time(const builtin_interfaces::msg::Time & time_msg)  // NOLINT
  : rcl_time_(get_empty_time_point())
  {
    rcl_time_.nanoseconds = RCL_S_TO_NS(time_msg.sec);
    rcl_time_.nanoseconds += time_msg.nanosec;
  }

  operator builtin_interfaces::msg::Time() const
  {
    builtin_interfaces::msg::Time msg_time;
    msg_time.sec = static_cast<std::int32_t>(RCL_NS_TO_S(rcl_time_.nanoseconds));
    msg_time.nanosec = static_cast<std::uint32_t>(rcl_time_.nanoseconds % (1000 * 1000 * 1000));
    return msg_time;
  }

  void
  operator=(const builtin_interfaces::msg::Time & time_msg)
  {
    auto time_point = get_empty_time_point();
    time_point.nanoseconds = RCL_S_TO_NS(time_msg.sec);
    time_point.nanoseconds += time_msg.nanosec;

    this->rcl_time_ = time_point;
  }

  uint64_t
  nanoseconds() const
  {
    return rcl_time_.nanoseconds;
  }

  bool
  operator==(const rclcpp::Time & rhs) const
  {
    return rcl_time_.nanoseconds == rhs.rcl_time_.nanoseconds;
  }

  bool
  operator<(const rclcpp::Time & rhs) const
  {
    return rcl_time_.nanoseconds < rhs.rcl_time_.nanoseconds;
  }

  bool
  operator<=(const rclcpp::Time & rhs) const
  {
    return rcl_time_.nanoseconds <= rhs.rcl_time_.nanoseconds;
  }

  bool
  operator>=(const rclcpp::Time & rhs) const
  {
    return rcl_time_.nanoseconds >= rhs.rcl_time_.nanoseconds;
  }

  bool
  operator>(const rclcpp::Time & rhs) const
  {
    return rcl_time_.nanoseconds > rhs.rcl_time_.nanoseconds;
  }

  Time
  operator+(const rclcpp::Time & rhs) const
  {
    if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
      throw std::runtime_error("can't add times with different time sources");
    }

    return Time(rcl_time_.nanoseconds + rhs.rcl_time_.nanoseconds);
  }

  Time
  operator-(const rclcpp::Time & rhs) const
  {
    if (rcl_time_.time_source->type != rhs.rcl_time_.time_source->type) {
      throw std::runtime_error("can't add times with different time sources");
    }

    return Time(rcl_time_.nanoseconds - rhs.rcl_time_.nanoseconds);
  }

private:
  rcl_time_point_t rcl_time_;

  explicit Time(rcl_time_point_t && rcl_time)
  : rcl_time_(std::forward<decltype(rcl_time)>(rcl_time))
  {}

public:
  virtual ~Time()
  {
    if (rcl_time_point_fini(&rcl_time_) != RCL_RET_OK) {
      RCUTILS_LOG_FATAL("failed to reclaim rcl_time_point_t in destructor of rclcpp::Time")
    }
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_
