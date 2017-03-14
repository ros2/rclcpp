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

#include <chrono>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace time
{

template<rcl_time_source_type_t ClockT = RCL_SYSTEM_TIME>
builtin_interfaces::msg::Time now()
{
  builtin_interfaces::msg::Time now;
  now.sec = 0;
  now.nanosec = 0;
  rcl_time_point_value_t rcl_now = 0;
  rcl_ret_t ret = RCL_RET_ERROR;
  if (ClockT == RCL_ROS_TIME) {
    // ret = rcl_ros_time_now(&rcl_now);
    fprintf(stderr, "RCL_ROS_TIME is currently not implemented.\n");
    ret = false;
  } else if (ClockT == RCL_ROS_TIME) {
    ret = rcl_system_time_now(&rcl_now);
  } else if (ClockT == RCL_STEADY_TIME) {
    ret = rcl_steady_time_now(&rcl_now);
  }
  if (ret != RCL_RET_OK) {
    fprintf(stderr, "Could not get current time: %s\n", rcl_get_error_string_safe());
  }
  now.sec = RCL_NS_TO_S(rcl_now);
  now.nanosec = rcl_now % (1000 * 1000 * 1000);
  return now;
}

}  // namespace time
}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_
