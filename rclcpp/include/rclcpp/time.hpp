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

template<class ClockT = std::chrono::high_resolution_clock>
builtin_interfaces::msg::Time now()
{
  builtin_interfaces::msg::Time now;
  now.sec = 0;
  now.nanosec = 0;
  auto now_nanosec = ClockT::now().time_since_epoch();
  if (now_nanosec > std::chrono::nanoseconds(0)) {
    now.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(
      RCL_NS_TO_S(now_nanosec.count()));
    now.nanosec = now_nanosec.count() % (1000 * 1000 * 1000);
  }
  return now;
}

}  // namespace time
}  // namespace rclcpp

#endif  // RCLCPP__TIME_HPP_
