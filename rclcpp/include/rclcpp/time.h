// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__TIME_H_
#define RCLCPP__TIME_H_

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>

namespace rclcpp
{

struct Time
{
  static builtin_interfaces::msg::Time now()
  {
    builtin_interfaces::msg::Time time;
    std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    if (now > std::chrono::nanoseconds(0)) {
      time.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
      time.nanosec = now.count() % 1000000000;
    }
    return time;
  }
};

}  // namespace rclcpp

#endif  // RCLCPP__TIME_H_
