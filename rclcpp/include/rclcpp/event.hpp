// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EVENT_HPP_
#define RCLCPP__EVENT_HPP_

#include <atomic>
#include <memory>

#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace event
{

class Event
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Event);

  RCLCPP_PUBLIC
  Event();

  RCLCPP_PUBLIC
  bool
  set();

  RCLCPP_PUBLIC
  bool
  check();

  RCLCPP_PUBLIC
  bool
  check_and_clear();

private:
  RCLCPP_DISABLE_COPY(Event);

  std::atomic_bool state_;
};

}  // namespace event
}  // namespace rclcpp

#endif  // RCLCPP__EVENT_HPP_
