// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EVENT_WAITABLE_HPP_
#define RCLCPP__EXECUTORS__EVENT_WAITABLE_HPP_

#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace executors
{

/**
 * @brief This class provides a wrapper around the waitable object, that is
 * meant to be used with the EventsExecutor.
 * The waitset related methods are stubbed out as they should not be called.
 * Nodes who want to implement a custom EventWaitable, can derive from this class and override
 * the execute function.
 */
class EventWaitable : public rclcpp::Waitable
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EventWaitable)

  // Constructor
  RCLCPP_PUBLIC
  EventWaitable() = default;

  // Destructor
  RCLCPP_PUBLIC
  virtual ~EventWaitable() = default;

  // Executing an EventWaitable is a no-op.
  // Derive from this class to implement execute function.
  RCLCPP_PUBLIC
  virtual void
  execute() = 0;

  // Stub API: not used by EventsExecutor
  RCLCPP_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) final
  {
    (void)wait_set;
    throw std::runtime_error("EventWaitable can't be checked if it's ready");
    return false;
  }

  // Stub API: not used by EventsExecutor
  RCLCPP_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) final
  {
    (void)wait_set;
    throw std::runtime_error("EventWaitable can't be added to a wait_set");
    return false;
  }
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__EVENT_WAITABLE_HPP_
