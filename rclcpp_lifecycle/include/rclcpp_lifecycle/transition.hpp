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

#ifndef RCLCPP_LIFECYCLE__TRANSITION_HPP_
#define RCLCPP_LIFECYCLE__TRANSITION_HPP_

#include <string>

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

#include "rcutils/allocator.h"

// forward declare rcl_transition_t
typedef struct rcl_lifecycle_transition_t rcl_lifecycle_transition_t;

namespace rclcpp_lifecycle
{

class Transition
{
public:
  RCLCPP_LIFECYCLE_PUBLIC
  Transition() = delete;

  RCLCPP_LIFECYCLE_PUBLIC
  explicit Transition(
    uint8_t id,
    const std::string & label = "",
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  RCLCPP_LIFECYCLE_PUBLIC
  Transition(
    uint8_t id, const std::string & label,
    State && start, State && goal,
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  RCLCPP_LIFECYCLE_PUBLIC
  explicit Transition(
    const rcl_lifecycle_transition_t * rcl_lifecycle_transition_handle,
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  RCLCPP_LIFECYCLE_PUBLIC
  Transition(const Transition & rhs);

  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~Transition();

  RCLCPP_LIFECYCLE_PUBLIC
  Transition & operator=(const Transition & rhs);

  RCLCPP_LIFECYCLE_PUBLIC
  uint8_t
  id() const;

  RCLCPP_LIFECYCLE_PUBLIC
  std::string
  label() const;

  RCLCPP_LIFECYCLE_PUBLIC
  State
  start_state() const;

  RCLCPP_LIFECYCLE_PUBLIC
  State
  goal_state() const;

protected:
  RCLCPP_LIFECYCLE_PUBLIC
  void
  reset();

  rcutils_allocator_t allocator_;

  bool owns_rcl_transition_handle_;

  rcl_lifecycle_transition_t * transition_handle_;
};

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__TRANSITION_HPP_
