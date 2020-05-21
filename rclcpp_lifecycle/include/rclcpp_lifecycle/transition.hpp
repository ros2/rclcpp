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

/// The Transition class abstract the Lifecycle's states.
/**
 * There are 7 transitions exposed to a supervisory process, they are: create, configure,
 * cleanup, activate, deactivate, shutdown and destroy.
 */
class Transition
{
public:
  RCLCPP_LIFECYCLE_PUBLIC
  Transition() = delete;

  /// Transition constructor.
  /**
   * \param[in] id of the transition
   * \param[in] label of the transition
   * \param[in] allocator a valid allocator used to initialized the state.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit Transition(
    uint8_t id,
    const std::string & label = "",
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// Transition constructor.
  /**
   * \param[in] id of the transition
   * \param[in] label of the transition
   * \param[in] start state of the transition
   * \param[in] goal state of the transition
   * \param[in] allocator a valid allocator used to initialized the state.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  Transition(
    uint8_t id, const std::string & label,
    State && start, State && goal,
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// Transition constructor.
  /**
   * \param[in] rcl_lifecycle_transition_handle structure with the transition details
   * \param[in] allocator a valid allocator used to initialized the state.
   */
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

  /// Return the id.
  /**
   * \return id of the state
   */
  RCLCPP_LIFECYCLE_PUBLIC
  uint8_t
  id() const;

  /// Return the label.
  /**
   * \return label of the transition
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::string
  label() const;

  /// Return the start state of the transition.
  /**
   * \return start state of the transition.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  State
  start_state() const;

  /// Return the goal state of the transition.
  /**
   * \return goal state of the transition.
   */
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
