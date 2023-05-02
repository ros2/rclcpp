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

#ifndef RCLCPP_LIFECYCLE__STATE_HPP_
#define RCLCPP_LIFECYCLE__STATE_HPP_

#include <string>

#include "rcl_lifecycle/data_types.h"
#include "rclcpp_lifecycle/visibility_control.h"

#include "rcutils/allocator.h"

namespace rclcpp_lifecycle
{
/// Forward declaration of mutex helper class
class MutexMap;

/// Abstract class for the Lifecycle's states.
/**
 * There are 4 primary states: Unconfigured, Inactive, Active and Finalized.
 */
class State
{
public:
  RCLCPP_LIFECYCLE_PUBLIC
  explicit State(rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// State constructor.
  /**
   * \param[in] id of the state
   * \param[in] label of the state
   * \param[in] allocator a valid allocator used to initialized the state.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  State(
    uint8_t id,
    const std::string & label,
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  /// State constructor.
  /**
   * \param[in] rcl_lifecycle_state_handle structure with the state details
   * \param[in] allocator a valid allocator used to initialized the state.
   */
  RCLCPP_LIFECYCLE_PUBLIC
  explicit State(
    const rcl_lifecycle_state_t * rcl_lifecycle_state_handle,
    rcutils_allocator_t allocator = rcutils_get_default_allocator());

  RCLCPP_LIFECYCLE_PUBLIC
  State(const State & rhs);

  RCLCPP_LIFECYCLE_PUBLIC
  virtual ~State();

  RCLCPP_LIFECYCLE_PUBLIC
  State & operator=(const State & rhs);

  /// Return the id.
  /**
   * \return id of the state
   */
  RCLCPP_LIFECYCLE_PUBLIC
  uint8_t
  id() const;

  /// Return the label.
  /**
   * \return label of state
   */
  RCLCPP_LIFECYCLE_PUBLIC
  std::string
  label() const;

protected:
  RCLCPP_LIFECYCLE_PUBLIC
  void
  reset() noexcept;

  rcutils_allocator_t allocator_;

  bool owns_rcl_state_handle_;

  rcl_lifecycle_state_t * state_handle_;

private:
  /// Maps state handle mutexes to each instance of State.
  /**
   * \details A mutex is added to this map when each new instance of State is constructed.
   *
   * The mutex is removed when the instance of State is destroyed.
   *
   * The mutex is locked while state_handle_ is being accessed.
   *
   * This static member exists to allow implementing the fix described in ros2/rclcpp#1756
   * in Humble without breaking ABI compatibility, since adding a new static data
   * member is permitted under REP-0009.
   */
  static MutexMap state_handle_mutex_map_;
};

}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__STATE_HPP_
