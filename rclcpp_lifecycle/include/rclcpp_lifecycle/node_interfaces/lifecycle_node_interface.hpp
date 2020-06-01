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

#ifndef RCLCPP_LIFECYCLE__NODE_INTERFACES__LIFECYCLE_NODE_INTERFACE_HPP_
#define RCLCPP_LIFECYCLE__NODE_INTERFACES__LIFECYCLE_NODE_INTERFACE_HPP_

#include "lifecycle_msgs/msg/transition.hpp"

#include "rcl_lifecycle/rcl_lifecycle.h"

#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/visibility_control.h"

namespace rclcpp_lifecycle
{
namespace node_interfaces
{
/// Interface class for a managed node.
/** Virtual functions as defined in
 * http://design.ros2.org/articles/node_lifecycle.html
 *
 * If the callback function returns successfully,
 * the specified transition is completed.
 * If the callback function fails or throws an
 * uncaught exception, the on_error function is
 * called.
 * By default, all functions remain optional to overwrite
 * and return true. Except the on_error function, which
 * returns false and thus goes to shutdown/finalize state.
 */
class LifecycleNodeInterface
{
protected:
  RCLCPP_LIFECYCLE_PUBLIC
  LifecycleNodeInterface() {}

public:
  enum class CallbackReturn : uint8_t
  {
    SUCCESS = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_SUCCESS,
    FAILURE = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_FAILURE,
    ERROR = lifecycle_msgs::msg::Transition::TRANSITION_CALLBACK_ERROR
  };

  /// Callback function for configure transition
  /*
   * \return true by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn
  on_configure(const State & previous_state);

  /// Callback function for cleanup transition
  /*
   * \return true by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn
  on_cleanup(const State & previous_state);

  /// Callback function for shutdown transition
  /*
   * \return true by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn
  on_shutdown(const State & previous_state);

  /// Callback function for activate transition
  /*
   * \return true by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn
  on_activate(const State & previous_state);

  /// Callback function for deactivate transition
  /*
   * \return true by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn
  on_deactivate(const State & previous_state);

  /// Callback function for errorneous transition
  /*
   * \return false by default
   */
  RCLCPP_LIFECYCLE_PUBLIC
  virtual CallbackReturn
  on_error(const State & previous_state);

  RCLCPP_LIFECYCLE_PUBLIC
  virtual
  ~LifecycleNodeInterface() {}
};

}  // namespace node_interfaces
}  // namespace rclcpp_lifecycle
#endif  // RCLCPP_LIFECYCLE__NODE_INTERFACES__LIFECYCLE_NODE_INTERFACE_HPP_
