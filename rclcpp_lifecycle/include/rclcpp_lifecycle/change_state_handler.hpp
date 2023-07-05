// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_HPP_
#define RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_HPP_

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace rclcpp_lifecycle
{
/// The object passed to asynchronous change_state user transition functions
class ChangeStateHandler
{
public:
  /// Continues the change state process handling proper callback order
  /** Used within the user defined transition callback to continue the change state process
   *  similar to a service call response
   *  Note this only allows sending a single response callback per object
   *  and will not send further responses if called mutiple times on the object
   * \param[in] cb_return_code result of user defined transition callback
   * \return true if the response was successfully sent
   */
  virtual bool send_callback_resp(
    node_interfaces::LifecycleNodeInterface::CallbackReturn cb_return_code) = 0;

  /// Updates the state machine based on the handling of a cancelled transition
  /**
   * \param[in] success true if the transition cancel request was successfully handled
   * \return true if the response was successfully sent to the state handler
  */
  virtual bool handle_canceled(bool success) = 0;

  /// Check to see if a send_callback_resp has been cancelled
  /**
   * @return true if response has been cancelled
   */
  virtual bool is_canceling() const = 0;

  // Check to see if the response has been sent
  /**
   * @return true if response has not been sent
   */
  virtual bool is_executing() const = 0;

  virtual ~ChangeStateHandler() = default;
};
}  // namespace rclcpp_lifecycle

#endif  // RCLCPP_LIFECYCLE__CHANGE_STATE_HANDLER_HPP_
