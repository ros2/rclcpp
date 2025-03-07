// Copyright 2025 Sony Group Corporation.
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

#ifndef RCLCPP_ACTION__GENERIC_CLIENT_HPP_
#define RCLCPP_ACTION__GENERIC_CLIENT_HPP_

#include <cstddef>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "action_msgs/srv/cancel_goal.hpp"
#include "action_msgs/msg/goal_info.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include "rclcpp_action/client_base.hpp"
#include "rclcpp_action/generic_client_goal_handle.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

#include "unique_identifier_msgs/msg/uuid.hpp"

namespace rclcpp_action
{
/// Action Generic Client
/**
 * This class creates an action generic client.
 *
 * To create an instance of an action client use `rclcpp_action::create_generic_client()`.
 */
class GenericClient : public ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(GenericClient)

  using Goal = void *;  // Deserialized data pointer of goal
  using GoalRequest = void *;  // Deserialized data pointer of goal request (uuid + Goal)
  using CancelRequest = typename action_msgs::srv::CancelGoal_Request;
  using CancelResponse = typename action_msgs::srv::CancelGoal_Response;
  using WrappedResult = typename GenericClientGoalHandle::WrappedResult;
  using GoalResponseCallback = std::function<void (typename GenericClientGoalHandle::SharedPtr)>;
  using FeedbackCallback = typename GenericClientGoalHandle::FeedbackCallback;
  using ResultCallback = typename GenericClientGoalHandle::ResultCallback;
  using CancelCallback = std::function<void (CancelResponse::SharedPtr)>;

  using IntrospectionMessageMembersPtr =
    const rosidl_typesupport_introspection_cpp::MessageMembers *;

  /// Options for sending a goal.
  /**
   * This struct is used to pass parameters to the function `async_send_goal`.
   */
  struct SendGoalOptions
  {
    SendGoalOptions()
    : goal_response_callback(nullptr),
      feedback_callback(nullptr),
      result_callback(nullptr)
    {
    }

    /// Function called when the goal is accepted or rejected.
    /**
     * Takes a single argument that is a goal handle shared pointer.
     * If the goal is accepted, then the pointer points to a valid goal handle.
     * If the goal is rejected, then pointer has the value `nullptr`.
     */
    GoalResponseCallback goal_response_callback;

    /// Function called whenever feedback is received for the goal.
    FeedbackCallback feedback_callback;

    /// Function called when the result for the goal is received.
    ResultCallback result_callback;
  };

  /// Construct an action generic client.
  /**
   * This constructs an action generic client, but it will not work until it has been added to a
   * node.
   * Use `rclcpp_action::create_generic_client()` to both construct and add to a node.
   *
   * \param[in] node_base A pointer to the base interface of a node.
   * \param[in] node_graph A pointer to an interface that allows getting graph information about
   *   a node.
   * \param[in] node_logging A pointer to an interface that allows getting a node's logger.
   * \param[in] action_name The action name.
   * \param[in] typesupport_lib A pointer to type support library for the action type
   * \param[in] action_typesupport_handle the action type support handle
   * \param[in] client_options Options to pass to the underlying `rcl_action::rcl_action_client_t`.
   */
  RCLCPP_ACTION_PUBLIC
  GenericClient(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    std::shared_ptr<rcpputils::SharedLibrary> typesupport_lib,
    const rosidl_action_type_support_t * action_typesupport_handle,
    const rcl_action_client_options_t & client_options = rcl_action_client_get_default_options());

  /// Send an action goal and asynchronously get the result.
  /**
   * If the goal is accepted by an action server, the returned future is set to a `GenericClientGoalHandle::SharedPtr`.
   * If the goal is rejected by an action server, then the future is set to a `nullptr`.
   *
   * The goal handle in the future is used to monitor the status of the goal and get the final result.
   *
   * If callbacks were set in @param options, you will receive callbacks, as long as you hold a reference
   * to the shared pointer contained in the returned future, or rclcpp_action::GenericClient is destroyed. Dropping
   * the shared pointer to the goal handle will not cancel the goal. In order to cancel it, you must explicitly
   * call async_cancel_goal.
   *
   * WARNING this method has inconsistent behaviour and a memory leak bug.
   * If you set the result callback in @param options, the handle will be self referencing, and you will receive
   * callbacks even though you do not hold a reference to the shared pointer. In this case, the self reference will
   * be deleted if the result callback was received. If there is no result callback, there will be a memory leak.
   *
   * To prevent the memory leak, you may call stop_callbacks() explicit. This will delete the self reference.
   *
   * \param[in] goal The goal.
   * \param[in] goal_size  The size of goal.
   * \param[in] options Options for sending the goal request. Contains references to callbacks for
   *   the goal response (accepted/rejected), feedback, and the final result.
   * \return A future that completes when the goal has been accepted or rejected.
   *   If the goal is rejected, then the result will be a `nullptr`.
   */
  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename GenericClientGoalHandle::SharedPtr>
  async_send_goal(
    const Goal goal,
    size_t goal_size,
    const SendGoalOptions & options = SendGoalOptions());

  /// Send an action goal request and asynchronously get the result.
  /**
   * \param[in] goal_request The goal request (uuid+goal).
   * \param[in] options Options for sending the goal request. Contains references to callbacks for
   *   the goal response (accepted/rejected), feedback, and the final result.
   * \return A future that completes when the goal has been accepted or rejected.
   *   If the goal is rejected, then the result will be a `nullptr`.
   */
  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename GenericClientGoalHandle::SharedPtr>
  async_send_goal(
    const GoalRequest goal_request,
    const SendGoalOptions & options = SendGoalOptions());

  /// Asynchronously get the result for an active goal.
  /**
   * \throws exceptions::UnknownGoalHandleError If the goal unknown or already reached a terminal
   *   state, or if there was an error requesting the result.
   * \param[in] goal_handle The goal handle for which to get the result.
   * \param[in] result_callback Optional callback that is called when the result is received.
   * \return A future that is set to the goal result when the goal is finished.
   */
  RCLCPP_ACTION_PUBLIC
  std::shared_future<WrappedResult>
  async_get_result(
    typename GenericClientGoalHandle::SharedPtr goal_handle,
    ResultCallback result_callback = nullptr);

  /// Asynchronously request a goal be canceled.
  /**
   * \throws exceptions::UnknownGoalHandleError If the goal is unknown or already reached a
   *   terminal state.
   * \param[in] goal_handle The goal handle requesting to be canceled.
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_goal(
    typename GenericClientGoalHandle::SharedPtr goal_handle,
    CancelCallback cancel_callback = nullptr);

  /// Asynchronously request for all goals to be canceled.
  /**
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_all_goals(CancelCallback cancel_callback = nullptr);

  /// Stops the callbacks for the goal in a thread safe way
  /**
   * This will NOT cancel the goal, it will only stop the callbacks.
   *
   * After the call to this function, it is guaranteed that there
   * will be no more callbacks from the goal. This is not guaranteed
   * if multiple threads are involved, and the goal_handle is just
   * dropped.
   *
   * \param[in] goal_handle The goal were the callbacks shall be stopped
   */
  RCLCPP_ACTION_PUBLIC
  void
  stop_callbacks(typename GenericClientGoalHandle::SharedPtr goal_handle);

  /// Stops the callbacks for the goal in a thread safe way
  /**
   * For further information see stop_callbacks(typename GenericGoalHandle::SharedPtr goal_handle)
   */
  RCLCPP_ACTION_PUBLIC
  void
  stop_callbacks(const GoalUUID & goal_id);

  /// Asynchronously request all goals at or before a specified time be canceled.
  /**
   * \param[in] stamp The timestamp for the cancel goal request.
   * \param[in] cancel_callback Optional callback that is called when the response is received.
   *   The callback takes one parameter: a shared pointer to the CancelResponse message.
   * \return A future to a CancelResponse message that is set when the request has been
   * acknowledged by an action server.
   * See
   * <a href="https://github.com/ros2/rcl_interfaces/blob/master/action_msgs/srv/CancelGoal.srv">
   * action_msgs/CancelGoal.srv</a>.
   */
  RCLCPP_ACTION_PUBLIC
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_goals_before(
    const rclcpp::Time & stamp,
    CancelCallback cancel_callback = nullptr);

  RCLCPP_ACTION_PUBLIC
  virtual
  ~GenericClient();

private:
  /// \internal
  std::shared_ptr<void>
  create_message(IntrospectionMessageMembersPtr message_members) const;

  /// \internal
  std::shared_ptr<void>
  create_goal_response() const override
  {
    return create_message(goal_service_response_type_members_);
  }

  /// \internal
  std::shared_ptr<void>
  create_result_request() const
  {
    return create_message(result_service_request_type_members_);
  }

  /// \internal
  std::shared_ptr<void>
  create_result_response() const override
  {
    return create_message(result_service_response_type_members_);
  }

  /// \internal
  std::shared_ptr<void>
  create_cancel_response() const override
  {
    return create_message(cancel_service_response_type_members_);
  }

  /// \internal
  std::shared_ptr<void>
  create_feedback_message() const override
  {
    return create_message(feedback_type_members_);
  }

  /// \internal
  void
  handle_feedback_message(std::shared_ptr<void> message) override;

  /// \internal
  std::shared_ptr<void>
  create_status_message() const override
  {
    using GoalStatusMessage = action_msgs::msg::GoalStatusArray;
    return std::shared_ptr<void>(new GoalStatusMessage());
  }

  /// \internal
  void
  handle_status_message(std::shared_ptr<void> message) override;

  /// \internal
  void
  make_result_aware(typename GenericClientGoalHandle::SharedPtr goal_handle);

  /// \internal
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel(
    typename CancelRequest::SharedPtr cancel_request,
    CancelCallback cancel_callback = nullptr);

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  IntrospectionMessageMembersPtr goal_service_request_type_members_;
  IntrospectionMessageMembersPtr goal_service_response_type_members_;
  IntrospectionMessageMembersPtr result_service_request_type_members_;
  IntrospectionMessageMembersPtr result_service_response_type_members_;
  IntrospectionMessageMembersPtr cancel_service_response_type_members_;
  IntrospectionMessageMembersPtr feedback_type_members_;

  std::map<GoalUUID, typename GenericClientGoalHandle::WeakPtr> goal_handles_;
  std::recursive_mutex goal_handles_mutex_;
};
}  // namespace rclcpp_action
#endif  // RCLCPP_ACTION__GENERIC_CLIENT_HPP_
