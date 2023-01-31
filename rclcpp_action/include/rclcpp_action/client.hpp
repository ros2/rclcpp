// Copyright 2018 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__CLIENT_HPP_
#define RCLCPP_ACTION__CLIENT_HPP_

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "rcl/event_callback.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_logging_interface.hpp"
#include "rclcpp/node_interfaces/node_graph_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/waitable.hpp"

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"

#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/exceptions.hpp"
#include "rclcpp_action/types.hpp"
#include "rclcpp_action/visibility_control.hpp"

#include "rcpputils/mutex.hpp"

namespace rclcpp_action
{
// Forward declaration
class ClientBaseImpl;

/// Base Action Client implementation
/// \internal
/**
 * This class should not be used directly by users wanting to create an aciton client.
 * Instead users should use `rclcpp_action::Client<>`.
 *
 * Internally, this class is responsible for interfacing with the `rcl_action` API.
 */
class ClientBase : public rclcpp::Waitable
{
public:
  RCLCPP_ACTION_PUBLIC
  virtual ~ClientBase();

  /// Enum to identify entities belonging to the action client
  enum class EntityType : std::size_t
  {
    GoalClient,
    ResultClient,
    CancelClient,
    FeedbackSubscription,
    StatusSubscription,
  };

  /// Return true if there is an action server that is ready to take goal requests.
  RCLCPP_ACTION_PUBLIC
  bool
  action_server_is_ready() const;

  /// Wait for action_server_is_ready() to become true, or until the given timeout is reached.
  template<typename RepT = int64_t, typename RatioT = std::milli>
  bool
  wait_for_action_server(
    std::chrono::duration<RepT, RatioT> timeout = std::chrono::duration<RepT, RatioT>(-1))
  {
    return wait_for_action_server_nanoseconds(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)
    );
  }

  // -------------
  // Waitables API

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_subscriptions() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_timers() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_clients() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_services() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void>
  take_data() override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  std::shared_ptr<void>
  take_data_by_entity_id(size_t id) override;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  void
  execute(std::shared_ptr<void> & data) override;

  /// \internal
  /// Set a callback to be called when action client entities have an event
  /**
   * The callback receives a size_t which is the number of messages received
   * since the last time this callback was called.
   * Normally this is 1, but can be > 1 if messages were received before any
   * callback was set.
   *
   * The callback also receives an int identifier argument, which identifies
   * the action client entity which is ready.
   * This implies that the provided callback can use the identifier to behave
   * differently depending on which entity triggered the waitable to become ready.
   *
   * Calling it again will clear any previously set callback.
   *
   * An exception will be thrown if the callback is not callable.
   *
   * This function is thread-safe.
   *
   * If you want more information available in the callback, like the subscription
   * or other information, you may use a lambda with captures or std::bind.
   *
   * \param[in] callback functor to be called when a new message is received.
   */
  RCLCPP_ACTION_PUBLIC
  void
  set_on_ready_callback(std::function<void(size_t, int)> callback) override;

  /// Unset the callback registered for new events, if any.
  RCLCPP_ACTION_PUBLIC
  void
  clear_on_ready_callback() override;

  // End Waitables API
  // -----------------

protected:
  RCLCPP_ACTION_PUBLIC
  ClientBase(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rosidl_action_type_support_t * type_support,
    const rcl_action_client_options_t & options);

  /// Wait for action_server_is_ready() to become true, or until the given timeout is reached.
  RCLCPP_ACTION_PUBLIC
  bool
  wait_for_action_server_nanoseconds(std::chrono::nanoseconds timeout);

  // -----------------------------------------------------
  // API for communication between ClientBase and Client<>
  using ResponseCallback = std::function<void (std::shared_ptr<void> response)>;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  rclcpp::Logger get_logger();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  GoalUUID
  generate_goal_id();

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_goal_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_result_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  send_cancel_request(
    std::shared_ptr<void> request,
    ResponseCallback callback);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_goal_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_goal_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> goal_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_result_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_result_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> result_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_cancel_response() const = 0;

  /// \internal
  RCLCPP_ACTION_PUBLIC
  virtual
  void
  handle_cancel_response(
    const rmw_request_id_t & response_header,
    std::shared_ptr<void> cancel_response);

  /// \internal
  virtual
  std::shared_ptr<void>
  create_feedback_message() const = 0;

  /// \internal
  virtual
  void
  handle_feedback_message(std::shared_ptr<void> message) = 0;

  /// \internal
  virtual
  std::shared_ptr<void>
  create_status_message() const = 0;

  /// \internal
  virtual
  void
  handle_status_message(std::shared_ptr<void> message) = 0;

  // End API for communication between ClientBase and Client<>
  // ---------------------------------------------------------

  /// \internal
  /// Set a callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void
  set_on_ready_callback(
    EntityType entity_type,
    rcl_event_callback_t callback,
    const void * user_data);

  // Mutex to protect the callbacks storage.
  std::recursive_mutex listener_mutex_;
  // Storage for std::function callbacks to keep them in scope
  std::unordered_map<EntityType, std::function<void(size_t)>> entity_type_to_on_ready_callback_;

private:
  std::unique_ptr<ClientBaseImpl> pimpl_;

  /// Set a std::function callback to be called when the specified entity is ready
  RCLCPP_ACTION_PUBLIC
  void
  set_callback_to_entity(
    EntityType entity_type,
    std::function<void(size_t, int)> callback);

  bool on_ready_callback_set_{false};
};

/// Action Client
/**
 * This class creates an action client.
 *
 * To create an instance of an action client use `rclcpp_action::create_client()`.
 *
 * Internally, this class is responsible for:
 *  - coverting between the C++ action type and generic types for `rclcpp_action::ClientBase`, and
 *  - calling user callbacks.
 */
template<typename ActionT>
class Client : public ClientBase
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(Client<ActionT>)

  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using GoalResponseCallback = std::function<void (typename GoalHandle::SharedPtr)>;
  using FeedbackCallback = typename GoalHandle::FeedbackCallback;
  using ResultCallback = typename GoalHandle::ResultCallback;
  using CancelRequest = typename ActionT::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;
  using CancelCallback = std::function<void (typename CancelResponse::SharedPtr)>;

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

  /// Construct an action client.
  /**
   * This constructs an action client, but it will not work until it has been added to a node.
   * Use `rclcpp_action::create_client()` to both construct and add to a node.
   *
   * \param[in] node_base A pointer to the base interface of a node.
   * \param[in] node_graph A pointer to an interface that allows getting graph information about
   *   a node.
   * \param[in] node_logging A pointer to an interface that allows getting a node's logger.
   * \param[in] action_name The action name.
   * \param[in] client_options Options to pass to the underlying `rcl_action::rcl_action_client_t`.
   */
  Client(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging,
    const std::string & action_name,
    const rcl_action_client_options_t & client_options = rcl_action_client_get_default_options()
  )
  : ClientBase(
      node_base, node_graph, node_logging, action_name,
      rosidl_typesupport_cpp::get_action_type_support_handle<ActionT>(),
      client_options)
  {
  }

  /// Send an action goal and asynchronously get the result.
  /**
   * If the goal is accepted by an action server, the returned future is set to a `ClientGoalHandle`.
   * If the goal is rejected by an action server, then the future is set to a `nullptr`.
   *
   * The returned goal handle is used to monitor the status of the goal and get the final result.
   * It is valid as long as you hold a reference to the shared pointer or until the
   * rclcpp_action::Client is destroyed at which point the goal status will become UNKNOWN.
   *
   * \param[in] goal The goal request.
   * \param[in] options Options for sending the goal request. Contains references to callbacks for
   *   the goal response (accepted/rejected), feedback, and the final result.
   * \return A future that completes when the goal has been accepted or rejected.
   *   If the goal is rejected, then the result will be a `nullptr`.
   */
  std::shared_future<typename GoalHandle::SharedPtr>
  async_send_goal(const Goal & goal, const SendGoalOptions & options = SendGoalOptions())
  {
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename GoalHandle::SharedPtr>>();
    std::shared_future<typename GoalHandle::SharedPtr> future(promise->get_future());
    using GoalRequest = typename ActionT::Impl::SendGoalService::Request;
    auto goal_request = std::make_shared<GoalRequest>();
    goal_request->goal_id.uuid = this->generate_goal_id();
    goal_request->goal = goal;
    this->send_goal_request(
      std::static_pointer_cast<void>(goal_request),
      [this, goal_request, options, promise](std::shared_ptr<void> response) mutable
      {
        using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
        auto goal_response = std::static_pointer_cast<GoalResponse>(response);
        if (!goal_response->accepted) {
          promise->set_value(nullptr);
          if (options.goal_response_callback) {
            options.goal_response_callback(nullptr);
          }
          return;
        }
        GoalInfo goal_info;
        goal_info.goal_id.uuid = goal_request->goal_id.uuid;
        goal_info.stamp = goal_response->stamp;
        // Do not use std::make_shared as friendship cannot be forwarded.
        std::shared_ptr<GoalHandle> goal_handle(
          new GoalHandle(goal_info, options.feedback_callback, options.result_callback));
        {
          std::lock_guard<std::mutex> guard(goal_handles_mutex_);
          goal_handles_[goal_handle->get_goal_id()] = goal_handle;
        }
        promise->set_value(goal_handle);
        if (options.goal_response_callback) {
          options.goal_response_callback(goal_handle);
        }

        if (options.result_callback) {
          this->make_result_aware(goal_handle);
        }
      });

    // TODO(jacobperron): Encapsulate into it's own function and
    //                    consider exposing an option to disable this cleanup
    // To prevent the list from growing out of control, forget about any goals
    // with no more user references
    {
      std::lock_guard<std::mutex> guard(goal_handles_mutex_);
      auto goal_handle_it = goal_handles_.begin();
      while (goal_handle_it != goal_handles_.end()) {
        if (!goal_handle_it->second.lock()) {
          RCLCPP_DEBUG(
            this->get_logger(),
            "Dropping weak reference to goal handle during send_goal()");
          goal_handle_it = goal_handles_.erase(goal_handle_it);
        } else {
          ++goal_handle_it;
        }
      }
    }

    return future;
  }

  /// Asynchronously get the result for an active goal.
  /**
   * \throws exceptions::UnknownGoalHandleError If the goal unknown or already reached a terminal
   *   state, or if there was an error requesting the result.
   * \param[in] goal_handle The goal handle for which to get the result.
   * \param[in] result_callback Optional callback that is called when the result is received.
   * \return A future that is set to the goal result when the goal is finished.
   */
  std::shared_future<WrappedResult>
  async_get_result(
    typename GoalHandle::SharedPtr goal_handle,
    ResultCallback result_callback = nullptr)
  {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      throw exceptions::UnknownGoalHandleError();
    }
    if (goal_handle->is_invalidated()) {
      // This case can happen if there was a failure to send the result request
      // during the goal response callback
      throw goal_handle->invalidate_exception_;
    }
    if (result_callback) {
      // This will override any previously registered callback
      goal_handle->set_result_callback(result_callback);
    }
    this->make_result_aware(goal_handle);
    return goal_handle->async_get_result();
  }

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
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_goal(
    typename GoalHandle::SharedPtr goal_handle,
    CancelCallback cancel_callback = nullptr)
  {
    std::lock_guard<std::mutex> lock(goal_handles_mutex_);
    if (goal_handles_.count(goal_handle->get_goal_id()) == 0) {
      throw exceptions::UnknownGoalHandleError();
    }
    auto cancel_request = std::make_shared<CancelRequest>();
    // cancel_request->goal_info.goal_id = goal_handle->get_goal_id();
    cancel_request->goal_info.goal_id.uuid = goal_handle->get_goal_id();
    return async_cancel(cancel_request, cancel_callback);
  }

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
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_all_goals(CancelCallback cancel_callback = nullptr)
  {
    auto cancel_request = std::make_shared<CancelRequest>();
    // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
    std::fill(
      cancel_request->goal_info.goal_id.uuid.begin(),
      cancel_request->goal_info.goal_id.uuid.end(), 0u);
    return async_cancel(cancel_request, cancel_callback);
  }

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
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel_goals_before(
    const rclcpp::Time & stamp,
    CancelCallback cancel_callback = nullptr)
  {
    auto cancel_request = std::make_shared<CancelRequest>();
    // std::fill(cancel_request->goal_info.goal_id.uuid, 0u);
    std::fill(
      cancel_request->goal_info.goal_id.uuid.begin(),
      cancel_request->goal_info.goal_id.uuid.end(), 0u);
    cancel_request->goal_info.stamp = stamp;
    return async_cancel(cancel_request, cancel_callback);
  }

  virtual
  ~Client()
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    auto it = goal_handles_.begin();
    while (it != goal_handles_.end()) {
      typename GoalHandle::SharedPtr goal_handle = it->second.lock();
      if (goal_handle) {
        goal_handle->invalidate(exceptions::UnawareGoalHandleError());
      }
      it = goal_handles_.erase(it);
    }
  }

private:
  /// \internal
  std::shared_ptr<void>
  create_goal_response() const override
  {
    using GoalResponse = typename ActionT::Impl::SendGoalService::Response;
    return std::shared_ptr<void>(new GoalResponse());
  }

  /// \internal
  std::shared_ptr<void>
  create_result_response() const override
  {
    using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;
    return std::shared_ptr<void>(new GoalResultResponse());
  }

  /// \internal
  std::shared_ptr<void>
  create_cancel_response() const override
  {
    return std::shared_ptr<void>(new CancelResponse());
  }

  /// \internal
  std::shared_ptr<void>
  create_feedback_message() const override
  {
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;
    return std::shared_ptr<void>(new FeedbackMessage());
  }

  /// \internal
  void
  handle_feedback_message(std::shared_ptr<void> message) override
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    using FeedbackMessage = typename ActionT::Impl::FeedbackMessage;
    typename FeedbackMessage::SharedPtr feedback_message =
      std::static_pointer_cast<FeedbackMessage>(message);
    const GoalUUID & goal_id = feedback_message->goal_id.uuid;
    if (goal_handles_.count(goal_id) == 0) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Received feedback for unknown goal. Ignoring...");
      return;
    }
    typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id].lock();
    // Forget about the goal if there are no more user references
    if (!goal_handle) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Dropping weak reference to goal handle during feedback callback");
      goal_handles_.erase(goal_id);
      return;
    }
    auto feedback = std::make_shared<Feedback>();
    *feedback = feedback_message->feedback;
    goal_handle->call_feedback_callback(goal_handle, feedback);
  }

  /// \internal
  std::shared_ptr<void>
  create_status_message() const override
  {
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;
    return std::shared_ptr<void>(new GoalStatusMessage());
  }

  /// \internal
  void
  handle_status_message(std::shared_ptr<void> message) override
  {
    std::lock_guard<std::mutex> guard(goal_handles_mutex_);
    using GoalStatusMessage = typename ActionT::Impl::GoalStatusMessage;
    auto status_message = std::static_pointer_cast<GoalStatusMessage>(message);
    for (const GoalStatus & status : status_message->status_list) {
      const GoalUUID & goal_id = status.goal_info.goal_id.uuid;
      if (goal_handles_.count(goal_id) == 0) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Received status for unknown goal. Ignoring...");
        continue;
      }
      typename GoalHandle::SharedPtr goal_handle = goal_handles_[goal_id].lock();
      // Forget about the goal if there are no more user references
      if (!goal_handle) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Dropping weak reference to goal handle during status callback");
        goal_handles_.erase(goal_id);
        continue;
      }
      goal_handle->set_status(status.status);
    }
  }

  /// \internal
  void
  make_result_aware(typename GoalHandle::SharedPtr goal_handle)
  {
    // Avoid making more than one request
    if (goal_handle->set_result_awareness(true)) {
      return;
    }
    using GoalResultRequest = typename ActionT::Impl::GetResultService::Request;
    auto goal_result_request = std::make_shared<GoalResultRequest>();
    goal_result_request->goal_id.uuid = goal_handle->get_goal_id();
    try {
      this->send_result_request(
        std::static_pointer_cast<void>(goal_result_request),
        [goal_handle, this](std::shared_ptr<void> response) mutable
        {
          // Wrap the response in a struct with the fields a user cares about
          WrappedResult wrapped_result;
          using GoalResultResponse = typename ActionT::Impl::GetResultService::Response;
          auto result_response = std::static_pointer_cast<GoalResultResponse>(response);
          wrapped_result.result = std::make_shared<typename ActionT::Result>();
          *wrapped_result.result = result_response->result;
          wrapped_result.goal_id = goal_handle->get_goal_id();
          wrapped_result.code = static_cast<ResultCode>(result_response->status);
          goal_handle->set_result(wrapped_result);
          std::lock_guard<std::mutex> lock(goal_handles_mutex_);
          goal_handles_.erase(goal_handle->get_goal_id());
        });
    } catch (rclcpp::exceptions::RCLError & ex) {
      // This will cause an exception when the user tries to access the result
      goal_handle->invalidate(exceptions::UnawareGoalHandleError(ex.message));
    }
  }

  /// \internal
  std::shared_future<typename CancelResponse::SharedPtr>
  async_cancel(
    typename CancelRequest::SharedPtr cancel_request,
    CancelCallback cancel_callback = nullptr)
  {
    // Put promise in the heap to move it around.
    auto promise = std::make_shared<std::promise<typename CancelResponse::SharedPtr>>();
    std::shared_future<typename CancelResponse::SharedPtr> future(promise->get_future());
    this->send_cancel_request(
      std::static_pointer_cast<void>(cancel_request),
      [cancel_callback, promise](std::shared_ptr<void> response) mutable
      {
        auto cancel_response = std::static_pointer_cast<CancelResponse>(response);
        promise->set_value(cancel_response);
        if (cancel_callback) {
          cancel_callback(cancel_response);
        }
      });
    return future;
  }

  std::map<GoalUUID, typename GoalHandle::WeakPtr> goal_handles_;
  rcpputils::PIMutex goal_handles_mutex_;
};
}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__CLIENT_HPP_
