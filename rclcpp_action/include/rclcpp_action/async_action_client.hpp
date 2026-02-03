// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_ACTION__ASYNC_ACTION_CLIENT_HPP_
#define RCLCPP_ACTION__ASYNC_ACTION_CLIENT_HPP_

#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/node.hpp"
#include "rclcpp/executors/async_helpers.hpp"
#include "rclcpp_action/client.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rclcpp_action/visibility_control.hpp"

namespace rclcpp_action
{

/// Exception thrown when an action server is not ready
class ActionNotReadyException : public rclcpp::executors::AsyncOperationException
{
public:
  RCLCPP_ACTION_PUBLIC
  explicit ActionNotReadyException(const std::string & action_name)
  : AsyncOperationException("Action server '" + action_name + "' is not ready"),
    action_name_(action_name)
  {
  }

  RCLCPP_ACTION_PUBLIC
  const std::string & action_name() const noexcept
  {
    return action_name_;
  }

private:
  std::string action_name_;
};

/// Exception thrown when a goal is rejected
class GoalRejectedException : public rclcpp::executors::AsyncOperationException
{
public:
  RCLCPP_ACTION_PUBLIC
  GoalRejectedException()
  : AsyncOperationException("Goal was rejected by the action server")
  {
  }
};

/// Exception thrown when a goal is cancelled
class GoalCancelledException : public rclcpp::executors::AsyncOperationException
{
public:
  RCLCPP_ACTION_PUBLIC
  GoalCancelledException()
  : AsyncOperationException("Goal was cancelled")
  {
  }
};

/// Exception thrown when a goal is aborted
class GoalAbortedException : public rclcpp::executors::AsyncOperationException
{
public:
  RCLCPP_ACTION_PUBLIC
  GoalAbortedException()
  : AsyncOperationException("Goal was aborted by the action server")
  {
  }
};

/// Async wrapper for ROS2 action clients
/**
 * Provides enhanced async functionality for action clients including:
 * - Simplified goal/feedback/result flow
 * - Timeout support
 * - Cancellation support
 * - Progress tracking
 */
template<typename ActionT>
class AsyncActionClientHelper
{
public:
  using Goal = typename ActionT::Goal;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using GoalHandleSharedPtr = typename GoalHandle::SharedPtr;
  using WrappedResult = typename GoalHandle::WrappedResult;
  using CancelRequest = typename ActionT::Impl::CancelGoalService::Request;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;

  /// Options for goal execution
  struct GoalOptions
  {
    /// Maximum time to wait for goal completion
    std::chrono::milliseconds timeout{std::chrono::milliseconds::max()};

    /// Cancellation token for cooperative cancellation
    rclcpp::executors::CancellationToken cancellation_token;

    /// Callback invoked for each feedback message
    std::function<void(const Feedback &)> feedback_callback;

    /// Callback invoked when goal is accepted
    std::function<void(GoalHandleSharedPtr)> goal_accepted_callback;

    /// Callback invoked when goal is rejected
    std::function<void(GoalHandleSharedPtr)> goal_rejected_callback;

    /// Goal UUID (auto-generated if not specified)
    rclcpp_action::GoalUUID goal_uuid{};
  };

  /// Create an AsyncActionClientHelper from a node
  /**
   * \param node The ROS node to create the client on
   * \param action_name Name of the action to connect to
   */
  RCLCPP_ACTION_PUBLIC
  AsyncActionClientHelper(
    rclcpp::Node::SharedPtr node,
    const std::string & action_name)
  : node_(node),
    action_name_(action_name)
  {
    callback_group_ = node->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);

    client_ = rclcpp_action::create_client<ActionT>(
      node,
      action_name,
      callback_group_);
  }

  /// Create an AsyncActionClientHelper from an existing client
  /**
   * \param client Existing action client to wrap
   */
  RCLCPP_ACTION_PUBLIC
  explicit AsyncActionClientHelper(typename Client<ActionT>::SharedPtr client)
  : client_(client)
  {
  }

  /// Send a goal and wait for the result
  /**
   * This is the simplest interface - sends a goal and returns the final result.
   *
   * \param goal The goal to send
   * \return Future with AsyncResult containing the wrapped result
   */
  std::future<rclcpp::executors::AsyncResult<WrappedResult>>
  async_send_goal(const Goal & goal)
  {
    GoalOptions options;
    return async_send_goal(goal, options);
  }

  /// Send a goal with timeout and wait for the result
  /**
   * \param goal The goal to send
   * \param timeout Maximum time to wait for completion
   * \return Future with AsyncResult containing the wrapped result or timeout status
   */
  std::future<rclcpp::executors::AsyncResult<WrappedResult>>
  async_send_goal(const Goal & goal, std::chrono::milliseconds timeout)
  {
    GoalOptions options;
    options.timeout = timeout;
    return async_send_goal(goal, options);
  }

  /// Send a goal with full options
  /**
   * \param goal The goal to send
   * \param options Goal execution options (timeout, cancellation, callbacks)
   * \return Future with AsyncResult containing the wrapped result
   */
  std::future<rclcpp::executors::AsyncResult<WrappedResult>>
  async_send_goal(const Goal & goal, const GoalOptions & options)
  {
    auto promise = std::make_shared<std::promise<rclcpp::executors::AsyncResult<WrappedResult>>>();
    auto future = promise->get_future();

    auto completed_flag = std::make_shared<std::atomic<bool>>(false);
    auto mutex = std::make_shared<std::mutex>();
    auto goal_handle_ptr = std::make_shared<GoalHandleSharedPtr>();

    // Set up native send_goal options
    typename Client<ActionT>::SendGoalOptions send_options;

    // Feedback callback
    if (options.feedback_callback) {
      auto feedback_cb = options.feedback_callback;
      send_options.feedback_callback =
        [feedback_cb](GoalHandleSharedPtr, const std::shared_ptr<const Feedback> feedback) {
          feedback_cb(*feedback);
        };
    }

    // Goal response callback
    send_options.goal_response_callback =
      [options, promise, completed_flag, mutex, goal_handle_ptr](
      const typename ClientGoalHandle<ActionT>::SharedPtr & goal_handle) {
        *goal_handle_ptr = goal_handle;

        if (!goal_handle) {
          std::lock_guard<std::mutex> lock(*mutex);
          if (!completed_flag->exchange(true)) {
            promise->set_value(
              rclcpp::executors::AsyncResult<WrappedResult>::error(
                std::make_exception_ptr(GoalRejectedException())));
          }
          if (options.goal_rejected_callback) {
            options.goal_rejected_callback(goal_handle);
          }
          return;
        }

        if (options.goal_accepted_callback) {
          options.goal_accepted_callback(goal_handle);
        }
      };

    // Result callback
    send_options.result_callback =
      [promise, completed_flag, mutex](const WrappedResult & result) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::success(result));
              break;
            case rclcpp_action::ResultCode::CANCELED:
              promise->set_value(
                rclcpp::executors::AsyncResult<WrappedResult>::error(
                  std::make_exception_ptr(GoalCancelledException())));
              break;
            case rclcpp_action::ResultCode::ABORTED:
              promise->set_value(
                rclcpp::executors::AsyncResult<WrappedResult>::error(
                  std::make_exception_ptr(GoalAbortedException())));
              break;
            default:
              promise->set_value(
                rclcpp::executors::AsyncResult<WrappedResult>::error(
                  std::make_exception_ptr(
                    rclcpp::executors::AsyncOperationException("Unknown result code"))));
              break;
          }
        }
      };

    // Cancellation handling
    std::shared_ptr<void> cancel_handle;
    if (options.cancellation_token.is_cancellation_requested()) {
      // Already cancelled
      promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::cancelled());
      return future;
    }

    cancel_handle = options.cancellation_token.register_callback(
      [this, promise, completed_flag, mutex, goal_handle_ptr]() {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          if (*goal_handle_ptr) {
            client_->async_cancel_goal(*goal_handle_ptr);
          }
          promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::cancelled());
        }
      });

    // Timeout handling
    if (options.timeout != std::chrono::milliseconds::max()) {
      std::thread timeout_thread(
        [this, promise, completed_flag, mutex, goal_handle_ptr, timeout = options.timeout,
        cancel_handle]() {
          std::this_thread::sleep_for(timeout);
          std::lock_guard<std::mutex> lock(*mutex);
          if (!completed_flag->exchange(true)) {
            if (*goal_handle_ptr) {
              client_->async_cancel_goal(*goal_handle_ptr);
            }
            promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::timeout());
          }
        });
      timeout_thread.detach();
    }

    // Send the goal
    client_->async_send_goal(goal, send_options);

    return future;
  }

  /// Send a goal and get the goal handle for manual control
  /**
   * Use this when you need direct control over the goal lifecycle.
   *
   * \param goal The goal to send
   * \return Future with AsyncResult containing the goal handle
   */
  std::future<rclcpp::executors::AsyncResult<GoalHandleSharedPtr>>
  async_send_goal_handle(const Goal & goal)
  {
    auto promise =
      std::make_shared<std::promise<rclcpp::executors::AsyncResult<GoalHandleSharedPtr>>>();
    auto future = promise->get_future();

    typename Client<ActionT>::SendGoalOptions send_options;
    send_options.goal_response_callback =
      [promise](const typename ClientGoalHandle<ActionT>::SharedPtr & goal_handle) {
        if (goal_handle) {
          promise->set_value(
            rclcpp::executors::AsyncResult<GoalHandleSharedPtr>::success(goal_handle));
        } else {
          promise->set_value(
            rclcpp::executors::AsyncResult<GoalHandleSharedPtr>::error(
              std::make_exception_ptr(GoalRejectedException())));
        }
      };

    client_->async_send_goal(goal, send_options);

    return future;
  }

  /// Wait for a result on an existing goal handle
  /**
   * \param goal_handle The goal handle to wait on
   * \return Future with AsyncResult containing the wrapped result
   */
  std::future<rclcpp::executors::AsyncResult<WrappedResult>>
  async_get_result(GoalHandleSharedPtr goal_handle)
  {
    auto promise = std::make_shared<std::promise<rclcpp::executors::AsyncResult<WrappedResult>>>();
    auto future = promise->get_future();

    client_->async_get_result(
      goal_handle,
      [promise](const WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::success(result));
            break;
          case rclcpp_action::ResultCode::CANCELED:
            promise->set_value(
              rclcpp::executors::AsyncResult<WrappedResult>::error(
                std::make_exception_ptr(GoalCancelledException())));
            break;
          case rclcpp_action::ResultCode::ABORTED:
            promise->set_value(
              rclcpp::executors::AsyncResult<WrappedResult>::error(
                std::make_exception_ptr(GoalAbortedException())));
            break;
          default:
            promise->set_value(
              rclcpp::executors::AsyncResult<WrappedResult>::error(
                std::make_exception_ptr(
                  rclcpp::executors::AsyncOperationException("Unknown result code"))));
            break;
        }
      });

    return future;
  }

  /// Wait for a result with timeout
  /**
   * \param goal_handle The goal handle to wait on
   * \param timeout Maximum time to wait
   * \return Future with AsyncResult containing the wrapped result or timeout status
   */
  std::future<rclcpp::executors::AsyncResult<WrappedResult>>
  async_get_result(GoalHandleSharedPtr goal_handle, std::chrono::milliseconds timeout)
  {
    auto promise = std::make_shared<std::promise<rclcpp::executors::AsyncResult<WrappedResult>>>();
    auto future = promise->get_future();

    auto completed_flag = std::make_shared<std::atomic<bool>>(false);
    auto mutex = std::make_shared<std::mutex>();

    // Timeout thread
    std::thread timeout_thread(
      [this, promise, completed_flag, mutex, goal_handle, timeout]() {
        std::this_thread::sleep_for(timeout);
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          client_->async_cancel_goal(goal_handle);
          promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::timeout());
        }
      });
    timeout_thread.detach();

    client_->async_get_result(
      goal_handle,
      [promise, completed_flag, mutex](const WrappedResult & result) {
        std::lock_guard<std::mutex> lock(*mutex);
        if (!completed_flag->exchange(true)) {
          switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
              promise->set_value(rclcpp::executors::AsyncResult<WrappedResult>::success(result));
              break;
            case rclcpp_action::ResultCode::CANCELED:
              promise->set_value(
                rclcpp::executors::AsyncResult<WrappedResult>::error(
                  std::make_exception_ptr(GoalCancelledException())));
              break;
            case rclcpp_action::ResultCode::ABORTED:
              promise->set_value(
                rclcpp::executors::AsyncResult<WrappedResult>::error(
                  std::make_exception_ptr(GoalAbortedException())));
              break;
            default:
              promise->set_value(
                rclcpp::executors::AsyncResult<WrappedResult>::error(
                  std::make_exception_ptr(
                    rclcpp::executors::AsyncOperationException("Unknown result code"))));
              break;
          }
        }
      });

    return future;
  }

  /// Cancel a specific goal
  /**
   * \param goal_handle The goal handle to cancel
   * \return Future with AsyncResult containing the cancel response
   */
  std::future<rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>>
  async_cancel_goal(GoalHandleSharedPtr goal_handle)
  {
    auto promise = std::make_shared<
      std::promise<rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>>>();
    auto future = promise->get_future();

    client_->async_cancel_goal(
      goal_handle,
      [promise](const std::shared_ptr<CancelResponse> response) {
        promise->set_value(
          rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>::success(response));
      });

    return future;
  }

  /// Cancel all goals
  /**
   * \return Future with AsyncResult containing the cancel response
   */
  std::future<rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>>
  async_cancel_all_goals()
  {
    auto promise = std::make_shared<
      std::promise<rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>>>();
    auto future = promise->get_future();

    client_->async_cancel_all_goals(
      [promise](const std::shared_ptr<CancelResponse> response) {
        promise->set_value(
          rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>::success(response));
      });

    return future;
  }

  /// Cancel goals before a specific timestamp
  /**
   * \param stamp The timestamp - goals sent before this time will be cancelled
   * \return Future with AsyncResult containing the cancel response
   */
  std::future<rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>>
  async_cancel_goals_before(const rclcpp::Time & stamp)
  {
    auto promise = std::make_shared<
      std::promise<rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>>>();
    auto future = promise->get_future();

    client_->async_cancel_goals_before(
      stamp,
      [promise](const std::shared_ptr<CancelResponse> response) {
        promise->set_value(
          rclcpp::executors::AsyncResult<std::shared_ptr<CancelResponse>>::success(response));
      });

    return future;
  }

  /// Check if the action server is ready
  bool action_server_is_ready() const
  {
    return client_->action_server_is_ready();
  }

  /// Wait for the action server to become available
  /**
   * \param timeout Maximum time to wait
   * \return Future that resolves to true if server is ready, false if timed out
   */
  std::future<bool> wait_for_action_server(std::chrono::milliseconds timeout)
  {
    return std::async(std::launch::async, [this, timeout]() {
        return client_->wait_for_action_server(timeout);
      });
  }

  /// Get the underlying action client
  typename Client<ActionT>::SharedPtr get_client() const
  {
    return client_;
  }

  /// Get the action name
  const std::string & get_action_name() const
  {
    return action_name_;
  }

private:
  typename Client<ActionT>::SharedPtr client_;
  rclcpp::Node::WeakPtr node_;
  std::string action_name_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};

/// Builder class for creating goal options with fluent interface
template<typename ActionT>
class GoalOptionsBuilder
{
public:
  using GoalOptions = typename AsyncActionClientHelper<ActionT>::GoalOptions;
  using Feedback = typename ActionT::Feedback;
  using GoalHandle = ClientGoalHandle<ActionT>;
  using GoalHandleSharedPtr = typename GoalHandle::SharedPtr;

  GoalOptionsBuilder() = default;

  GoalOptionsBuilder & with_timeout(std::chrono::milliseconds timeout)
  {
    options_.timeout = timeout;
    return *this;
  }

  GoalOptionsBuilder & with_cancellation_token(rclcpp::executors::CancellationToken token)
  {
    options_.cancellation_token = token;
    return *this;
  }

  GoalOptionsBuilder & on_feedback(std::function<void(const Feedback &)> callback)
  {
    options_.feedback_callback = std::move(callback);
    return *this;
  }

  GoalOptionsBuilder & on_accepted(std::function<void(GoalHandleSharedPtr)> callback)
  {
    options_.goal_accepted_callback = std::move(callback);
    return *this;
  }

  GoalOptionsBuilder & on_rejected(std::function<void(GoalHandleSharedPtr)> callback)
  {
    options_.goal_rejected_callback = std::move(callback);
    return *this;
  }

  GoalOptions build() const
  {
    return options_;
  }

  operator GoalOptions() const
  {
    return options_;
  }

private:
  GoalOptions options_;
};

/// Factory function for creating GoalOptionsBuilder
template<typename ActionT>
GoalOptionsBuilder<ActionT> make_goal_options()
{
  return GoalOptionsBuilder<ActionT>();
}

}  // namespace rclcpp_action

#endif  // RCLCPP_ACTION__ASYNC_ACTION_CLIENT_HPP_
