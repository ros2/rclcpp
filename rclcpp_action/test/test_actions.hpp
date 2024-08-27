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
// limitations under the License.#include <gtest/gtest.h>

#pragma once

#include <thread>

#include <test_msgs/action/fibonacci.hpp>
#include <rclcpp_action/client_goal_handle.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp/experimental/executors/events_executor/events_executor.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using Fibonacci = test_msgs::action::Fibonacci;
using ActionGoalHandle = rclcpp_action::ClientGoalHandle<Fibonacci>;
using GoalHandleFibonacci = typename rclcpp_action::ServerGoalHandle<Fibonacci>;
using GoalHandleSharedPtr = typename std::shared_ptr<GoalHandleFibonacci>;

using rclcpp::experimental::executors::EventsExecutor;

// Define a structure to hold test info and utilities
class TestInfo
{
public:
  ~TestInfo()
  {
    this->exit_thread = true;

    if (server_thread.joinable()) {
      server_thread.join();
    }
  }

  rclcpp::Node::SharedPtr
  create_node(std::string name, bool ipc_enabled)
  {
      auto node_options = rclcpp::NodeOptions();
      node_options.use_intra_process_comms(ipc_enabled);

      return rclcpp::Node::make_shared(name, "test_namespace", node_options);
  }

  rclcpp_action::Client<Fibonacci>::SharedPtr
  create_action_client(rclcpp::Node::SharedPtr & node)
  {
      return rclcpp_action::create_client<Fibonacci>(
        node, "fibonacci"
      );
  }

  // The server executes the following in a thread when accepting the goal
  void execute()
  {
    auto & goal_handle = this->server_goal_handle_;

    rclcpp::Rate loop_rate(double(this->server_rate_hz)); // 100Hz
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    feedback->sequence = this->feedback_sequence;

    while(!this->exit_thread && rclcpp::ok())
    {
      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<Fibonacci::Result>();
        result->sequence = this->canceled_sequence;
        goal_handle->canceled(result);
        return;
      }

      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }
  }

  void succeed_goal()
  {
    // Wait for feedback to be received, otherwise succeding the goal
    // will remove the goal handle, and feedback callback will not
    // be called
    wait_for_feedback_called();
    this->exit_thread = true;
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = this->succeeded_sequence;
    this->server_goal_handle_->succeed(result);
  }

  void abort_goal()
  {
    wait_for_feedback_called();
    this->exit_thread = true;
    auto result = std::make_shared<Fibonacci::Result>();
    result->sequence = this->aborted_sequence;
    this->server_goal_handle_->abort(result);
  }

  // Server: Handle goal callback
  rclcpp_action::GoalResponse
  handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    (void)uuid;
    if (goal->order > 20) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    this->server_goal_handle_ = goal_handle;
    this->server_thread = std::thread([&]() { execute(); });
  }

  rclcpp_action::Server<Fibonacci>::SharedPtr
  create_action_server(rclcpp::Node::SharedPtr & node)
  {
    return rclcpp_action::create_server<Fibonacci>(
            node,
            "fibonacci",
            [this] (const rclcpp_action::GoalUUID & guuid,
                std::shared_ptr<const Fibonacci::Goal> goal)
            {
                return this->handle_goal(guuid, goal);
            },
            [this] (const std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                (void) goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            },
            [this] (const std::shared_ptr<GoalHandleFibonacci> goal_handle)
            {
                return this->handle_accepted(goal_handle);
            }
      );
  }

  rclcpp::Executor::UniquePtr create_executor(bool use_events_executor)
  {
    if (use_events_executor) {
      return std::make_unique<EventsExecutor>();
    } else {
      return std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    }
  }

  rclcpp_action::Client<Fibonacci>::SendGoalOptions create_goal_options()
  {
    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    send_goal_options.result_callback =
      [this](const typename ActionGoalHandle::WrappedResult & result)
      {
        this->result_cb_called = true;
        (void)result;
      };

    send_goal_options.goal_response_callback =
      [this](typename ActionGoalHandle::SharedPtr goal_handle)
        {
          this->goal_response_cb_called = true;
          (void)goal_handle;
        };

    send_goal_options.feedback_callback = [this](
          typename ActionGoalHandle::SharedPtr handle,
          const std::shared_ptr<const Fibonacci::Feedback> feedback)
        {
          (void) handle;
          this->feedback_cb_called = result_is_correct(
            feedback->sequence, rclcpp_action::ResultCode::UNKNOWN);
        };

    return send_goal_options;
  }

  void wait_for_feedback_called()
  {
    rclcpp::Rate loop_rate(100);
    auto start_time = std::chrono::steady_clock::now();
    while(!this->feedback_cb_called && rclcpp::ok()) {
      auto current_time = std::chrono::steady_clock::now();
      auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
      if (elapsed_time >= 5) {
        break;
      }
      loop_rate.sleep();
    }
  }

  bool result_is_correct(
    std::vector<int> result_sequence,
    rclcpp_action::ResultCode result_code)
  {
    std::vector<int> expected_sequence;

    switch (result_code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        expected_sequence = this->succeeded_sequence;
        break;
      case rclcpp_action::ResultCode::CANCELED:
        expected_sequence = this->canceled_sequence;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        expected_sequence = this->aborted_sequence;
        break;
      case rclcpp_action::ResultCode::UNKNOWN:
        expected_sequence = this->feedback_sequence;
    }

    if (result_sequence.size() != expected_sequence.size()) {
      return false;
    }

    for (size_t i = 0; i < result_sequence.size(); i++) {
      if (result_sequence[i] != expected_sequence[i]) {
        return false;
      }
    }

    return true;
  }

  bool result_callback_called() { return result_cb_called; }
  bool feedback_callback_called() { return feedback_cb_called; }
  size_t server_rate_hz{500};

private:
  GoalHandleSharedPtr server_goal_handle_;
  std::atomic<bool> result_cb_called{false};
  std::atomic<bool> feedback_cb_called{false};
  std::atomic<bool> goal_response_cb_called{false};
  std::atomic<bool> exit_thread{false};
  std::vector<int> succeeded_sequence{0, 1, 1, 2, 3};
  std::vector<int> feedback_sequence{1, 2, 3};
  std::vector<int> canceled_sequence{42};
  std::vector<int> aborted_sequence{6, 6, 6};
  std::thread server_thread;
};
