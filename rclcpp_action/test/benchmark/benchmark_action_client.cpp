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

#include <memory>
#include <string>

#include "performance_test_fixture/performance_test_fixture.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/action/fibonacci.hpp"

using performance_test_fixture::PerformanceTest;

using Fibonacci = test_msgs::action::Fibonacci;
using GoalHandle = rclcpp_action::ServerGoalHandle<Fibonacci>;
using CancelResponse = typename Fibonacci::Impl::CancelGoalService::Response;
using GoalUUID = rclcpp_action::GoalUUID;

constexpr char fibonacci_action_name[] = "fibonacci";

namespace
{

test_msgs::action::Fibonacci::Goal GetGoalOfOrder(int order)
{
  test_msgs::action::Fibonacci::Goal goal;
  goal.order = order;
  return goal;
}

}  // namespace

class ActionClientPerformanceTest : public PerformanceTest
{
public:
  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    // Use same node for server and client to avoid interprocess communication
    node = std::make_shared<rclcpp::Node>("node", "ns");
    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void SetUpServer(const std::string & action_name)
  {
    // This action server accepts and defers so that execution can be timed separately from
    // accepting the goal
    action_server = rclcpp_action::create_server<Fibonacci>(
      node, action_name,
      [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal> goal) {
        if (goal->order <= 0) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<GoalHandle> goal_handle) {
        current_goal_handle = goal_handle;
      });
  }

  void ComputeFibonacciAndSetSuccess()
  {
    // This method is suprisingly slow, primarily due to the goal_handle->execute/succeed calls.
    current_goal_handle->execute();
    const auto goal = current_goal_handle->get_goal();
    auto result = std::make_shared<Fibonacci::Result>();

    // Should be checked by the server above
    assert(goal->order > 0);
    result->sequence.resize(static_cast<size_t>(goal->order));
    result->sequence[0] = 0;
    if (goal->order == 1) {
      current_goal_handle->succeed(result);
      return;
    }
    result->sequence[1] = 1;
    if (goal->order == 2) {
      current_goal_handle->succeed(result);
      return;
    }
    for (size_t i = 2; i < static_cast<size_t>(goal->order); ++i) {
      result->sequence[i] =
        result->sequence[i - 1] + result->sequence[i - 2];
    }
    current_goal_handle->succeed(result);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);
    // Ensure proper sequencing of destruction
    current_goal_handle.reset();
    action_server.reset();
    node.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp_action::Server<test_msgs::action::Fibonacci>> action_server;

  // Goal handle needs to be kept alive by the server in order for client request specific to the
  // goal to succeed.
  std::shared_ptr<GoalHandle> current_goal_handle;
};

BENCHMARK_F(ActionClientPerformanceTest, construct_client_without_server)(benchmark::State & state)
{
  constexpr char action_name[] = "no_corresponding_server";
  for (auto _ : state) {
    (void)_;
    auto client = rclcpp_action::create_client<Fibonacci>(node, action_name);

    // Only timing construction, so destruction needs to happen explicitly while timing is paused
    state.PauseTiming();
    client.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ActionClientPerformanceTest, construct_client_with_server)(benchmark::State & state)
{
  SetUpServer(fibonacci_action_name);
  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);

    // Only timing construction, so destruction needs to happen explicitly while timing is paused
    state.PauseTiming();
    client.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ActionClientPerformanceTest, destroy_client)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    // This client does not have a corresponding server
    state.PauseTiming();
    auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
    state.ResumeTiming();

    client.reset();
  }
}

BENCHMARK_F(ActionClientPerformanceTest, async_send_goal_only)(benchmark::State & state)
{
  auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
  SetUpServer(fibonacci_action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    state.SkipWithError("Waiting for server timed out");
    return;
  }

  const auto goal = GetGoalOfOrder(5);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto future_goal_handle = client->async_send_goal(goal);
  }
}

BENCHMARK_F(ActionClientPerformanceTest, async_send_goal_rejected)(benchmark::State & state)
{
  auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
  SetUpServer(fibonacci_action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    state.SkipWithError("Waiting for server timed out");
    return;
  }

  // Order of 0 is invalid
  const auto goal = GetGoalOfOrder(0);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto future_goal_handle = client->async_send_goal(goal);
    rclcpp::spin_until_future_complete(node, future_goal_handle, std::chrono::seconds(1));
    if (!future_goal_handle.valid()) {
      state.SkipWithError("Shared future was invalid");
      return;
    }
    if (nullptr != future_goal_handle.get()) {
      state.SkipWithError("Invalid goal was not rejected");
      return;
    }
  }
}

BENCHMARK_F(ActionClientPerformanceTest, async_send_goal_get_accepted_response)(
  benchmark::State & state)
{
  auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
  SetUpServer(fibonacci_action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    state.SkipWithError("Waiting for server timed out");
    return;
  }

  const auto goal = GetGoalOfOrder(10);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    // This server's execution is deferred
    auto future_goal_handle = client->async_send_goal(goal);
    rclcpp::spin_until_future_complete(node, future_goal_handle, std::chrono::seconds(1));

    if (!future_goal_handle.valid()) {
      state.SkipWithError("Shared future was invalid");
      return;
    }

    auto goal_handle = future_goal_handle.get();
    if (rclcpp_action::GoalStatus::STATUS_ACCEPTED != goal_handle->get_status()) {
      state.SkipWithError("Valid goal was not accepted");
      return;
    }
  }
}

BENCHMARK_F(ActionClientPerformanceTest, async_get_result)(benchmark::State & state)
{
  auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
  SetUpServer(fibonacci_action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    state.SkipWithError("Waiting for server timed out");
    return;
  }

  constexpr int expected_order = 5;
  const auto goal = GetGoalOfOrder(expected_order);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    // Send goal, accept and execute while timing is paused
    state.PauseTiming();
    auto future_goal_handle = client->async_send_goal(goal);

    // Action server accepts and defers, so this spin doesn't include result
    rclcpp::spin_until_future_complete(node, future_goal_handle, std::chrono::seconds(1));

    if (!future_goal_handle.valid()) {
      state.SkipWithError("Shared future was invalid");
      return;
    }
    auto goal_handle = future_goal_handle.get();
    if (nullptr == goal_handle) {
      state.SkipWithError("Goal handle was a nullptr");
      break;
    }

    // Perform actual execution and set success
    ComputeFibonacciAndSetSuccess();
    state.ResumeTiming();

    // Measure how long it takes client to receive the succeeded result
    auto future_result = client->async_get_result(goal_handle);
    rclcpp::spin_until_future_complete(node, future_result, std::chrono::seconds(1));
    const auto & wrapped_result = future_result.get();
    if (rclcpp_action::ResultCode::SUCCEEDED != wrapped_result.code) {
      state.SkipWithError("Fibonacci action did not succeed");
      break;
    }

    const auto & sequence = wrapped_result.result->sequence;
    if (sequence.size() != expected_order || sequence.back() != 3) {
      state.SkipWithError("Fibonacci result was not correct");
      break;
    }
  }
}

BENCHMARK_F(ActionClientPerformanceTest, async_cancel_goal)(benchmark::State & state)
{
  auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
  SetUpServer(fibonacci_action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    state.SkipWithError("Waiting for server timed out");
    return;
  }

  constexpr int expected_order = 5;
  const auto goal = GetGoalOfOrder(expected_order);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto future_goal_handle = client->async_send_goal(goal);

    // Action server accepts and defers, so action can be canceled
    rclcpp::spin_until_future_complete(node, future_goal_handle, std::chrono::seconds(1));
    auto goal_handle = future_goal_handle.get();
    state.ResumeTiming();

    auto future_cancel = client->async_cancel_goal(goal_handle);
    rclcpp::spin_until_future_complete(node, future_cancel, std::chrono::seconds(1));
    auto cancel_response = future_cancel.get();

    using CancelActionResponse = test_msgs::action::Fibonacci::Impl::CancelGoalService::Response;
    if (CancelActionResponse::ERROR_NONE != cancel_response->return_code) {
      state.SkipWithError("Cancel request did not succeed");
      break;
    }
  }
}

BENCHMARK_F(ActionClientPerformanceTest, async_cancel_all_goals)(benchmark::State & state)
{
  auto client = rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
  SetUpServer(fibonacci_action_name);
  if (!client->wait_for_action_server(std::chrono::seconds(1))) {
    state.SkipWithError("Waiting for server timed out");
    return;
  }

  constexpr int expected_order = 5;
  const auto goal = GetGoalOfOrder(expected_order);
  constexpr int num_concurrently_inflight_goals = 10u;

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    for (int i = 0; i < num_concurrently_inflight_goals; ++i) {
      auto future_goal_handle = client->async_send_goal(goal);
      rclcpp::spin_until_future_complete(node, future_goal_handle, std::chrono::seconds(1));
    }
    // Action server accepts and defers, so action can be canceled
    state.ResumeTiming();

    auto future_cancel_all = client->async_cancel_all_goals();
    rclcpp::spin_until_future_complete(node, future_cancel_all, std::chrono::seconds(1));
    auto cancel_response = future_cancel_all.get();

    using CancelActionResponse = test_msgs::action::Fibonacci::Impl::CancelGoalService::Response;
    if (CancelActionResponse::ERROR_NONE != cancel_response->return_code) {
      state.SkipWithError("Cancel request did not succeed");
      break;
    }
  }
}
