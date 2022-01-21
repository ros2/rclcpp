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

class ActionServerPerformanceTest : public PerformanceTest
{
public:
  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("node", "ns");
    action_client =
      rclcpp_action::create_client<Fibonacci>(node, fibonacci_action_name);
    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);

    action_client.reset();
    node.reset();
    rclcpp::shutdown();
  }

  auto AsyncSendGoalOfOrder(const int order)
  {
    test_msgs::action::Fibonacci::Goal goal;
    goal.order = order;

    return action_client->async_send_goal(goal);
  }

protected:
  std::shared_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp_action::Client<Fibonacci>> action_client;
};

BENCHMARK_F(ActionServerPerformanceTest, construct_server_without_client)(benchmark::State & state)
{
  constexpr char action_name[] = "no_corresponding_client";
  for (auto _ : state) {
    (void)_;
    auto action_server = rclcpp_action::create_server<Fibonacci>(
      node, action_name,
      [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](std::shared_ptr<GoalHandle>) {});
    benchmark::DoNotOptimize(action_server);
    benchmark::ClobberMemory();

    state.PauseTiming();
    action_server.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ActionServerPerformanceTest, construct_server_with_client)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    auto action_server = rclcpp_action::create_server<Fibonacci>(
      node, fibonacci_action_name,
      [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](std::shared_ptr<GoalHandle>) {});
    benchmark::DoNotOptimize(action_server);
    benchmark::ClobberMemory();

    state.PauseTiming();
    action_server.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ActionServerPerformanceTest, destroy_server)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto action_server = rclcpp_action::create_server<Fibonacci>(
      node, fibonacci_action_name,
      [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
      },
      [](std::shared_ptr<GoalHandle>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [](std::shared_ptr<GoalHandle>) {});
    state.ResumeTiming();
    benchmark::DoNotOptimize(action_server);
    benchmark::ClobberMemory();

    action_server.reset();
  }
}

BENCHMARK_F(ActionServerPerformanceTest, action_server_accept_goal)(benchmark::State & state)
{
  std::shared_ptr<GoalHandle> current_goal_handle = nullptr;
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, fibonacci_action_name,
    [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    },
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&current_goal_handle](std::shared_ptr<GoalHandle> goal_handle) {
      current_goal_handle = goal_handle;
    });

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto client_goal_handle_future = AsyncSendGoalOfOrder(1);
    state.ResumeTiming();

    rclcpp::spin_until_future_complete(node, client_goal_handle_future);
    auto goal_handle = client_goal_handle_future.get();
    if (rclcpp_action::GoalStatus::STATUS_ACCEPTED != goal_handle->get_status()) {
      state.SkipWithError("Valid goal was not accepted");
      return;
    }
  }
}

BENCHMARK_F(ActionServerPerformanceTest, action_server_cancel_goal)(benchmark::State & state)
{
  // The goal handle needs to be assigned to a variable for the lifetime of the goal so that it is
  // not cleaned up before the cancel request is received and processed.
  std::shared_ptr<GoalHandle> server_goal_handle = nullptr;
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, fibonacci_action_name,
    [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    },
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&server_goal_handle](std::shared_ptr<GoalHandle> goal_handle) {
      server_goal_handle = goal_handle;
    });

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto client_goal_handle_future = AsyncSendGoalOfOrder(1);
    // This spin completes when the goal has been accepted, but not executed because server
    // responds with ACCEPT_AND_DEFER
    rclcpp::spin_until_future_complete(node, client_goal_handle_future, std::chrono::seconds(1));
    auto client_goal_handle = client_goal_handle_future.get();
    auto future_cancel = action_client->async_cancel_goal(client_goal_handle);
    state.ResumeTiming();

    rclcpp::spin_until_future_complete(node, future_cancel, std::chrono::seconds(1));
    auto cancel_response = future_cancel.get();
    using CancelActionResponse = test_msgs::action::Fibonacci::Impl::CancelGoalService::Response;
    if (CancelActionResponse::ERROR_NONE != cancel_response->return_code) {
      state.SkipWithError("Cancel request did not succeed");
      break;
    }
  }
}

BENCHMARK_F(ActionServerPerformanceTest, action_server_execute_goal)(benchmark::State & state)
{
  std::shared_ptr<GoalHandle> server_goal_handle = nullptr;
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, fibonacci_action_name,
    [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    },
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&server_goal_handle](std::shared_ptr<GoalHandle> goal_handle) {
      server_goal_handle = goal_handle;
    });

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto client_goal_handle_future = AsyncSendGoalOfOrder(1);

    rclcpp::spin_until_future_complete(node, client_goal_handle_future);
    auto goal_handle = client_goal_handle_future.get();
    if (rclcpp_action::GoalStatus::STATUS_ACCEPTED != goal_handle->get_status()) {
      state.SkipWithError("Valid goal was not accepted");
      return;
    }
    state.ResumeTiming();

    server_goal_handle->execute();
  }
}

BENCHMARK_F(ActionServerPerformanceTest, action_server_set_success)(benchmark::State & state)
{
  constexpr int goal_order = 1;
  std::shared_ptr<GoalHandle> server_goal_handle = nullptr;
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, fibonacci_action_name,
    [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    },
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&server_goal_handle](std::shared_ptr<GoalHandle> goal_handle) {
      server_goal_handle = goal_handle;
    });

  // MSVC and Clang disagree how goal_order should be captured here. Though this capture is a bit
  // too wide, they at least could agree it was fine. In my testing MSVC errored if goal_order was
  // not captured, but clang would warn if it was explicitly captured.
  const auto result = [&]() {
      auto action_result = std::make_shared<Fibonacci::Result>();
      for (int i = 0; i < goal_order; ++i) {
        // Not the fibonacci sequence, but that's not important to this benchmark
        action_result->sequence.push_back(i);
      }
      return action_result;
    } ();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto client_goal_handle_future = AsyncSendGoalOfOrder(goal_order);

    rclcpp::spin_until_future_complete(node, client_goal_handle_future);
    auto goal_handle = client_goal_handle_future.get();
    if (rclcpp_action::GoalStatus::STATUS_ACCEPTED != goal_handle->get_status()) {
      state.SkipWithError("Valid goal was not accepted");
      return;
    }
    server_goal_handle->execute();
    state.ResumeTiming();

    server_goal_handle->succeed(result);
  }
}

BENCHMARK_F(ActionServerPerformanceTest, action_server_abort)(benchmark::State & state)
{
  constexpr int goal_order = 1;
  std::shared_ptr<GoalHandle> server_goal_handle = nullptr;
  auto action_server = rclcpp_action::create_server<Fibonacci>(
    node, fibonacci_action_name,
    [](const GoalUUID &, std::shared_ptr<const Fibonacci::Goal>) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    },
    [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [&server_goal_handle](std::shared_ptr<GoalHandle> goal_handle) {
      server_goal_handle = goal_handle;
    });

  // Capturing with & because MSVC and Clang disagree about how to capture goal_order
  const auto result = [&]() {
      auto action_result = std::make_shared<Fibonacci::Result>();
      for (int i = 0; i < goal_order; ++i) {
        // Not the fibonacci sequence, but that's not important to this benchmark
        action_result->sequence.push_back(i);
      }
      return action_result;
    } ();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto client_goal_handle_future = AsyncSendGoalOfOrder(goal_order);

    rclcpp::spin_until_future_complete(node, client_goal_handle_future);
    auto goal_handle = client_goal_handle_future.get();
    if (rclcpp_action::GoalStatus::STATUS_ACCEPTED != goal_handle->get_status()) {
      state.SkipWithError("Valid goal was not accepted");
      return;
    }
    server_goal_handle->execute();
    state.ResumeTiming();

    server_goal_handle->abort(result);
  }
}
