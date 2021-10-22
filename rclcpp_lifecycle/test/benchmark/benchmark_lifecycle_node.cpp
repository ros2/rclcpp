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

#include <benchmark/benchmark.h>

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "performance_test_fixture/performance_test_fixture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/**
 * Benchmarks for evaluating rclcpp_lifecycle::LifecycleNode.
 *
 * Much of the lifecycle node API calls rclcpp::Node and node_interface functions, which are
 * likely to be inlined and are not of much value to benchmark.
 */

class BenchmarkLifecycleNodeConstruction : public performance_test_fixture::PerformanceTest
{
public:
  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);
    rclcpp::shutdown();
  }
};

BENCHMARK_F(BenchmarkLifecycleNodeConstruction, construct_lifecycle_node)(
  benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node", "ns");
    PERFORMANCE_TEST_FIXTURE_PAUSE_MEASUREMENTS(
      state,
    {
      node.reset();
    });
  }
}

BENCHMARK_F(BenchmarkLifecycleNodeConstruction, destroy_lifecycle_node)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node(nullptr);
    PERFORMANCE_TEST_FIXTURE_PAUSE_MEASUREMENTS(
      state,
    {
      node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node", "ns");
    });
    node.reset();
  }
}

class BenchmarkLifecycleNode : public performance_test_fixture::PerformanceTest
{
public:
  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("node", "ns");
    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);
    node.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node;
};

// This is a simple getter, but it crosses over into the rcl library.
BENCHMARK_F(BenchmarkLifecycleNode, get_current_state)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    const auto & lifecycle_state = node->get_current_state();
    if (lifecycle_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      const std::string message =
        std::string("Node's current state is: ") + std::to_string(lifecycle_state.id());
      state.SkipWithError(message.c_str());
    }
    benchmark::DoNotOptimize(lifecycle_state);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(BenchmarkLifecycleNode, get_available_states)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    constexpr size_t expected_states = 11u;
    const auto lifecycle_states = node->get_available_states();
    if (lifecycle_states.size() != expected_states) {
      const std::string msg = std::to_string(lifecycle_states.size());
      state.SkipWithError(msg.c_str());
    }
    benchmark::DoNotOptimize(lifecycle_states);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(BenchmarkLifecycleNode, get_available_transitions)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    constexpr size_t expected_transitions = 2u;
    const auto & transitions = node->get_available_transitions();
    if (transitions.size() != expected_transitions) {
      const std::string msg = std::to_string(transitions.size());
      state.SkipWithError(msg.c_str());
    }
    benchmark::DoNotOptimize(transitions);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(BenchmarkLifecycleNode, get_transition_graph)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    constexpr size_t expected_transitions = 25u;
    const auto & transitions = node->get_transition_graph();
    if (transitions.size() != expected_transitions) {
      const std::string msg =
        std::string("Expected number of transitions did not match actual: ") +
        std::to_string(transitions.size());
      state.SkipWithError(msg.c_str());
    }
    benchmark::DoNotOptimize(transitions);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(BenchmarkLifecycleNode, transition_valid_state)(benchmark::State & state) {
  const auto & configured =
    node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (configured.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    state.SkipWithError("Transition to configured state failed");
  }

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    const auto & active =
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (active.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      state.SkipWithError("Transition to active state failed");
    }
    const auto & inactive =
      node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    if (inactive.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      state.SkipWithError("Transition to inactive state failed");
    }
    benchmark::DoNotOptimize(active);
    benchmark::DoNotOptimize(inactive);
    benchmark::ClobberMemory();
  }
}
