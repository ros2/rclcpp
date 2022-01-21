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
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_available_states.hpp"
#include "lifecycle_msgs/srv/get_available_transitions.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "performance_test_fixture/performance_test_fixture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

/**
 * Benchmarks for measuring performance of a lifecycle_node client.
 *
 * A service client of a lifecycle node is not a class that's explicitly part of ros2 core,
 * but it is the expected interface for interacting with a lifecycle node, so for that reason
 * this benchmark exists.
 */

using namespace std::chrono_literals;
constexpr char const * lifecycle_node_name = "lc_talker";

constexpr char const * node_get_state_topic = "/lc_talker/get_state";
constexpr char const * node_change_state_topic = "/lc_talker/change_state";
constexpr char const * node_get_available_states_topic = "/lc_talker/get_available_states";
constexpr char const * node_get_available_transitions_topic =
  "/lc_talker/get_available_transitions";
constexpr char const * node_get_transition_graph_topic =
  "/lc_talker/get_transition_graph";
const lifecycle_msgs::msg::State unknown_state = lifecycle_msgs::msg::State();
class LifecycleServiceClient : public rclcpp::Node
{
public:
  explicit LifecycleServiceClient(std::string node_name)
  : Node(node_name)
  {
    client_get_available_states_ = this->create_client<lifecycle_msgs::srv::GetAvailableStates>(
      node_get_available_states_topic);
    client_get_available_transitions_ =
      this->create_client<lifecycle_msgs::srv::GetAvailableTransitions>(
      node_get_available_transitions_topic);
    client_get_transition_graph_ =
      this->create_client<lifecycle_msgs::srv::GetAvailableTransitions>(
      node_get_transition_graph_topic);
    client_get_state_ = this->create_client<lifecycle_msgs::srv::GetState>(
      node_get_state_topic);
    client_change_state_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      node_change_state_topic);
  }

  lifecycle_msgs::msg::State
  get_state(std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!client_get_state_->wait_for_service(time_out)) {
      throw std::runtime_error("Wait for service timed out");
    }

    auto future_result = client_get_state_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      throw std::runtime_error("Get state request failed");
    }

    if (!future_result.valid()) {
      throw std::runtime_error("Future result was not valid");
    }
    return future_result.get()->current_state;
  }

  bool
  change_state(std::uint8_t transition, std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!client_change_state_->wait_for_service(time_out)) {
      throw std::runtime_error("Wait for service timed out");
    }

    auto future_result = client_change_state_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      throw std::runtime_error("Change state request failed");
    }

    if (!future_result.valid()) {
      throw std::runtime_error("Future result was not valid");
    }

    return future_result.get()->success;
  }

  std::vector<lifecycle_msgs::msg::State>
  get_available_states(std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetAvailableStates::Request>();

    if (!client_get_available_states_->wait_for_service(time_out)) {
      throw std::runtime_error("Wait for service timed out");
    }

    auto future_result = client_get_available_states_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      throw std::runtime_error("Get available states request failed");
    }

    if (!future_result.valid()) {
      throw std::runtime_error("Future result was not valid");
    }

    return future_result.get()->available_states;
  }

  std::vector<lifecycle_msgs::msg::TransitionDescription>
  get_available_transitions(std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetAvailableTransitions::Request>();

    if (!client_get_available_transitions_->wait_for_service(time_out)) {
      throw std::runtime_error("Wait for service timed out");
    }

    auto future_result = client_get_available_transitions_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      throw std::runtime_error("Get available transitions request failed");
    }

    if (!future_result.valid()) {
      throw std::runtime_error("Future result was not valid");
    }

    return future_result.get()->available_transitions;
  }

  std::vector<lifecycle_msgs::msg::TransitionDescription>
  get_transition_graph(std::chrono::seconds time_out = 1s)
  {
    auto request = std::make_shared<lifecycle_msgs::srv::GetAvailableTransitions::Request>();

    if (!client_get_transition_graph_->wait_for_service(time_out)) {
      throw std::runtime_error("Wait for service timed out");
    }

    auto future_result = client_get_transition_graph_->async_send_request(request);
    auto future_status = future_result.wait_for(time_out);

    if (future_status != std::future_status::ready) {
      throw std::runtime_error("Get transition graph request failed");
    }

    if (!future_result.valid()) {
      throw std::runtime_error("Future result was not valid");
    }

    return future_result.get()->available_transitions;
  }

private:
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetAvailableStates>>
  client_get_available_states_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetAvailableTransitions>>
  client_get_available_transitions_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetAvailableTransitions>>
  client_get_transition_graph_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state_;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state_;
};

class BenchmarkLifecycleClient : public performance_test_fixture::PerformanceTest
{
public:
  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    lifecycle_node = std::make_shared<rclcpp_lifecycle::LifecycleNode>(lifecycle_node_name);
    lifecycle_client = std::make_shared<LifecycleServiceClient>("lifecycle_client");
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(lifecycle_node->get_node_base_interface());
    executor->add_node(lifecycle_client->get_node_base_interface());
    spinner_ = std::thread(&rclcpp::executors::SingleThreadedExecutor::spin, executor);
    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);
    executor->cancel();
    spinner_.join();
    executor.reset();
    lifecycle_client.reset();
    lifecycle_node.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node;
  std::shared_ptr<LifecycleServiceClient> lifecycle_client;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread spinner_;
};

BENCHMARK_F(BenchmarkLifecycleClient, get_state)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    const auto lifecycle_state = lifecycle_client->get_state();
    if (lifecycle_state.id != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      const std::string msg =
        std::string("Expected state did not match actual: ") +
        std::to_string(lifecycle_state.id);
      state.SkipWithError(msg.c_str());
    }
    benchmark::DoNotOptimize(lifecycle_state);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(BenchmarkLifecycleClient, change_state)(benchmark::State & state) {
  bool success =
    lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (!success) {
    state.SkipWithError("Transition to configured state failed");
  }

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    success =
      lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
    if (!success) {
      state.SkipWithError("Transition to active state failed");
    }

    success =
      lifecycle_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    if (!success) {
      state.SkipWithError("Transition to inactive state failed");
    }
  }
}

BENCHMARK_F(BenchmarkLifecycleClient, get_available_states)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    constexpr size_t expected_states = 11u;
    const auto states = lifecycle_client->get_available_states();
    if (states.size() != expected_states) {
      const std::string msg =
        std::string("Expected number of states did not match actual: ") +
        std::to_string(states.size());
      state.SkipWithError(msg.c_str());
    }
    benchmark::DoNotOptimize(states);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(BenchmarkLifecycleClient, get_available_transitions)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    constexpr size_t expected_transitions = 2u;
    const auto transitions = lifecycle_client->get_available_transitions();
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

BENCHMARK_F(BenchmarkLifecycleClient, get_transition_graph)(benchmark::State & state) {
  for (auto _ : state) {
    (void)_;
    constexpr size_t expected_transitions = 25u;
    const auto transitions = lifecycle_client->get_transition_graph();
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
