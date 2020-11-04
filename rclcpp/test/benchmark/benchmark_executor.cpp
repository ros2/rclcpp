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
#include <utility>
#include <vector>

#include "performance_test_fixture/performance_test_fixture.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/scope_exit.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using performance_test_fixture::PerformanceTest;

constexpr unsigned int kNumberOfNodes = 10;

class PerformanceTestExecutor : public PerformanceTest
{
public:
  void SetUp(benchmark::State & st)
  {
    rclcpp::init(0, nullptr);
    callback_count = 0;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      nodes.push_back(std::make_shared<rclcpp::Node>("my_node_" + std::to_string(i)));

      publishers.push_back(
        nodes[i]->create_publisher<test_msgs::msg::Empty>(
          "/empty_msgs_" + std::to_string(i), rclcpp::QoS(10)));

      auto callback = [this](test_msgs::msg::Empty::SharedPtr) {this->callback_count++;};
      subscriptions.push_back(
        nodes[i]->create_subscription<test_msgs::msg::Empty>(
          "/empty_msgs_" + std::to_string(i), rclcpp::QoS(10), std::move(callback)));
    }
    PerformanceTest::SetUp(st);
  }
  void TearDown(benchmark::State & st)
  {
    PerformanceTest::TearDown(st);
    subscriptions.clear();
    publishers.clear();
    nodes.clear();
    rclcpp::shutdown();
  }

  test_msgs::msg::Empty empty_msgs;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  std::vector<rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr> publishers;
  std::vector<rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr> subscriptions;
  int callback_count;
};

BENCHMARK_F(PerformanceTestExecutor, single_thread_executor_spin_some)(benchmark::State & st)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
    executor.add_node(nodes[i]);
    publishers[i]->publish(empty_msgs);
    executor.spin_some(100ms);
  }

  callback_count = 0;
  reset_heap_counters();

  for (auto _ : st) {
    st.PauseTiming();
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      publishers[i]->publish(empty_msgs);
    }
    st.ResumeTiming();

    executor.spin_some(100ms);
  }
  if (callback_count == 0) {
    st.SkipWithError("No message was received");
  }
}

BENCHMARK_F(PerformanceTestExecutor, multi_thread_executor_spin_some)(benchmark::State & st)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
    executor.add_node(nodes[i]);
    publishers[i]->publish(empty_msgs);
    executor.spin_some(100ms);
  }

  callback_count = 0;
  reset_heap_counters();

  for (auto _ : st) {
    st.PauseTiming();
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      publishers[i]->publish(empty_msgs);
    }
    st.ResumeTiming();

    executor.spin_some(100ms);
  }
  if (callback_count == 0) {
    st.SkipWithError("No message was received");
  }
}

class PerformanceTestExecutorSimple : public PerformanceTest
{
public:
  void SetUp(benchmark::State & st)
  {
    rclcpp::init(0, nullptr);
    node = std::make_shared<rclcpp::Node>("my_node");

    PerformanceTest::SetUp(st);
  }
  void TearDown(benchmark::State & st)
  {
    PerformanceTest::TearDown(st);
    node.reset();
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node;
};


BENCHMARK_F(PerformanceTestExecutorSimple, single_thread_executor_add_node)(benchmark::State & st)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto _ : st) {
    executor.add_node(node);
    st.PauseTiming();
    executor.remove_node(node);
    st.ResumeTiming();
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple, single_thread_executor_remove_node)(benchmark::State & st)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  for (auto _ : st) {
    st.PauseTiming();
    executor.add_node(node);
    st.ResumeTiming();
    executor.remove_node(node);
  }
}

BENCHMARK_F(PerformanceTestExecutorSimple, multi_thread_executor_add_node)(benchmark::State & st)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto _ : st) {
    executor.add_node(node);
    st.PauseTiming();
    executor.remove_node(node);
    st.ResumeTiming();
  }
}

BENCHMARK_F(PerformanceTestExecutorSimple, multi_thread_executor_remove_node)(benchmark::State & st)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto _ : st) {
    st.PauseTiming();
    executor.add_node(node);
    st.ResumeTiming();
    executor.remove_node(node);
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_add_node)(benchmark::State & st)
{
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto _ : st) {
    executor.add_node(node);
    st.PauseTiming();
    executor.remove_node(node);
    st.ResumeTiming();
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_remove_node)(benchmark::State & st)
{
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto _ : st) {
    st.PauseTiming();
    executor.add_node(node);
    st.ResumeTiming();
    executor.remove_node(node);
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_spin_until_future_complete)(benchmark::State & st)
{
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  auto ret = executor.spin_until_future_complete(shared_future, 100ms);
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    st.SkipWithError(rcutils_get_error_string().str);
  }

  reset_heap_counters();

  for (auto _ : st) {
    // static_single_thread_executor has a special design. We need to add/remove the node each
    // time you call spin
    st.PauseTiming();
    executor.add_node(node);
    st.ResumeTiming();

    ret = executor.spin_until_future_complete(shared_future, 100ms);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
      break;
    }
    st.PauseTiming();
    executor.remove_node(node);
    st.ResumeTiming();
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  single_thread_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  auto ret = rclcpp::executors::spin_node_until_future_complete(
    executor, node, shared_future, 1s);
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    st.SkipWithError(rcutils_get_error_string().str);
  }

  reset_heap_counters();

  for (auto _ : st) {
    ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
      break;
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  multi_thread_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  auto ret = rclcpp::executors::spin_node_until_future_complete(
    executor, node, shared_future, 1s);
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    st.SkipWithError(rcutils_get_error_string().str);
  }

  reset_heap_counters();

  for (auto _ : st) {
    ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
      break;
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
      break;
    }
  }
}

BENCHMARK_F(PerformanceTestExecutorSimple, spin_until_future_complete)(benchmark::State & st)
{
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  auto ret = rclcpp::spin_until_future_complete(node, shared_future, 1s);
  if (ret != rclcpp::FutureReturnCode::SUCCESS) {
    st.SkipWithError(rcutils_get_error_string().str);
  }

  reset_heap_counters();

  for (auto _ : st) {
    ret = rclcpp::spin_until_future_complete(node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
      break;
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_executor_entities_collector_execute)(benchmark::State & st)
{
  rclcpp::executors::StaticExecutorEntitiesCollector::SharedPtr entities_collector_ =
    std::make_shared<rclcpp::executors::StaticExecutorEntitiesCollector>();
  entities_collector_->add_node(node->get_node_base_interface());

  rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
  rcl_allocator_t allocator = rcl_get_default_allocator();
  auto shared_context = node->get_node_base_interface()->get_context();
  rcl_context_t * context = shared_context->get_rcl_context().get();
  rcl_ret_t ret = rcl_wait_set_init(&wait_set, 100, 100, 100, 100, 100, 100, context, allocator);
  if (ret != RCL_RET_OK) {
    st.SkipWithError(rcutils_get_error_string().str);
  }
  RCLCPP_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_wait_set_fini(&wait_set);
    if (ret != RCL_RET_OK) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
  });

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);
  rcl_guard_condition_t rcl_guard_condition = guard_condition.get_rcl_guard_condition();

  entities_collector_->init(&wait_set, memory_strategy, &rcl_guard_condition);
  RCLCPP_SCOPE_EXIT(entities_collector_->fini());

  reset_heap_counters();

  for (auto _ : st) {
    entities_collector_->execute();
  }
}
