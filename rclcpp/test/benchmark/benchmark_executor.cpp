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

#include "performance_test_fixture/performance_test_fixture.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/scope_exit.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using performance_test_fixture::PerformanceTest;

class PerformanceTestExecutor : public PerformanceTest
{
public:
  void SetUp(benchmark::State & st)
  {
    rclcpp::init(0, nullptr);
    callback_count = 0;
    node = std::make_shared<rclcpp::Node>("my_node");

    publisher = node->create_publisher<test_msgs::msg::Empty>(
      "empty_msgs", rclcpp::QoS(10));
    auto callback = [this](test_msgs::msg::Empty::SharedPtr) {this->callback_count++;};
    subscription =
      node->create_subscription<test_msgs::msg::Empty>(
      "empty_msgs", rclcpp::QoS(10), std::move(callback));

    PerformanceTest::SetUp(st);
  }
  void TearDown(benchmark::State & st)
  {
    PerformanceTest::TearDown(st);
    publisher.reset();
    subscription.reset();
    node.reset();
    rclcpp::shutdown();
  }

  test_msgs::msg::Empty empty_msgs;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr publisher;
  rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr subscription;
  int callback_count;
};

BENCHMARK_F(PerformanceTestExecutor, single_thread_executor)(benchmark::State & st)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  reset_heap_counters();

  for (auto _ : st) {
    st.PauseTiming();
    publisher->publish(empty_msgs);
    st.ResumeTiming();

    executor.spin_some(100ms);

    benchmark::ClobberMemory();
  }
  if (callback_count == 0) {
    st.SkipWithError("No message was received");
  }
}

BENCHMARK_F(PerformanceTestExecutor, multi_thread_executor)(benchmark::State & st)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  reset_heap_counters();

  for (auto _ : st) {
    st.PauseTiming();
    publisher->publish(empty_msgs);
    st.ResumeTiming();

    executor.spin_some(100ms);

    benchmark::ClobberMemory();
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

BENCHMARK_F(PerformanceTestExecutorSimple, static_single_thread_executor)(benchmark::State & st)
{
  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = executor.spin_until_future_complete(shared_future, 100ms);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
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

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
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

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
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
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  single_thread_executor_spin_node_base_interface_until_future_complete)(benchmark::State & st)
{
  rclcpp::executors::SingleThreadedExecutor executor;
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node->get_node_base_interface(), shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  multi_thread_executor_spin_node_base_interface_until_future_complete)(benchmark::State & st)
{
  rclcpp::executors::MultiThreadedExecutor executor;
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::executors::spin_node_until_future_complete(
      executor, node->get_node_base_interface(), shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_spin_node_base_interface_until_future_complete)(
  benchmark::State &
  st)
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
      executor, node->get_node_base_interface(), shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
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

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::spin_until_future_complete(node, shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  node_base_interface_spin_until_future_complete)(benchmark::State & st)
{
  // test success of an immediately finishing future
  std::promise<bool> promise;
  std::future<bool> future = promise.get_future();
  promise.set_value(true);
  auto shared_future = future.share();

  reset_heap_counters();

  for (auto _ : st) {
    auto ret = rclcpp::spin_until_future_complete(
      node->get_node_base_interface(), shared_future, 1s);
    if (ret != rclcpp::FutureReturnCode::SUCCESS) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
  }
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_executor_entities_collector)(benchmark::State & st)
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
