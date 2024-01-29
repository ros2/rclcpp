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
#include "rclcpp/executors/cbg_executor.hpp"
#include "rcpputils/scope_exit.hpp"
#include "test_msgs/msg/empty.hpp"

using namespace std::chrono_literals;
using performance_test_fixture::PerformanceTest;

constexpr unsigned int kNumberOfNodes = 10;
constexpr unsigned int kNumberOfPubSubs = 10;

class PerformanceTestExecutor : public PerformanceTest
{
public:
  void SetUp(benchmark::State & st)
  {
    rclcpp::init(0, nullptr);
    callback_count = 0;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      nodes.push_back(std::make_shared<rclcpp::Node>("my_node_" + std::to_string(i)));

      for (unsigned int j = 0u; j < kNumberOfPubSubs; j++) {
        publishers.push_back(
          nodes[i]->create_publisher<test_msgs::msg::Empty>(
            "/empty_msgs_" + std::to_string(i) + "_" + std::to_string(j), rclcpp::QoS(10)));

        auto callback = [this](test_msgs::msg::Empty::ConstSharedPtr) {this->callback_count++;};
        subscriptions.push_back(
          nodes[i]->create_subscription<test_msgs::msg::Empty>(
            "/empty_msgs_" + std::to_string(i) + "_" + std::to_string(j), rclcpp::QoS(10), std::move(callback)));
      }
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

  template<class Executer>
  void executor_spin_some(benchmark::State & st)
  {
    Executer executor;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      executor.add_node(nodes[i]);
      publishers[i]->publish(empty_msgs);
      executor.spin_some(100ms);
    }

    callback_count = 0;
    reset_heap_counters();

    for (auto _ : st) {
      (void)_;
      st.PauseTiming();
      for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
        publishers[i]->publish(empty_msgs);
      }
      st.ResumeTiming();

      callback_count = 0;
      while (callback_count < kNumberOfNodes) {
        executor.spin_some(100ms);
      }
    }
    if (callback_count == 0) {
      st.SkipWithError("No message was received");
    }
  }

  template<class Executer>
  void benchmark_wait_for_work(benchmark::State & st)
  {
    class ExecuterDerived : public Executer
    {
public:
      void call_wait_for_work(std::chrono::nanoseconds timeout)
      {
        Executer::wait_for_work(timeout);
      }
    };

    ExecuterDerived executor;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      executor.add_node(nodes[i]);
      executor.spin_some(100ms);
    }
    // we need one ready event
    publishers[0]->publish(empty_msgs);

    reset_heap_counters();

    for (auto _ : st) {
      (void)_;

      st.PauseTiming();
      // we need one ready event
      publishers[0]->publish(empty_msgs);
      st.ResumeTiming();


      executor.call_wait_for_work(100ms);
      st.PauseTiming();
      executor.spin_some(100ms);
      st.ResumeTiming();
    }
  }

  test_msgs::msg::Empty empty_msgs;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  std::vector<rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr> publishers;
  std::vector<rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr> subscriptions;
  uint callback_count;
};


BENCHMARK_F(PerformanceTestExecutor, single_thread_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutor, multi_thread_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutor, cbg_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::CBGExecutor>(st);
}


BENCHMARK_F(PerformanceTestExecutor, single_thread_executor_wait_for_work)(benchmark::State & st)
{
  benchmark_wait_for_work<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutor, multi_thread_executor_wait_for_work)(benchmark::State & st)
{
  benchmark_wait_for_work<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutor, cbg_executor_wait_for_work)(benchmark::State & st)
{
  benchmark_wait_for_work<rclcpp::executors::CBGExecutor>(st);
}

class CascadedPerformanceTestExecutor : public PerformanceTest
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

      auto callback = [this, i](test_msgs::msg::Empty::ConstSharedPtr) {
          if (i == kNumberOfNodes - 1) {
            this->callback_count++;
          } else {
            publishers[i + 1]->publish(empty_msgs);
          }
        };
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

  template<class Executer>
  void executor_spin_some(benchmark::State & st)
  {
    Executer executor;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      executor.add_node(nodes[i]);
      executor.spin_some(100ms);
    }

    callback_count = 0;
    reset_heap_counters();

    for (auto _ : st) {
      (void)_;
      // start the cascasde
      publishers[0]->publish(empty_msgs);

      callback_count = 0;
      while (callback_count == 0) {
        executor.spin_some(100ms);
      }
    }
    if (callback_count == 0) {
      st.SkipWithError("No message was received");
    }
  }

  test_msgs::msg::Empty empty_msgs;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  std::vector<rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr> publishers;
  std::vector<rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr> subscriptions;
  uint callback_count;
};


BENCHMARK_F(
  CascadedPerformanceTestExecutor,
  single_thread_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(CascadedPerformanceTestExecutor, multi_thread_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(CascadedPerformanceTestExecutor, cbg_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::CBGExecutor>(st);
}


class PerformanceTestExecutorMultipleCallbackGroups : public PerformanceTest
{
public:
//   static constexpr unsigned int kNumberOfNodes = 20;

  void SetUp(benchmark::State & st)
  {
    rclcpp::init(0, nullptr);
    callback_count = 0;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      nodes.push_back(std::make_shared<rclcpp::Node>("my_node_" + std::to_string(i)));

      callback_groups.push_back(
        nodes[i]->create_callback_group(
          rclcpp::CallbackGroupType::
          MutuallyExclusive));

      rclcpp::PublisherOptions pubOps;
      pubOps.callback_group = callback_groups[i];

      publishers.push_back(
        nodes[i]->create_publisher<test_msgs::msg::Empty>(
          "/empty_msgs_" + std::to_string(i), rclcpp::QoS(10), pubOps));

      rclcpp::SubscriptionOptions subOps;
      subOps.callback_group = callback_groups[i];

      auto callback = [this](test_msgs::msg::Empty::ConstSharedPtr) {this->callback_count++;};
      subscriptions.push_back(
        nodes[i]->create_subscription<test_msgs::msg::Empty>(
          "/empty_msgs_" + std::to_string(i), rclcpp::QoS(10), std::move(callback), subOps));
    }
    PerformanceTest::SetUp(st);
  }
  void TearDown(benchmark::State & st)
  {
    PerformanceTest::TearDown(st);
    subscriptions.clear();
    publishers.clear();
    callback_groups.clear();
    nodes.clear();
    rclcpp::shutdown();
  }

  template<class Executer>
  void executor_spin_some(benchmark::State & st)
  {
    Executer executor;
    for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
      executor.add_node(nodes[i]);
      publishers[i]->publish(empty_msgs);
      executor.spin_some(100ms);
    }

    callback_count = 0;
    reset_heap_counters();

    for (auto _ : st) {
      (void)_;
      st.PauseTiming();
      for (unsigned int i = 0u; i < kNumberOfNodes; i++) {
        publishers[i]->publish(empty_msgs);
      }
      st.ResumeTiming();

      callback_count = 0;
      while (callback_count < kNumberOfNodes) {
        executor.spin_some(100ms);
      }
    }
    if (callback_count == 0) {
      st.SkipWithError("No message was received");
    }
  }

  test_msgs::msg::Empty empty_msgs;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  std::vector<rclcpp::Publisher<test_msgs::msg::Empty>::SharedPtr> publishers;
  std::vector<rclcpp::Subscription<test_msgs::msg::Empty>::SharedPtr> subscriptions;
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups;
  uint callback_count;
};


BENCHMARK_F(
  PerformanceTestExecutorMultipleCallbackGroups,
  single_thread_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorMultipleCallbackGroups,
  multi_thread_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorMultipleCallbackGroups,
  cbg_executor_spin_some)(benchmark::State & st)
{
  executor_spin_some<rclcpp::executors::CBGExecutor>(st);
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

  template<class Executor>
  void add_node(benchmark::State & st)
  {
    Executor executor;
    for (auto _ : st) {
      (void)_;
      executor.add_node(node);
      st.PauseTiming();
      executor.remove_node(node);
      st.ResumeTiming();
    }
  }

  template<class Executor>
  void remove_node(benchmark::State & st)
  {
    Executor executor;
    for (auto _ : st) {
      (void)_;
      st.PauseTiming();
      executor.add_node(node);
      st.ResumeTiming();
      executor.remove_node(node);
    }
  }


  template<class Executor>
  void spin_until_future_complete(benchmark::State & st)
  {
    Executor executor;
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
      (void)_;
      ret = executor.spin_until_future_complete(shared_future, 100ms);
      if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        st.SkipWithError(rcutils_get_error_string().str);
        break;
      }
    }

  }

  template<class Executor>
  void spin_node_until_future_complete(benchmark::State & st)
  {
    Executor executor;
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
      (void)_;
      ret = rclcpp::executors::spin_node_until_future_complete(
        executor, node, shared_future, 1s);
      if (ret != rclcpp::FutureReturnCode::SUCCESS) {
        st.SkipWithError(rcutils_get_error_string().str);
        break;
      }
    }

  }

  rclcpp::Node::SharedPtr node;
};


BENCHMARK_F(PerformanceTestExecutorSimple, single_thread_executor_add_node)(benchmark::State & st)
{
  add_node<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple, single_thread_executor_remove_node)(benchmark::State & st)
{
  remove_node<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutorSimple, multi_thread_executor_add_node)(benchmark::State & st)
{
  add_node<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutorSimple, multi_thread_executor_remove_node)(benchmark::State & st)
{
  remove_node<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutorSimple, cbg_executor_add_node)(benchmark::State & st)
{
  add_node<rclcpp::executors::CBGExecutor>(st);
}

BENCHMARK_F(PerformanceTestExecutorSimple, cbg_executor_remove_node)(benchmark::State & st)
{
  remove_node<rclcpp::executors::CBGExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_add_node)(benchmark::State & st)
{
  add_node<rclcpp::executors::StaticSingleThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_remove_node)(benchmark::State & st)
{
  remove_node<rclcpp::executors::StaticSingleThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  single_thread_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  spin_node_until_future_complete<rclcpp::executors::SingleThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  multi_thread_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  spin_node_until_future_complete<rclcpp::executors::MultiThreadedExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  cbg_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  spin_node_until_future_complete<rclcpp::executors::CBGExecutor>(st);
}

BENCHMARK_F(
  PerformanceTestExecutorSimple,
  static_single_thread_executor_spin_node_until_future_complete)(benchmark::State & st)
{
  spin_node_until_future_complete<rclcpp::executors::StaticSingleThreadedExecutor>(st);
}

class SharedPtrHolder : public PerformanceTest
{
public:
  void SetUp(benchmark::State & st)
  {
    shared_ptr = std::make_shared<std::string>("foo");
    weak_ptr = shared_ptr;
    PerformanceTest::SetUp(st);
  }
  void TearDown(benchmark::State & st)
  {
    PerformanceTest::TearDown(st);
    shared_ptr.reset();
    weak_ptr.reset();
  }
  std::shared_ptr<std::string> shared_ptr;
  std::weak_ptr<std::string> weak_ptr;
};


BENCHMARK_F(SharedPtrHolder,
  shared_ptr_ref)(benchmark::State & st)
{
    for (auto _ : st) {
    (void)_;
      size_t cnt = shared_ptr.use_count();
      benchmark::DoNotOptimize(cnt);
    }
}

BENCHMARK_F(SharedPtrHolder,
  weakt_upcast)(benchmark::State & st)
{
    for (auto _ : st) {
    (void)_;

      std::shared_ptr<std::string> ptr = weak_ptr.lock();
      benchmark::DoNotOptimize(ptr);

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
    (void)_;
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
    (void)_;
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
  RCPPUTILS_SCOPE_EXIT(
  {
    rcl_ret_t ret = rcl_wait_set_fini(&wait_set);
    if (ret != RCL_RET_OK) {
      st.SkipWithError(rcutils_get_error_string().str);
    }
  });

  auto memory_strategy = rclcpp::memory_strategies::create_default_strategy();
  rclcpp::GuardCondition guard_condition(shared_context);

  entities_collector_->init(&wait_set, memory_strategy);
  RCPPUTILS_SCOPE_EXIT(entities_collector_->fini());

  reset_heap_counters();

  for (auto _ : st) {
    (void)_;
    std::shared_ptr<void> data = entities_collector_->take_data();
    entities_collector_->execute(data);
  }
}
