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

#include "performance_test_fixture/performance_test_fixture.hpp"
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/srv/empty.hpp"

using performance_test_fixture::PerformanceTest;

constexpr char empty_service_name[] = "empty_service";

class ServicePerformanceTest : public PerformanceTest
{
public:
  ServicePerformanceTest()
  : callback_count(0) {}

  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    node = std::make_unique<rclcpp::Node>("node", "ns");
    empty_client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
    callback_count = 0;

    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);

    empty_client.reset();
    node.reset();
    rclcpp::shutdown();
  }

  void ServiceCallback(
    const test_msgs::srv::Empty::Request::SharedPtr,
    test_msgs::srv::Empty::Response::SharedPtr)
  {
    callback_count++;
  }

protected:
  std::unique_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Client<test_msgs::srv::Empty>> empty_client;
  int callback_count;
};

BENCHMARK_F(ServicePerformanceTest, construct_service_no_client)(benchmark::State & state) {
  auto callback = std::bind(
    &ServicePerformanceTest::ServiceCallback,
    this, std::placeholders::_1, std::placeholders::_2);

  auto outer_service = node->create_service<test_msgs::srv::Empty>("not_a_service", callback);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto service = node->create_service<test_msgs::srv::Empty>("not_a_service", callback);
    benchmark::DoNotOptimize(service);
    benchmark::ClobberMemory();

    state.PauseTiming();
    service.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ServicePerformanceTest, construct_service_empty_srv)(benchmark::State & state) {
  auto callback = std::bind(
    &ServicePerformanceTest::ServiceCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  auto outer_service = node->create_service<test_msgs::srv::Empty>(empty_service_name, callback);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto service = node->create_service<test_msgs::srv::Empty>(empty_service_name, callback);
    benchmark::DoNotOptimize(service);
    benchmark::ClobberMemory();

    state.PauseTiming();
    service.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ServicePerformanceTest, destroy_service_empty_srv)(benchmark::State & state) {
  auto callback = std::bind(
    &ServicePerformanceTest::ServiceCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  auto outer_service = node->create_service<test_msgs::srv::Empty>(empty_service_name, callback);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto service = node->create_service<test_msgs::srv::Empty>(empty_service_name, callback);
    state.ResumeTiming();
    benchmark::DoNotOptimize(service);
    benchmark::ClobberMemory();

    service.reset();
  }
}

BENCHMARK_F(ServicePerformanceTest, async_send_response)(benchmark::State & state) {
  auto callback = std::bind(
    &ServicePerformanceTest::ServiceCallback,
    this, std::placeholders::_1, std::placeholders::_2);
  auto service = node->create_service<test_msgs::srv::Empty>(empty_service_name, callback);

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    // Clear executor queue
    rclcpp::spin_some(node->get_node_base_interface());

    auto request = std::make_shared<test_msgs::srv::Empty::Request>();
    auto future = empty_client->async_send_request(request);
    state.ResumeTiming();
    benchmark::DoNotOptimize(service);
    benchmark::ClobberMemory();

    rclcpp::spin_until_future_complete(node->get_node_base_interface(), future);
  }
  if (callback_count == 0) {
    state.SkipWithError("Service callback was not called");
  }
}
