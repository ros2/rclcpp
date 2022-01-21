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
#include "rclcpp/rclcpp.hpp"
#include "test_msgs/srv/empty.hpp"

using performance_test_fixture::PerformanceTest;

constexpr char empty_service_name[] = "empty_service";

class ClientPerformanceTest : public PerformanceTest
{
public:
  explicit ClientPerformanceTest(rclcpp::NodeOptions = rclcpp::NodeOptions()) {}
  void SetUp(benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    node = std::make_unique<rclcpp::Node>("node", "ns");

    auto empty_service_callback =
      [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};
    empty_service =
      node->create_service<test_msgs::srv::Empty>(empty_service_name, empty_service_callback);

    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(benchmark::State & state)
  {
    performance_test_fixture::PerformanceTest::TearDown(state);
    empty_service.reset();
    node.reset();
    rclcpp::shutdown();
  }

protected:
  std::unique_ptr<rclcpp::Node> node;
  std::shared_ptr<rclcpp::Service<test_msgs::srv::Empty>> empty_service;
};

BENCHMARK_F(ClientPerformanceTest, construct_client_no_service)(benchmark::State & state) {
  // Prime cache
  auto outer_client = node->create_client<test_msgs::srv::Empty>("not_an_existing_service");
  outer_client.reset();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto client = node->create_client<test_msgs::srv::Empty>("not_an_existing_service");
    benchmark::DoNotOptimize(client);
    benchmark::ClobberMemory();

    state.PauseTiming();
    client.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ClientPerformanceTest, construct_client_empty_srv)(benchmark::State & state) {
  // Prime cache
  auto outer_client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
  outer_client.reset();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
    benchmark::DoNotOptimize(client);
    benchmark::ClobberMemory();

    state.PauseTiming();
    client.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(ClientPerformanceTest, destroy_client_empty_srv)(benchmark::State & state) {
  // Prime cache
  auto outer_client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
  outer_client.reset();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    auto client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
    state.ResumeTiming();
    benchmark::DoNotOptimize(client);
    benchmark::ClobberMemory();

    client.reset();
  }
}

BENCHMARK_F(ClientPerformanceTest, wait_for_service)(benchmark::State & state) {
  int count = 0;
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    const std::string service_name = std::string("service_") + std::to_string(count++);
    // Create client before service so it has to 'discover' the service after construction
    auto client = node->create_client<test_msgs::srv::Empty>(service_name);
    auto callback =
      [](
      const test_msgs::srv::Empty::Request::SharedPtr,
      test_msgs::srv::Empty::Response::SharedPtr) {};
    auto service =
      node->create_service<test_msgs::srv::Empty>(service_name, callback);
    state.ResumeTiming();

    client->wait_for_service(std::chrono::seconds(1));
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(ClientPerformanceTest, async_send_request_only)(benchmark::State & state) {
  auto client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
  auto shared_request = std::make_shared<test_msgs::srv::Empty::Request>();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto future = client->async_send_request(shared_request);
    benchmark::DoNotOptimize(future);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(ClientPerformanceTest, async_send_request_and_response)(benchmark::State & state) {
  auto client = node->create_client<test_msgs::srv::Empty>(empty_service_name);
  auto shared_request = std::make_shared<test_msgs::srv::Empty::Request>();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    auto future = client->async_send_request(shared_request);
    rclcpp::spin_until_future_complete(
      node->get_node_base_interface(), future, std::chrono::seconds(1));
    benchmark::DoNotOptimize(future);
    benchmark::ClobberMemory();
  }
}
