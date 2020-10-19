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
#include <vector>

#include "performance_test_fixture/performance_test_fixture.hpp"
#include "rclcpp/rclcpp.hpp"

using performance_test_fixture::PerformanceTest;

class NodePerformanceTest : public PerformanceTest
{
public:
  void SetUp(::benchmark::State & state)
  {
    rclcpp::init(0, nullptr);
    performance_test_fixture::PerformanceTest::SetUp(state);
  }

  void TearDown(::benchmark::State & state)
  {
    rclcpp::shutdown();
    performance_test_fixture::PerformanceTest::TearDown(state);
  }
};

BENCHMARK_F(NodePerformanceTest, create_node)(benchmark::State & state)
{
  std::vector<std::unique_ptr<rclcpp::Node>> nodes(10000);
  for (auto _ : state) {
    auto node = std::make_unique<rclcpp::Node>("node");
    benchmark::DoNotOptimize(node);
    benchmark::ClobberMemory();

    // Ensure destruction of node is not counted toward timing
    state.PauseTiming();
    node.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(NodePerformanceTest, destroy_node)(benchmark::State & state)
{
  for (auto _ : state) {
    state.PauseTiming();
    auto node = std::make_unique<rclcpp::Node>("node");
    state.ResumeTiming();

    benchmark::DoNotOptimize(node);
    benchmark::ClobberMemory();

    node.reset();
  }
}
