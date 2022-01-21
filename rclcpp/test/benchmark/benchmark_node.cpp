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

using performance_test_fixture::PerformanceTest;

class NodePerformanceTest : public PerformanceTest
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

BENCHMARK_F(NodePerformanceTest, create_node)(benchmark::State & state)
{
  // Warmup and prime caches
  auto outer_node = std::make_shared<rclcpp::Node>("node");
  outer_node.reset();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    // Using pointer to separate construction and destruction in timing
    auto node = std::make_shared<rclcpp::Node>("node");
#ifndef __clang_analyzer__
    benchmark::DoNotOptimize(node);
#endif
    benchmark::ClobberMemory();

    // Ensure destruction of node is not counted toward timing
    state.PauseTiming();
    node.reset();
    state.ResumeTiming();
  }
}

BENCHMARK_F(NodePerformanceTest, destroy_node)(benchmark::State & state)
{
  // Warmup and prime caches
  auto outer_node = std::make_shared<rclcpp::Node>("node");
  outer_node.reset();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    // Using pointer to separate construction and destruction in timing
    state.PauseTiming();
    auto node = std::make_shared<rclcpp::Node>("node");
    state.ResumeTiming();

#ifndef __clang_analyzer__
    benchmark::DoNotOptimize(node);
#endif
    benchmark::ClobberMemory();

    node.reset();
  }
}
