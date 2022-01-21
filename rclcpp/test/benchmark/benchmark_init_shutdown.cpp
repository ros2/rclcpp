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

#include "performance_test_fixture/performance_test_fixture.hpp"

#include "rclcpp/rclcpp.hpp"

using performance_test_fixture::PerformanceTest;

BENCHMARK_F(PerformanceTest, rclcpp_init)(benchmark::State & state)
{
  // Warmup and prime caches
  rclcpp::init(0, nullptr);
  rclcpp::shutdown();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    rclcpp::init(0, nullptr);

    state.PauseTiming();
    rclcpp::shutdown();
    state.ResumeTiming();
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(PerformanceTest, rclcpp_shutdown)(benchmark::State & state)
{
  // Warmup and prime caches
  rclcpp::init(0, nullptr);
  rclcpp::shutdown();

  reset_heap_counters();
  for (auto _ : state) {
    (void)_;
    state.PauseTiming();
    rclcpp::init(0, nullptr);
    state.ResumeTiming();

    rclcpp::shutdown();
    benchmark::ClobberMemory();
  }
}
