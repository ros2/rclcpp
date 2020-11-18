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
#include "rclcpp_lifecycle/transition.hpp"

using PerformanceTest = performance_test_fixture::PerformanceTest;

// These are relatively quick benchmarks, so it would be difficult to separate construction from
// destruction
BENCHMARK_F(PerformanceTest, construct_destruct_state)(benchmark::State & state)
{
  for (auto _ : state) {
    rclcpp_lifecycle::State lifecycle_state(1, "state");
    benchmark::DoNotOptimize(lifecycle_state);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(PerformanceTest, copy_destruct_state)(benchmark::State & state)
{
  rclcpp_lifecycle::State lifecycle_state(1, "state");
  for (auto _ : state) {
    rclcpp_lifecycle::State state_copy(lifecycle_state);
    benchmark::DoNotOptimize(state_copy);
    benchmark::ClobberMemory();
  }
}
