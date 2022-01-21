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
BENCHMARK_F(PerformanceTest, construct_destruct_transition)(benchmark::State & state)
{
  for (auto _ : state) {
    (void)_;
    rclcpp_lifecycle::Transition transition(1, "transition");
    benchmark::DoNotOptimize(transition);
    benchmark::ClobberMemory();
  }
}

BENCHMARK_F(PerformanceTest, copy_destruct_transition)(benchmark::State & state)
{
  rclcpp_lifecycle::Transition transition(1, "transition");
  for (auto _ : state) {
    (void)_;
    rclcpp_lifecycle::Transition transition_copy(transition);
    benchmark::DoNotOptimize(transition_copy);
    benchmark::ClobberMemory();
  }
}
