// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTORS__EXECUTOR_TYPES_HPP_
#define RCLCPP__EXECUTORS__EXECUTOR_TYPES_HPP_

#include <gtest/gtest.h>

#include <string>
#include <type_traits>

#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/executors/static_single_threaded_executor.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

using ExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::executors::StaticSingleThreadedExecutor,
  rclcpp::experimental::executors::EventsExecutor>;

class ExecutorTypeNames
{
public:
  template<typename T>
  static std::string GetName(int idx)
  {
    (void)idx;
    if (std::is_same<T, rclcpp::executors::SingleThreadedExecutor>()) {
      return "SingleThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::executors::MultiThreadedExecutor>()) {
      return "MultiThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::executors::StaticSingleThreadedExecutor>()) {
      return "StaticSingleThreadedExecutor";
    }

    if (std::is_same<T, rclcpp::experimental::executors::EventsExecutor>()) {
      return "EventsExecutor";
    }

    return "";
  }
};

// StaticSingleThreadedExecutor is not included in these tests for now, due to:
// https://github.com/ros2/rclcpp/issues/1219
using StandardExecutors =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  rclcpp::experimental::executors::EventsExecutor>;

#endif  // RCLCPP__EXECUTORS__EXECUTOR_TYPES_HPP_
