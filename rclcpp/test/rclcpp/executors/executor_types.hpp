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

// suppress deprecated StaticSingleThreadedExecutor warning
// we define an alias that explicitly indicates that this class is deprecated, while avoiding
// polluting a lot of files the gcc pragmas
#if !defined(_WIN32)
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#else  // !defined(_WIN32)
# pragma warning(push)
# pragma warning(disable: 4996)
#endif
using DeprecatedStaticSingleThreadedExecutor = rclcpp::executors::StaticSingleThreadedExecutor;
// remove warning suppression
#if !defined(_WIN32)
# pragma GCC diagnostic pop
#else  // !defined(_WIN32)
# pragma warning(pop)
#endif

#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
using ExecutorTypes =
  ::testing::Types<
  rclcpp::executors::SingleThreadedExecutor,
  rclcpp::executors::MultiThreadedExecutor,
  DeprecatedStaticSingleThreadedExecutor,
  rclcpp::experimental::executors::EventsExecutor>;
#ifdef __clang__
# pragma clang diagnostic pop
#endif

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
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
    if (std::is_same<T, DeprecatedStaticSingleThreadedExecutor>()) {
      return "StaticSingleThreadedExecutor";
    }
#ifdef __clang__
# pragma clang diagnostic pop
#endif

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
