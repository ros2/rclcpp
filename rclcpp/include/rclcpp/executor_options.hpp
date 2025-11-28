// Copyright 2014-2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__EXECUTOR_OPTIONS_HPP_
#define RCLCPP__EXECUTOR_OPTIONS_HPP_

#include <memory>

#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/memory_strategies.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{

class ExecutorOptionsImplementation;

/// Options to be passed to the executor constructor.
struct ExecutorOptions
{
  RCLCPP_PUBLIC
  ExecutorOptions();

  RCLCPP_PUBLIC
  virtual ~ExecutorOptions();

  RCLCPP_PUBLIC
  ExecutorOptions(const ExecutorOptions &);

  RCLCPP_PUBLIC
  ExecutorOptions & operator=(const ExecutorOptions &);

  rclcpp::memory_strategy::MemoryStrategy::SharedPtr memory_strategy;
  rclcpp::Context::SharedPtr context;
  size_t max_conditions;

private:
  /// Pointer to implementation
  std::unique_ptr<ExecutorOptionsImplementation> impl_;
};

}  // namespace rclcpp

#endif  // RCLCPP__EXECUTOR_OPTIONS_HPP_
