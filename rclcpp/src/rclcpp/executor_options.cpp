// Copyright 2024 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/executor_options.hpp"

using rclcpp::ExecutorOptions;

namespace rclcpp
{

class ExecutorOptionsImplementation {};

}  // namespace rclcpp

ExecutorOptions::ExecutorOptions()
: memory_strategy(rclcpp::memory_strategies::create_default_strategy()),
  context(rclcpp::contexts::get_global_default_context()),
  max_conditions(0),
  impl_(nullptr)
{}

ExecutorOptions::~ExecutorOptions()
{}

ExecutorOptions::ExecutorOptions(const ExecutorOptions & other)
{
  *this = other;
}

ExecutorOptions & ExecutorOptions::operator=(const ExecutorOptions & other)
{
  if (this == &other) {
    return *this;
  }

  this->memory_strategy = other.memory_strategy;
  this->context = other.context;
  this->max_conditions = other.max_conditions;
  if (nullptr != other.impl_) {
    this->impl_ = std::make_unique<ExecutorOptionsImplementation>(*other.impl_);
  }

  return *this;
}
