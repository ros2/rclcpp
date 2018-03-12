// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/memory_strategies.hpp"

#include <memory>

#include "rclcpp/strategies/allocator_memory_strategy.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

rclcpp::memory_strategy::MemoryStrategy::SharedPtr
rclcpp::memory_strategies::create_default_strategy()
{
  return std::make_shared<AllocatorMemoryStrategy<>>();
}
