// Copyright 2014 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP_RCLCPP_MEMORY_STRATEGIES_HPP_
#define RCLCPP_RCLCPP_MEMORY_STRATEGIES_HPP_

#include <rclcpp/strategies/dynamic_memory_strategy.hpp>
#include <rclcpp/strategies/static_memory_strategy.hpp>

namespace rclcpp
{
namespace memory_strategies
{

typedef std::shared_ptr<memory_strategy::MemoryStrategy> MemoryStrategySharedPtr;

using rclcpp::memory_strategies::dynamic_memory_strategy::DynamicMemoryStrategy;
using rclcpp::memory_strategies::static_memory_strategy::StaticMemoryStrategy;

MemoryStrategySharedPtr create_default_strategy()
{
  return std::make_shared<DynamicMemoryStrategy>(DynamicMemoryStrategy());
}

}
}

#endif
