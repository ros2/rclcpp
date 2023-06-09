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

#ifndef RCLCPP__MEMORY_STRATEGIES_HPP_
#define RCLCPP__MEMORY_STRATEGIES_HPP_

#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace memory_strategies
{

/// Create a MemoryStrategy sharedPtr
/**
 * \return a MemoryStrategy sharedPtr
 */
RCLCPP_PUBLIC
memory_strategy::MemoryStrategy::SharedPtr
create_default_strategy();

}  // namespace memory_strategies
}  // namespace rclcpp

#endif  // RCLCPP__MEMORY_STRATEGIES_HPP_
