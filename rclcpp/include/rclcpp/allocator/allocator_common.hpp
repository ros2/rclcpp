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

#ifndef RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_
#define RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_

#include <memory>

#include "rcl/allocator.h"

#include "rclcpp/allocator/allocator_deleter.hpp"

namespace rclcpp
{
namespace allocator
{

template<typename T, typename Alloc>
using AllocRebind = typename std::allocator_traits<Alloc>::template rebind_traits<T>;

}  // namespace allocator

/// Return the equivalent rcl_allocator_t for the C++ standard allocator.
/**
 * This assumes that the user intent behind both allocators is the
 * same: Using system malloc for allocation.
 *
 * If you're using a custom allocator in ROS, you'll need to provide
 * your own overload for this function.
 */
template<typename T>
rcl_allocator_t get_rcl_allocator(std::allocator<T> allocator)
{
  (void)allocator;  // Remove warning
  return rcl_get_default_allocator();
}

}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_
