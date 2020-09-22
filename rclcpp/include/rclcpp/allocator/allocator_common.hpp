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

template<typename Alloc>
void * retyped_allocate(size_t size, void * untyped_allocator)
{
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  return std::allocator_traits<Alloc>::allocate(*typed_allocator, size);
}

template<typename Alloc>
void retyped_deallocate(void * untyped_pointer, void * untyped_allocator)
{
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  auto typed_ptr = static_cast<typename Alloc::value_type *>(untyped_pointer);
  std::allocator_traits<Alloc>::deallocate(*typed_allocator, typed_ptr, 1);
}

template<typename Alloc>
void * retyped_reallocate(void * untyped_pointer, size_t size, void * untyped_allocator)
{
  auto typed_allocator = static_cast<Alloc *>(untyped_allocator);
  if (!typed_allocator) {
    throw std::runtime_error("Received incorrect allocator type");
  }
  auto typed_ptr = static_cast<typename Alloc::value_type *>(untyped_pointer);
  std::allocator_traits<Alloc>::deallocate(*typed_allocator, typed_ptr, 1);
  return std::allocator_traits<Alloc>::allocate(*typed_allocator, size);
}

}  // namespace allocator

// Convert a std::allocator_traits-formatted Allocator into an rcl allocator
template<typename Alloc>
rcl_allocator_t get_rcl_allocator(Alloc & allocator)
{
  rcl_allocator_t rcl_allocator = rcl_get_default_allocator();
#ifndef _WIN32
  rcl_allocator.allocate = &allocator::retyped_allocate<Alloc>;
  rcl_allocator.deallocate = &allocator::retyped_deallocate<Alloc>;
  rcl_allocator.reallocate = &allocator::retyped_reallocate<Alloc>;
  rcl_allocator.state = &allocator;
#else
  (void)allocator;  // Remove warning
#endif
  return rcl_allocator;
}

template <typename T>
rcl_allocator_t get_rcl_allocator(std::allocator<T>& allocator)
{
  (void)allocator;  // Remove warning
  return rcl_get_default_allocator();
}

}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_
