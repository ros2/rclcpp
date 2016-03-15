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

// Convert a std::allocator_traits-formatted Allocator into an rcl allocator
template<typename T, typename Alloc, typename std::enable_if<!std::is_same<Alloc, std::allocator<void>>::value>::type * = nullptr>
rcl_allocator_t get_rcl_allocator(Alloc & allocator)
{
  rcl_allocator_t rcl_allocator = rcl_get_default_allocator();

  // Argh
  auto allocate_callback = std::function<void *(size_t, void *)>(
    [](size_t size, void * state) -> void * {
      auto allocator_ptr = static_cast<Alloc *>(state);
      // TODO check if null
      return std::allocator_traits<Alloc>::allocate(*allocator_ptr, size);
    }).target<void *(size_t, void *)>();
  auto deallocate_callback = std::function<void(void *, void *)>(
    [](void * pointer, void * state) {
      auto allocator_ptr = static_cast<Alloc *>(state);
      // TODO check if null
      // TODO size?
      auto typed_pointer = static_cast<T *>(pointer);
      std::allocator_traits<Alloc>::deallocate(*allocator_ptr, typed_pointer, sizeof(T));
    }).target<void(void *, void *)>();

  rcl_allocator.allocate = allocate_callback;
  rcl_allocator.deallocate = deallocate_callback;
  rcl_allocator.reallocate = nullptr;
  rcl_allocator.state = &allocator;
  return rcl_allocator;
}

// TODO(jacquelinekay) Workaround for an incomplete implementation of std::allocator<void>
template<typename T, typename Alloc, typename std::enable_if<std::is_same<Alloc, std::allocator<void>>::value>::type * = nullptr>
rcl_allocator_t get_rcl_allocator(Alloc & allocator)
{
  (void)allocator;
  return rcl_get_default_allocator();
}


// TODO provide a nicer way of making a C++ allocator maybe?
// or passing rcl allocators directly?

}  // namespace allocator
}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_
