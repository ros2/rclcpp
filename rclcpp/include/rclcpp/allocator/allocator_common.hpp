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

template<typename T>
using clean_t = std::remove_cv_t<std::remove_reference_t<T>>;

// Primary template: false
template<typename, typename = std::void_t<>>
struct has_get_rcl_allocator : std::false_type {};

// Specialization: true if expression is valid
template<typename T>
struct has_get_rcl_allocator<T,
  std::void_t<
    decltype(std::declval<clean_t<T> &>().get_rcl_allocator())
  >
>
  : std::bool_constant<
    std::is_same_v<
      decltype(std::declval<clean_t<T> &>().get_rcl_allocator()),
      rcl_allocator_t
    >
  >
{};

// Helper variable template
template<typename T>
inline constexpr bool has_get_rcl_allocator_v =
  has_get_rcl_allocator<T>::value;

template<typename T, typename Alloc>
using AllocRebind = typename std::allocator_traits<Alloc>::template rebind_traits<T>;

// TODO(jacquelinekay) Workaround for an incomplete implementation of std::allocator<void>
template<
  typename T,
  typename Alloc,
  typename std::enable_if<std::is_same<Alloc, std::allocator<void>>::value>::type * = nullptr>
rcl_allocator_t get_rcl_allocator(Alloc & allocator)
{
  (void)allocator;
  return rcl_get_default_allocator();
}

}  // namespace allocator
}  // namespace rclcpp

#endif  // RCLCPP__ALLOCATOR__ALLOCATOR_COMMON_HPP_
