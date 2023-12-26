// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__GUARD_CONDITION_OPTIONS_HPP_
#define RCLCPP__GUARD_CONDITION_OPTIONS_HPP_

#include <memory>

#include "rcl/guard_condition.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/guard_condition.hpp"

namespace rclcpp
{

template<typename Allocator>
struct GuardConditionWithAllocator
{
  /// Optional custom allocator
  std::shared_ptr<Allocator> allocator = nullptr;

  GuardConditionWithAllocator() {}

  /// Convert this class into a rcl_guard_condition_options_t
  rcl_guard_condition_options_t
  to_rcl_guard_condition_options() const
  {
    rcl_guard_condition_options_t result = rcl_guard_condition_get_default_options();
    result.allocator = this->get_rcl_allocator();
    return result;
  }

  /// Get the allocator, creating one if needed
  std::shared_ptr<Allocator>
  get_allocator() const
  {
    if (!this->allocator) {
      if (!allocator_storage_) {
        allocator_storage_ = std::make_shared<Allocator>();
      }
      return allocator_storage_;
    }
    return this->allocator;
  }

private:
  using PlainAllocator =
    typename std::allocator_traits<Allocator>::template rebind_alloc<char>;

  rcl_allocator_t
  get_rcl_allocator() const
  {
    if (!plain_allocator_storage_) {
      plain_allocator_storage_ =
        std::make_shared<PlainAllocator>(*this->get_allocator());
    }
    return rclcpp::allocator::get_rcl_allocator<char>(*plain_allocator_storage_);
  }

  // always returns a copy of the same allocator
  mutable std::shared_ptr<Allocator> allocator_storage_;

  // always returns a copy of the same allocator
  mutable std::shared_ptr<PlainAllocator> plain_allocator_storage_;
};

using GuardConditionOptions =
  GuardConditionWithAllocator<std::allocator<void>>;

}  // namespace rclcpp

#endif  // RCLCPP__GUARD_CONDITION_OPTIONS_HPP_
