// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_

#include <algorithm>
#include <array>
#include <memory>

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set_policies/detail/storage_policy_common.hpp"

namespace rclcpp
{
namespace wait_set_policies
{

/// WaitSet policy that provides dynamically sized storage.
class DynamicStorage : public rclcpp::wait_set_policies::detail::StoragePolicyCommon<false>
{
protected:
  using is_mutable = std::true_type;

  using SequenceOfWeakGuardConditions = std::vector<std::weak_ptr<rclcpp::GuardCondition>>;
  using GuardConditionsIterable = std::vector<std::shared_ptr<rclcpp::GuardCondition>>;

  explicit
  DynamicStorage(
    const GuardConditionsIterable & guard_conditions,
    rclcpp::Context::SharedPtr context
  )
  : StoragePolicyCommon(guard_conditions, context),
    guard_conditions_(guard_conditions.cbegin(), guard_conditions.cend()),
    shared_guard_conditions_(guard_conditions_.size())
  {}

  ~DynamicStorage() = default;

  void
  storage_rebuild_rcl_wait_set()
  {
    this->storage_rebuild_rcl_wait_set_with_sets(
      guard_conditions_
    );
  }

  bool
  storage_has_guard_condition(const rclcpp::GuardCondition & guard_condition)
  {
    return std::any_of(
      guard_conditions_.cbegin(),
      guard_conditions_.cend(),
      [&guard_condition](const auto & gc) { return &guard_condition == gc.lock().get(); });
  }

  auto
  storage_find_guard_condition(const rclcpp::GuardCondition & guard_condition)
  {
    return std::find_if(
      guard_conditions_.begin(),
      guard_conditions_.end(),
      [&guard_condition](const auto & gc) { return &guard_condition == gc.lock().get(); });
  }

  void
  storage_add_guard_condition(std::shared_ptr<rclcpp::GuardCondition> && guard_condition)
  {
    if (this->storage_has_guard_condition(*guard_condition)) {
      throw std::runtime_error("guard_condition already in wait set");
    }
    guard_conditions_.push_back(std::move(guard_condition));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_guard_condition(std::shared_ptr<rclcpp::GuardCondition> && guard_condition)
  {
    auto it = this->storage_find_guard_condition(*guard_condition);
    if (guard_conditions_.cend() == it) {
      throw std::runtime_error("guard_condition not in wait set");
    }
    guard_conditions_.erase(it);
    this->storage_flag_for_resize();
  }

  // this is noexcept because:
  //   - std::weak_ptr::expired is noexcept
  //   - the erase-remove idiom is noexcept, since we're not using the ExecutionPolicy version
  //   - std::vector::erase is noexcept if the assignment operator of T is also
  //     - and, the operator= for std::weak_ptr is noexcept
  void
  storage_prune_deleted_entities() noexcept
  {
    // reusable (templated) lambda for removal predicate
    auto p =
      [](const auto & weak_ptr) {
        // remove entries which have expired
        return weak_ptr.expired();
      };
    // remove guard conditions which have been deleted
    guard_conditions_.erase(std::remove_if(guard_conditions_.begin(), guard_conditions_.end(), p));
  }

  void
  storage_acquire_ownerships()
  {
    if (++ownership_reference_counter_ > 1) {
      // Avoid redundant locking.
      return;
    }
    // Setup common locking function.
    auto lock_all = [](const auto & weak_ptrs, auto & shared_ptrs) {
      shared_ptrs.resize(weak_ptrs.size());
      size_t index = 0;
      for (const auto & weak_ptr : weak_ptrs) {
        shared_ptrs[index++] = weak_ptr.lock();
      }
    };
    // Lock all the weak pointers and hold them until released.
    lock_all(guard_conditions_, shared_guard_conditions_);
  }

  void
  storage_release_ownerships()
  {
    if (--ownership_reference_counter_ > 0) {
      // Avoid releasing ownership until reference count is 0.
      return;
    }
    // "Unlock" all shared pointers by resetting them.
    auto reset_all = [](auto & shared_ptrs) {
      for (auto & shared_ptr : shared_ptrs) {
        shared_ptr.reset();
      }
    };
    reset_all(shared_guard_conditions_);
  }

  size_t ownership_reference_counter_ = 0;
  SequenceOfWeakGuardConditions guard_conditions_;
  GuardConditionsIterable shared_guard_conditions_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_
