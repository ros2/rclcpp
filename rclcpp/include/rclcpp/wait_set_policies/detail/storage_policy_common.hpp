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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__STORAGE_POLICY_COMMON_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__STORAGE_POLICY_COMMON_HPP_

#include <array>
#include <memory>

#include "rcl/wait.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// Common structure for storage policies, which provides rcl wait set access.
template<bool HasStrongOwnership>
class StoragePolicyCommon
{
protected:
  template<
    // class SubscriptionsIterable,
    class GuardConditionsIterable
    // class ServicesIterable,
    // class ClientsIterable,
    // class TimersIterable,
    // class EventsIterable,
    // class WaitablesIterable
  >
  explicit
  StoragePolicyCommon(
    const GuardConditionsIterable & guard_conditions,
    rclcpp::Context::SharedPtr context
  )
  : rcl_wait_set_(rcl_get_zero_initialized_wait_set()), context_(context)
  {
    // Check context is not nullptr.
    if (nullptr == context) {
      throw std::invalid_argument("context is nullptr");
    }
    // Initialize wait set using initial inputs.
    rcl_ret_t ret = rcl_wait_set_init(
      &rcl_wait_set_,
      0,  // subs
      guard_conditions.size(),
      0,  // timers
      0,  // clients
      0,  // services
      0,  // events
      context_->get_rcl_context().get(),
      // TODO(wjwwood): support custom allocator, maybe restrict to polymorphic allocator
      rcl_get_default_allocator());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    // (Re)build the wait set for the first time.
    this->storage_rebuild_rcl_wait_set_with_sets(guard_conditions);
  }

  ~StoragePolicyCommon()
  {
    rcl_ret_t ret = rcl_wait_set_fini(&rcl_wait_set_);
    if (RCL_RET_OK != ret) {
      try {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      } catch (const std::exception & exception) {
        RCLCPP_ERROR(
          rclcpp::get_logger("rclcpp"),
          "Error in destruction of rcl wait set: %s", exception.what());
      }
    }
  }

  template<class EntityT>
  std::pair<void *, EntityT *>
  get_raw_pointer_from_smart_pointer(const std::shared_ptr<EntityT> & shared_pointer)
  {
    return {nullptr, shared_pointer.get()};
  }

  template<class EntityT>
  std::pair<std::shared_ptr<EntityT>, EntityT *>
  get_raw_pointer_from_smart_pointer(const std::weak_ptr<EntityT> & weak_pointer)
  {
    auto shared_pointer = weak_pointer.lock();
    return {shared_pointer, shared_pointer.get()};
  }

  /// Rebuild the wait set, preparing it for the next wait call.
  /**
   * The wait set is rebuilt by:
   *
   *   - resizing the wait set if needed,
   *   - clearing the wait set if not already done by resizing, and
   *   - re-adding the entities.
   */
  template<
    // class SubscriptionsIterable,
    class GuardConditionsIterable
    // class ServicesIterable,
    // class ClientsIterable,
    // class TimersIterable,
    // class EventsIterable,
    // class WaitablesIterable
  >
  void
  storage_rebuild_rcl_wait_set_with_sets(
    const GuardConditionsIterable & guard_conditions
  )
  {
    bool was_resized = false;
    // Resize the wait set, but only if it needs to be.
    if (needs_resize_) {
      // Resizing with rcl_wait_set_resize() is a no-op if nothing has changed,
      // but tracking the need to resize in this class avoids an unnecessary
      // library call (rcl is most likely a separate shared library) each wait
      // loop.
      // Also, since static storage wait sets will never need resizing, so it
      // avoids completely redundant calls to this function in that case.
      rcl_ret_t ret = rcl_wait_set_resize(
        &rcl_wait_set_,
        0,  // subscriptions_size
        guard_conditions.size(),
        0,  // timers_size
        0,  // clients_size
        0,  // services_size
        0  // events_size
      );
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
      was_resized = true;
      // Assumption: the calling code ensures this function is not called
      // concurrently with functions that set this variable to true, either
      // with documentation (as is the case for the SequentialSychronization
      // policy), or with synchronization primitives (as is the case with
      // the ThreadSafeSynchronization policy).
      needs_resize_ = false;
    }

    // Now clear the wait set, but only if it was not resized, as resizing also
    // clears the wait set.
    if (!was_resized) {
      rcl_ret_t ret = rcl_wait_set_clear(&rcl_wait_set_);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // Add guard conditions.
    for (const auto & guard_condition : guard_conditions) {
      auto guard_condition_ptr_pair = get_raw_pointer_from_smart_pointer(guard_condition);
      if (nullptr == guard_condition_ptr_pair.second) {
        // In this case it was probably stored as a weak_ptr, but is now locking to nullptr.
        if (HasStrongOwnership) {
          // This will not happen in fixed sized storage, as it holds
          // shared ownership the whole time and is never in need of pruning.
          throw std::runtime_error("unexpected condition, fixed storage policy needs pruning");
        }
        // Flag for pruning.
        needs_pruning_ = true;
        continue;
      }
      rcl_ret_t ret = rcl_wait_set_add_guard_condition(
        &rcl_wait_set_,
        &guard_condition_ptr_pair.second->get_rcl_guard_condtion(),
        nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }
  }

  const rcl_wait_set_t &
  storage_get_rcl_wait_set() const
  {
    return rcl_wait_set_;
  }

  rcl_wait_set_t &
  storage_get_rcl_wait_set()
  {
    return rcl_wait_set_;
  }

  void
  storage_flag_for_resize()
  {
    needs_resize_ = true;
  }

  rcl_wait_set_t rcl_wait_set_;
  rclcpp::Context::SharedPtr context_;

  bool needs_pruning_ = false;
  bool needs_resize_ = false;
};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__STORAGE_POLICY_COMMON_HPP_
