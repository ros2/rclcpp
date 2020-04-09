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

#include <memory>
#include <stdexcept>
#include <utility>

#include "rcl/wait.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

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
    class SubscriptionsIterable,
    class GuardConditionsIterable,
    class ExtraGuardConditionsIterable,
    class TimersIterable,
    class ClientsIterable,
    class ServicesIterable,
    class WaitablesIterable
  >
  explicit
  StoragePolicyCommon(
    const SubscriptionsIterable & subscriptions,
    const GuardConditionsIterable & guard_conditions,
    const ExtraGuardConditionsIterable & extra_guard_conditions,
    const TimersIterable & timers,
    const ClientsIterable & clients,
    const ServicesIterable & services,
    const WaitablesIterable & waitables,
    rclcpp::Context::SharedPtr context
  )
  : rcl_wait_set_(rcl_get_zero_initialized_wait_set()), context_(context)
  {
    // Check context is not nullptr.
    if (nullptr == context) {
      throw std::invalid_argument("context is nullptr");
    }
    // Accumulate total contributions from waitables.
    size_t subscriptions_from_waitables = 0;
    size_t guard_conditions_from_waitables = 0;
    size_t timers_from_waitables = 0;
    size_t clients_from_waitables = 0;
    size_t services_from_waitables = 0;
    size_t events_from_waitables = 0;
    for (const auto & waitable_entry : waitables) {
      rclcpp::Waitable & waitable = *waitable_entry.waitable.get();
      subscriptions_from_waitables += waitable.get_number_of_ready_subscriptions();
      guard_conditions_from_waitables += waitable.get_number_of_ready_guard_conditions();
      timers_from_waitables += waitable.get_number_of_ready_timers();
      clients_from_waitables += waitable.get_number_of_ready_clients();
      services_from_waitables += waitable.get_number_of_ready_services();
      events_from_waitables += waitable.get_number_of_ready_events();
    }
    // Initialize wait set using initial inputs.
    rcl_ret_t ret = rcl_wait_set_init(
      &rcl_wait_set_,
      subscriptions.size() + subscriptions_from_waitables,
      guard_conditions.size() + extra_guard_conditions.size() + guard_conditions_from_waitables,
      timers.size() + timers_from_waitables,
      clients.size() + clients_from_waitables,
      services.size() + services_from_waitables,
      events_from_waitables,
      context_->get_rcl_context().get(),
      // TODO(wjwwood): support custom allocator, maybe restrict to polymorphic allocator
      rcl_get_default_allocator());
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }

    // (Re)build the wait set for the first time.
    this->storage_rebuild_rcl_wait_set_with_sets(
      subscriptions,
      guard_conditions,
      extra_guard_conditions,
      timers,
      clients,
      services,
      waitables);
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
    class SubscriptionsIterable,
    class GuardConditionsIterable,
    class ExtraGuardConditionsIterable,
    class TimersIterable,
    class ClientsIterable,
    class ServicesIterable,
    class WaitablesIterable
  >
  void
  storage_rebuild_rcl_wait_set_with_sets(
    const SubscriptionsIterable & subscriptions,
    const GuardConditionsIterable & guard_conditions,
    const ExtraGuardConditionsIterable & extra_guard_conditions,
    const TimersIterable & timers,
    const ClientsIterable & clients,
    const ServicesIterable & services,
    const WaitablesIterable & waitables
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
      // Accumulate total contributions from waitables.
      size_t subscriptions_from_waitables = 0;
      size_t guard_conditions_from_waitables = 0;
      size_t timers_from_waitables = 0;
      size_t clients_from_waitables = 0;
      size_t services_from_waitables = 0;
      size_t events_from_waitables = 0;
      for (const auto & waitable_entry : waitables) {
        auto waitable_ptr_pair = get_raw_pointer_from_smart_pointer(waitable_entry.waitable);
        if (nullptr == waitable_ptr_pair.second) {
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
        rclcpp::Waitable & waitable = *waitable_ptr_pair.second;
        subscriptions_from_waitables += waitable.get_number_of_ready_subscriptions();
        guard_conditions_from_waitables += waitable.get_number_of_ready_guard_conditions();
        timers_from_waitables += waitable.get_number_of_ready_timers();
        clients_from_waitables += waitable.get_number_of_ready_clients();
        services_from_waitables += waitable.get_number_of_ready_services();
        events_from_waitables += waitable.get_number_of_ready_events();
      }
      rcl_ret_t ret = rcl_wait_set_resize(
        &rcl_wait_set_,
        subscriptions.size() + subscriptions_from_waitables,
        guard_conditions.size() + extra_guard_conditions.size() + guard_conditions_from_waitables,
        timers.size() + timers_from_waitables,
        clients.size() + clients_from_waitables,
        services.size() + services_from_waitables,
        events_from_waitables
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

    // Add subscriptions.
    for (const auto & subscription_entry : subscriptions) {
      auto subscription_ptr_pair =
        get_raw_pointer_from_smart_pointer(subscription_entry.subscription);
      if (nullptr == subscription_ptr_pair.second) {
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
      rcl_ret_t ret = rcl_wait_set_add_subscription(
        &rcl_wait_set_,
        subscription_ptr_pair.second->get_subscription_handle().get(),
        nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // Setup common code to add guard_conditions.
    auto add_guard_conditions =
      [this](const auto & inner_guard_conditions)
      {
        for (const auto & guard_condition : inner_guard_conditions) {
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
            &guard_condition_ptr_pair.second->get_rcl_guard_condition(),
            nullptr);
          if (RCL_RET_OK != ret) {
            rclcpp::exceptions::throw_from_rcl_error(ret);
          }
        }
      };

    // Add guard conditions.
    add_guard_conditions(guard_conditions);

    // Add extra guard conditions.
    add_guard_conditions(extra_guard_conditions);

    // Add timers.
    for (const auto & timer : timers) {
      auto timer_ptr_pair = get_raw_pointer_from_smart_pointer(timer);
      if (nullptr == timer_ptr_pair.second) {
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
      rcl_ret_t ret = rcl_wait_set_add_timer(
        &rcl_wait_set_,
        timer_ptr_pair.second->get_timer_handle().get(),
        nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // Add clients.
    for (const auto & client : clients) {
      auto client_ptr_pair = get_raw_pointer_from_smart_pointer(client);
      if (nullptr == client_ptr_pair.second) {
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
      rcl_ret_t ret = rcl_wait_set_add_client(
        &rcl_wait_set_,
        client_ptr_pair.second->get_client_handle().get(),
        nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // Add services.
    for (const auto & service : services) {
      auto service_ptr_pair = get_raw_pointer_from_smart_pointer(service);
      if (nullptr == service_ptr_pair.second) {
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
      rcl_ret_t ret = rcl_wait_set_add_service(
        &rcl_wait_set_,
        service_ptr_pair.second->get_service_handle().get(),
        nullptr);
      if (RCL_RET_OK != ret) {
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    }

    // Add waitables.
    for (auto & waitable_entry : waitables) {
      auto waitable_ptr_pair = get_raw_pointer_from_smart_pointer(waitable_entry.waitable);
      if (nullptr == waitable_ptr_pair.second) {
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
      rclcpp::Waitable & waitable = *waitable_ptr_pair.second;
      bool successful = waitable.add_to_wait_set(&rcl_wait_set_);
      if (!successful) {
        throw std::runtime_error("waitable unexpectedly failed to be added to wait set");
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
