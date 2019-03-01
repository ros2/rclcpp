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

#ifndef RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_
#define RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_

#include <memory>
#include <vector>

#include "rcl/allocator.h"

#include "rclcpp/allocator/allocator_common.hpp"
#include "rclcpp/memory_strategy.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/visibility_control.hpp"

#include "rcutils/logging_macros.h"

#include "rmw/types.h"

namespace rclcpp
{
namespace memory_strategies
{
namespace allocator_memory_strategy
{

/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
template<typename Alloc = std::allocator<void>>
class AllocatorMemoryStrategy : public memory_strategy::MemoryStrategy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(AllocatorMemoryStrategy<Alloc>)

  using VoidAllocTraits = typename allocator::AllocRebind<void *, Alloc>;
  using VoidAlloc = typename VoidAllocTraits::allocator_type;

  explicit AllocatorMemoryStrategy(std::shared_ptr<Alloc> allocator)
  {
    allocator_ = std::make_shared<VoidAlloc>(*allocator.get());
  }

  AllocatorMemoryStrategy()
  {
    allocator_ = std::make_shared<VoidAlloc>();
  }

  void add_guard_condition(const rcl_guard_condition_t * guard_condition)
  {
    for (const auto & existing_guard_condition : guard_conditions_) {
      if (existing_guard_condition == guard_condition) {
        return;
      }
    }
    guard_conditions_.push_back(guard_condition);
  }

  void remove_guard_condition(const rcl_guard_condition_t * guard_condition)
  {
    for (auto it = guard_conditions_.begin(); it != guard_conditions_.end(); ++it) {
      if (*it == guard_condition) {
        guard_conditions_.erase(it);
        break;
      }
    }
  }

  void clear_handles()
  {
    waitable_handles_.clear();
  }

  virtual void remove_null_handles(rcl_wait_set_t * wait_set)
  {
    // TODO(jacobperron): Check if wait set sizes are what we expect them to be?
    //                    e.g. wait_set->size_of_clients == client_handles_.size()

    for (size_t i = 0; i < waitable_handles_.size(); ++i) {
      if (!waitable_handles_[i]->is_ready(wait_set)) {
        waitable_handles_[i].reset();
      }
    }

    waitable_handles_.erase(
      std::remove(waitable_handles_.begin(), waitable_handles_.end(), nullptr),
      waitable_handles_.end()
    );
  }

  bool collect_entities(const WeakNodeVector & weak_nodes)
  {
    bool has_invalid_weak_nodes = false;
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        has_invalid_weak_nodes = true;
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group || !group->can_be_taken_from().load()) {
          continue;
        }

        for (auto & weak_publisher : group->get_publisher_ptrs()) {
          auto publisher = weak_publisher.lock();
          if (publisher) {
            waitable_handles_.push_back(publisher);
          }
        }
        for (auto & weak_subscription : group->get_subscription_ptrs()) {
          auto subscription = weak_subscription.lock();
          if (subscription) {
            waitable_handles_.push_back(subscription);
          }
        }
        for (auto & weak_service : group->get_service_ptrs()) {
          auto service = weak_service.lock();
          if (service) {
            waitable_handles_.push_back(service);
          }
        }
        for (auto & weak_client : group->get_client_ptrs()) {
          auto client = weak_client.lock();
          if (client) {
            waitable_handles_.push_back(client);
          }
        }
        for (auto & weak_timer : group->get_timer_ptrs()) {
          auto timer = weak_timer.lock();
          if (timer) {
            waitable_handles_.push_back(timer);
          }
        }
        for (auto & weak_waitable : group->get_waitable_ptrs()) {
          auto waitable = weak_waitable.lock();
          if (waitable) {
            waitable_handles_.push_back(waitable);
          }
        }
      }
    }
    return has_invalid_weak_nodes;
  }

  bool add_handles_to_wait_set(rcl_wait_set_t * wait_set)
  {
    for (auto guard_condition : guard_conditions_) {
      if (rcl_wait_set_add_guard_condition(wait_set, guard_condition, NULL) != RCL_RET_OK) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "Couldn't add guard_condition to wait set: %s",
          rcl_get_error_string().str);
        return false;
      }
    }

    for (auto waitable : waitable_handles_) {
      if (!waitable->add_to_wait_set(wait_set)) {
        RCUTILS_LOG_ERROR_NAMED(
          "rclcpp",
          "Couldn't add waitable to wait set: %s", rcl_get_error_string().str);
        return false;
      }
    }
    return true;
  }

  virtual void
  get_next_waitable(executor::AnyExecutable & any_exec, const WeakNodeVector & weak_nodes)
  {
    auto it = waitable_handles_.begin();
    while (it != waitable_handles_.end()) {
      auto waitable = *it;
      if (waitable) {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_waitable(waitable, weak_nodes);
        if (!group) {
          // Group was not found, meaning the waitable is not valid...
          // Remove it from the ready list and continue looking
          it = waitable_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec.waitable = waitable;
        any_exec.callback_group = group;
        any_exec.node_base = get_node_by_group(group, weak_nodes);
        waitable_handles_.erase(it);
        return;
      }
      // Else, the waitable is no longer valid, remove it and continue
      it = waitable_handles_.erase(it);
    }
  }

  virtual rcl_allocator_t get_allocator()
  {
    return rclcpp::allocator::get_rcl_allocator<void *, VoidAlloc>(*allocator_.get());
  }

  size_t number_of_ready_subscriptions() const
  {
    size_t number_of_subscriptions = 0;
    for (auto waitable : waitable_handles_) {
      number_of_subscriptions += waitable->get_number_of_ready_subscriptions();
    }
    return number_of_subscriptions;
  }

  size_t number_of_ready_services() const
  {
    size_t number_of_services = 0;
    for (auto waitable : waitable_handles_) {
      number_of_services += waitable->get_number_of_ready_services();
    }
    return number_of_services;
  }

  size_t number_of_ready_events() const
  {
    size_t number_of_events = 0;
    for (auto waitable : waitable_handles_) {
      number_of_events += waitable->get_number_of_ready_events();
    }
    return number_of_events;
  }

  size_t number_of_ready_clients() const
  {
    size_t number_of_clients = 0;
    for (auto waitable : waitable_handles_) {
      number_of_clients += waitable->get_number_of_ready_clients();
    }
    return number_of_clients;
  }

  size_t number_of_guard_conditions() const
  {
    size_t number_of_guard_conditions = guard_conditions_.size();
    for (auto waitable : waitable_handles_) {
      number_of_guard_conditions += waitable->get_number_of_ready_guard_conditions();
    }
    return number_of_guard_conditions;
  }

  size_t number_of_ready_timers() const
  {
    size_t number_of_timers = 0;
    for (auto waitable : waitable_handles_) {
      number_of_timers += waitable->get_number_of_ready_timers();
    }
    return number_of_timers;
  }

  size_t number_of_waitables() const
  {
    return waitable_handles_.size();
  }

private:
  template<typename T>
  using VectorRebind =
    std::vector<T, typename std::allocator_traits<Alloc>::template rebind_alloc<T>>;

  VectorRebind<const rcl_guard_condition_t *> guard_conditions_;
  VectorRebind<std::shared_ptr<Waitable>> waitable_handles_;

  std::shared_ptr<VoidAlloc> allocator_;
};

}  // namespace allocator_memory_strategy
}  // namespace memory_strategies
}  // namespace rclcpp

#endif  // RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_
