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

  using ExecAllocTraits = allocator::AllocRebind<executor::AnyExecutable, Alloc>;
  using ExecAlloc = typename ExecAllocTraits::allocator_type;
  using ExecDeleter = allocator::Deleter<ExecAlloc, executor::AnyExecutable>;
  using VoidAllocTraits = typename allocator::AllocRebind<void *, Alloc>;
  using VoidAlloc = typename VoidAllocTraits::allocator_type;

  explicit AllocatorMemoryStrategy(std::shared_ptr<Alloc> allocator)
  {
    executable_allocator_ = std::make_shared<ExecAlloc>(*allocator.get());
    allocator_ = std::make_shared<VoidAlloc>(*allocator.get());
  }

  AllocatorMemoryStrategy()
  {
    executable_allocator_ = std::make_shared<ExecAlloc>();
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
    subscription_handles_.clear();
    service_handles_.clear();
    client_handles_.clear();
    timer_handles_.clear();
  }

  virtual void remove_null_handles(rcl_wait_set_t * wait_set)
  {
    for (size_t i = 0; i < wait_set->size_of_subscriptions; ++i) {
      if (!wait_set->subscriptions[i]) {
        subscription_handles_[i] = nullptr;
      }
    }
    for (size_t i = 0; i < wait_set->size_of_services; ++i) {
      if (!wait_set->services[i]) {
        service_handles_[i] = nullptr;
      }
    }
    for (size_t i = 0; i < wait_set->size_of_clients; ++i) {
      if (!wait_set->clients[i]) {
        client_handles_[i] = nullptr;
      }
    }
    for (size_t i = 0; i < wait_set->size_of_timers; ++i) {
      if (!wait_set->timers[i]) {
        timer_handles_[i] = nullptr;
      }
    }

    subscription_handles_.erase(
      std::remove(subscription_handles_.begin(), subscription_handles_.end(), nullptr),
      subscription_handles_.end()
    );

    service_handles_.erase(
      std::remove(service_handles_.begin(), service_handles_.end(), nullptr),
      service_handles_.end()
    );

    client_handles_.erase(
      std::remove(client_handles_.begin(), client_handles_.end(), nullptr),
      client_handles_.end()
    );

    timer_handles_.erase(
      std::remove(timer_handles_.begin(), timer_handles_.end(), nullptr),
      timer_handles_.end()
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
        for (auto & weak_subscription : group->get_subscription_ptrs()) {
          auto subscription = weak_subscription.lock();
          if (subscription) {
            subscription_handles_.push_back(subscription->get_subscription_handle());
            if (subscription->get_intra_process_subscription_handle()) {
              subscription_handles_.push_back(
                subscription->get_intra_process_subscription_handle());
            }
          }
        }
        for (auto & weak_service : group->get_service_ptrs()) {
          auto service = weak_service.lock();
          if (service) {
            service_handles_.push_back(service->get_service_handle());
          }
        }
        for (auto & weak_client : group->get_client_ptrs()) {
          auto client = weak_client.lock();
          if (client) {
            client_handles_.push_back(client->get_client_handle());
          }
        }
        for (auto & weak_timer : group->get_timer_ptrs()) {
          auto timer = weak_timer.lock();
          if (timer) {
            timer_handles_.push_back(timer->get_timer_handle());
          }
        }
      }
    }
    return has_invalid_weak_nodes;
  }

  bool add_handles_to_waitset(rcl_wait_set_t * wait_set)
  {
    for (auto subscription : subscription_handles_) {
      if (rcl_wait_set_add_subscription(wait_set, subscription) != RCL_RET_OK) {
        fprintf(stderr, "Couldn't add subscription to waitset: %s\n", rcl_get_error_string_safe());
        return false;
      }
    }

    for (auto client : client_handles_) {
      if (rcl_wait_set_add_client(wait_set, client) != RCL_RET_OK) {
        fprintf(stderr, "Couldn't add client to waitset: %s\n", rcl_get_error_string_safe());
        return false;
      }
    }

    for (auto service : service_handles_) {
      if (rcl_wait_set_add_service(wait_set, service) != RCL_RET_OK) {
        fprintf(stderr, "Couldn't add service to waitset: %s\n", rcl_get_error_string_safe());
        return false;
      }
    }

    for (auto timer : timer_handles_) {
      if (rcl_wait_set_add_timer(wait_set, timer) != RCL_RET_OK) {
        fprintf(stderr, "Couldn't add timer to waitset: %s\n", rcl_get_error_string_safe());
        return false;
      }
    }

    for (auto guard_condition : guard_conditions_) {
      if (rcl_wait_set_add_guard_condition(wait_set, guard_condition) != RCL_RET_OK) {
        fprintf(stderr, "Couldn't add guard_condition to waitset: %s\n",
          rcl_get_error_string_safe());
        return false;
      }
    }
    return true;
  }

  /// Provide a newly initialized AnyExecutable object.
  // \return Shared pointer to the fresh executable.
  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    return std::allocate_shared<executor::AnyExecutable>(*executable_allocator_.get());
  }

  virtual void
  get_next_subscription(executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes)
  {
    auto it = subscription_handles_.begin();
    while (it != subscription_handles_.end()) {
      auto subscription = get_subscription_by_handle(*it, weak_nodes);
      if (subscription) {
        // Figure out if this is for intra-process or not.
        bool is_intra_process = false;
        if (subscription->get_intra_process_subscription_handle()) {
          is_intra_process = subscription->get_intra_process_subscription_handle() == *it;
        }
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_subscription(subscription, weak_nodes);
        if (!group) {
          // Group was not found, meaning the subscription is not valid...
          // Remove it from the ready list and continue looking
          subscription_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        if (is_intra_process) {
          any_exec->subscription_intra_process = subscription;
        } else {
          any_exec->subscription = subscription;
        }
        any_exec->callback_group = group;
        any_exec->node_base = get_node_by_group(group, weak_nodes);
        subscription_handles_.erase(it);
        return;
      }
      // Else, the subscription is no longer valid, remove it and continue
      subscription_handles_.erase(it);
    }
  }

  virtual void
  get_next_service(executor::AnyExecutable::SharedPtr any_exec,
    const WeakNodeVector & weak_nodes)
  {
    auto it = service_handles_.begin();
    while (it != service_handles_.end()) {
      auto service = get_service_by_handle(*it, weak_nodes);
      if (service) {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_service(service, weak_nodes);
        if (!group) {
          // Group was not found, meaning the service is not valid...
          // Remove it from the ready list and continue looking
          service_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->service = service;
        any_exec->callback_group = group;
        any_exec->node_base = get_node_by_group(group, weak_nodes);
        service_handles_.erase(it);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      service_handles_.erase(it);
    }
  }

  virtual void
  get_next_client(executor::AnyExecutable::SharedPtr any_exec, const WeakNodeVector & weak_nodes)
  {
    auto it = client_handles_.begin();
    while (it != client_handles_.end()) {
      auto client = get_client_by_handle(*it, weak_nodes);
      if (client) {
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_client(client, weak_nodes);
        if (!group) {
          // Group was not found, meaning the service is not valid...
          // Remove it from the ready list and continue looking
          client_handles_.erase(it);
          continue;
        }
        if (!group->can_be_taken_from().load()) {
          // Group is mutually exclusive and is being used, so skip it for now
          // Leave it to be checked next time, but continue searching
          ++it;
          continue;
        }
        // Otherwise it is safe to set and return the any_exec
        any_exec->client = client;
        any_exec->callback_group = group;
        any_exec->node_base = get_node_by_group(group, weak_nodes);
        client_handles_.erase(it);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      client_handles_.erase(it);
    }
  }

  virtual rcl_allocator_t get_allocator()
  {
    return rclcpp::allocator::get_rcl_allocator<void *, VoidAlloc>(*allocator_.get());
  }

  size_t number_of_ready_subscriptions() const
  {
    return subscription_handles_.size();
  }

  size_t number_of_ready_services() const
  {
    return service_handles_.size();
  }

  size_t number_of_ready_clients() const
  {
    return client_handles_.size();
  }

  size_t number_of_guard_conditions() const
  {
    return guard_conditions_.size();
  }

  size_t number_of_ready_timers() const
  {
    return timer_handles_.size();
  }

private:
  template<typename T>
  using VectorRebind =
      std::vector<T, typename std::allocator_traits<Alloc>::template rebind_alloc<T>>;

  VectorRebind<const rcl_guard_condition_t *> guard_conditions_;

  VectorRebind<const rcl_subscription_t *> subscription_handles_;
  VectorRebind<const rcl_service_t *> service_handles_;
  VectorRebind<const rcl_client_t *> client_handles_;
  VectorRebind<const rcl_timer_t *> timer_handles_;

  std::shared_ptr<ExecAlloc> executable_allocator_;
  std::shared_ptr<VoidAlloc> allocator_;
};

}  // namespace allocator_memory_strategy
}  // namespace memory_strategies
}  // namespace rclcpp

#endif  // RCLCPP__STRATEGIES__ALLOCATOR_MEMORY_STRATEGY_HPP_
