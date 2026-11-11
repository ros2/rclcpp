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

#include "rclcpp/memory_strategies.hpp"

#include <algorithm>
#include <list>
#include <memory>

#include "rclcpp/strategies/allocator_memory_strategy.hpp"

using rclcpp::memory_strategies::allocator_memory_strategy::AllocatorMemoryStrategy;

namespace
{

/// Keeps the handles that were ready while their callback group was in use.
/**
 * These handles are not waited on again, they are executed first, in the order in which
 * they were blocked, as soon as their callback group can be taken from.
 */
class DefaultMemoryStrategy : public AllocatorMemoryStrategy<>
{
public:
  void clear_handles() override
  {
    if (waited_) {
      // The handles that are left were ready, but their callback group was in use.
      keep_blocked(EntityType::Timer, timer_handles_);
      keep_blocked(EntityType::Subscription, subscription_handles_);
      keep_blocked(EntityType::Service, service_handles_);
      keep_blocked(EntityType::Client, client_handles_);
      keep_blocked(EntityType::Waitable, waitable_handles_);
      waited_ = false;
    }
    AllocatorMemoryStrategy<>::clear_handles();
  }

  void remove_null_handles(rcl_wait_set_t * wait_set) override
  {
    AllocatorMemoryStrategy<>::remove_null_handles(wait_set);
    // A handle that is already blocked must not be executed a second time.
    for (const auto & blocked : blocked_handles_) {
      auto handle = blocked.handle.lock();
      remove_handle(timer_handles_, handle);
      remove_handle(subscription_handles_, handle);
      remove_handle(service_handles_, handle);
      remove_handle(client_handles_, handle);
      remove_handle(waitable_handles_, handle);
    }
    waited_ = true;
  }

  void
  get_next_timer(
    rclcpp::AnyExecutable & any_exec,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) override
  {
    if (restore_blocked(EntityType::Timer, weak_groups_to_nodes)) {
      AllocatorMemoryStrategy<>::get_next_timer(any_exec, weak_groups_to_nodes);
    }
  }

  void
  get_next_subscription(
    rclcpp::AnyExecutable & any_exec,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) override
  {
    if (restore_blocked(EntityType::Subscription, weak_groups_to_nodes)) {
      AllocatorMemoryStrategy<>::get_next_subscription(any_exec, weak_groups_to_nodes);
    }
  }

  void
  get_next_service(
    rclcpp::AnyExecutable & any_exec,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) override
  {
    if (restore_blocked(EntityType::Service, weak_groups_to_nodes)) {
      AllocatorMemoryStrategy<>::get_next_service(any_exec, weak_groups_to_nodes);
    }
  }

  void
  get_next_client(
    rclcpp::AnyExecutable & any_exec,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) override
  {
    if (restore_blocked(EntityType::Client, weak_groups_to_nodes)) {
      AllocatorMemoryStrategy<>::get_next_client(any_exec, weak_groups_to_nodes);
    }
  }

  void
  get_next_waitable(
    rclcpp::AnyExecutable & any_exec,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes) override
  {
    if (restore_blocked(EntityType::Waitable, weak_groups_to_nodes)) {
      AllocatorMemoryStrategy<>::get_next_waitable(any_exec, weak_groups_to_nodes);
    }
  }

private:
  enum class EntityType {Timer, Subscription, Service, Client, Waitable};

  struct BlockedHandle
  {
    EntityType type;
    std::weak_ptr<const void> handle;
  };

  template<typename HandlesT>
  void keep_blocked(EntityType type, const HandlesT & handles)
  {
    for (const auto & handle : handles) {
      blocked_handles_.push_back({type, handle});
    }
  }

  template<typename HandlesT>
  static void remove_handle(HandlesT & handles, const std::shared_ptr<const void> & handle)
  {
    handles.erase(std::remove(handles.begin(), handles.end(), handle), handles.end());
  }

  rclcpp::CallbackGroup::SharedPtr
  get_group(
    const BlockedHandle & blocked,
    const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
  {
    auto handle = blocked.handle.lock();
    if (!handle) {
      return nullptr;
    }
    switch (blocked.type) {
      case EntityType::Timer:
        {
          auto timer = get_timer_by_handle(
            std::static_pointer_cast<const rcl_timer_t>(handle), weak_groups_to_nodes);
          // The timer may have been canceled or reset in the meantime.
          if (!timer || !timer->is_ready()) {
            return nullptr;
          }
          return get_group_by_timer(timer, weak_groups_to_nodes);
        }
      case EntityType::Subscription:
        return get_group_by_subscription(
          get_subscription_by_handle(
            std::static_pointer_cast<const rcl_subscription_t>(handle), weak_groups_to_nodes),
          weak_groups_to_nodes);
      case EntityType::Service:
        return get_group_by_service(
          get_service_by_handle(
            std::static_pointer_cast<const rcl_service_t>(handle), weak_groups_to_nodes),
          weak_groups_to_nodes);
      case EntityType::Client:
        return get_group_by_client(
          get_client_by_handle(
            std::static_pointer_cast<const rcl_client_t>(handle), weak_groups_to_nodes),
          weak_groups_to_nodes);
      case EntityType::Waitable:
        return get_group_by_waitable(to_waitable(handle), weak_groups_to_nodes);
    }
    return nullptr;
  }

  static rclcpp::Waitable::SharedPtr to_waitable(const std::shared_ptr<const void> & handle)
  {
    return std::const_pointer_cast<rclcpp::Waitable>(
      std::static_pointer_cast<const rclcpp::Waitable>(handle));
  }

  /// Put the oldest blocked handle whose callback group can be taken from back in front.
  /**
   * \return false if that handle is of another type, nothing of this type may be taken then.
   */
  bool
  restore_blocked(EntityType type, const WeakCallbackGroupsToNodesMap & weak_groups_to_nodes)
  {
    for (auto it = blocked_handles_.begin(); it != blocked_handles_.end(); ) {
      auto group = get_group(*it, weak_groups_to_nodes);
      if (!group) {
        it = blocked_handles_.erase(it);
        continue;
      }
      if (!group->can_be_taken_from().load()) {
        ++it;
        continue;
      }
      if (it->type != type) {
        return false;
      }
      auto handle = it->handle.lock();
      switch (type) {
        case EntityType::Timer:
          timer_handles_.insert(
            timer_handles_.begin(), std::static_pointer_cast<const rcl_timer_t>(handle));
          break;
        case EntityType::Subscription:
          subscription_handles_.insert(
            subscription_handles_.begin(),
            std::static_pointer_cast<const rcl_subscription_t>(handle));
          break;
        case EntityType::Service:
          service_handles_.insert(
            service_handles_.begin(), std::static_pointer_cast<const rcl_service_t>(handle));
          break;
        case EntityType::Client:
          client_handles_.insert(
            client_handles_.begin(), std::static_pointer_cast<const rcl_client_t>(handle));
          break;
        case EntityType::Waitable:
          waitable_handles_.insert(waitable_handles_.begin(), to_waitable(handle));
          break;
      }
      blocked_handles_.erase(it);
      return true;
    }
    return true;
  }

  std::list<BlockedHandle> blocked_handles_;
  bool waited_ = false;
};

}  // namespace

rclcpp::memory_strategy::MemoryStrategy::SharedPtr
rclcpp::memory_strategies::create_default_strategy()
{
  return std::make_shared<DefaultMemoryStrategy>();
}
