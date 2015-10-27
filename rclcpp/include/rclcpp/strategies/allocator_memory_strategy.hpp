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

#ifndef RCLCPP_RCLCPP_ALLOCATOR_MEMORY_STRATEGY_HPP_
#define RCLCPP_RCLCPP_ALLOCATOR_MEMORY_STRATEGY_HPP_

#include <memory>
#include <vector>

#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/memory_strategy.hpp>

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
  RCLCPP_SMART_PTR_DEFINITIONS(AllocatorMemoryStrategy<Alloc>);

  using WeakNode = std::weak_ptr<rclcpp::node::Node>>;
  using NodeVector = std::vector<WeakNode>;

  using ExecAllocTraits = allocator::AllocRebind<executor::AnyExecutable, Alloc>;
  using ExecAlloc = typename ExecAllocTraits::allocator_type;
  using ExecDeleter = allocator::Deleter<ExecAlloc, executor::AnyExecutable>;
  using VoidAllocTraits = typename allocator::AllocRebind<void *, Alloc>;
  using VoidAlloc = typename VoidAllocTraits::allocator_type;

  AllocatorMemoryStrategy(std::shared_ptr<Alloc> allocator)
  {
    executable_allocator_ = std::make_shared<ExecAlloc>(*allocator.get());
    allocator_ = std::make_shared<VoidAlloc>(*allocator.get());
  }

  AllocatorMemoryStrategy()
  {
    executable_allocator_ = std::make_shared<ExecAlloc>();
    allocator_ = std::make_shared<VoidAlloc>();
  }

  size_t fill_subscriber_handles(void ** ptr) {
    size_t max_size = subscriptions_.size();
    if (subscriber_handles_.size() < max_size) {
      subscriber_handles_.resize(max_size);
    }
    size_t count = 0;
    for (auto & subscription : subscriptions_) {
      subscriber_handles_[count++] = subscription->get_subscription_handle()->data;
      if (subscription->get_intra_process_subscription_handle()) {
        subscriber_handles_[count++] =
          subscription->get_intra_process_subscription_handle()->data;
      }
    }
    ptr = static_cast<void**>(subscriber_handles_.data());
    return count;
  }

  // return the new number of services
  size_t fill_service_handles(void ** ptr) {
    size_t max_size = services_.size();
    if (service_handles_.size() < max_size) {
      service_handles_.resize(max_size);
    }
    size_t count = 0;
    for (auto & service : services_) {
      service_handles_[count++] = service->get_service_handle()->data;
    }
    ptr = static_cast<void**>(service_handles_.data());
    return count;
  }

  // return the new number of clients
  size_t fill_client_handles(void ** ptr) {
    size_t max_size = clients_.size();
    if (client_handles_.size() < max_size) {
      client_handles_.resize(max_size);
    }
    size_t count = 0;
    for (auto & client : clients_) {
      client_handles_[count++] = client->get_client_handle()->data;
    }
    ptr = static_cast<void**>(client_handles_.data());
    return count;
  }

  void clear_active_entities() {
    subscriptions_.clear();
    services_.clear();
    clients_.clear();
  }

  void clear_handles() {
    subscriber_handles_.clear();
    service_handles_.clear();
    client_handles_.clear();
  }

  bool collect_entities(const NodeVector & weak_nodes) {
    bool has_invalid_weak_nodes = false;
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        has_invalid_weak_nodes = false;
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
            subscriptions_.push_back(subscription);
          }
        }
        for (auto & service : group->get_service_ptrs()) {
          services_.push_back(service);
        }
        for (auto & client : group->get_client_ptrs()) {
          clients_.push_back(client);
        }
      }
    }
    return has_invalid_weak_nodes;
  }


  /// Provide a newly initialized AnyExecutable object.
  // \return Shared pointer to the fresh executable.
  executor::AnyExecutable::SharedPtr instantiate_next_executable()
  {
    return std::allocate_shared<executor::AnyExecutable>(*executable_allocator_.get());
  }

  static rclcpp::subscription::SubscriptionBase::SharedPtr
  get_subscription_by_handle(void * subscriber_handle,
    const NodeVector & weak_nodes)
  {
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto & weak_subscription : group->get_subscription_ptrs()) {
          auto subscription = weak_subscription.lock();
          if (subscription) {
            if (subscription->get_subscription_handle()->data == subscriber_handle) {
              return subscription;
            }
            if (subscription->get_intra_process_subscription_handle() &&
              subscription->get_intra_process_subscription_handle()->data == subscriber_handle)
            {
              return subscription;
            }
          }
        }
      }
    }
    return nullptr;
  }

  static rclcpp::service::ServiceBase::SharedPtr
  get_service_by_handle(void * service_handle,
    const NodeVector & weak_nodes)
  {
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto & service : group->get_service_ptrs()) {
          if (service->get_service_handle()->data == service_handle) {
            return service;
          }
        }
      }
    }
    return rclcpp::service::ServiceBase::SharedPtr();
  }

  static rclcpp::client::ClientBase::SharedPtr
  get_client_by_handle(void * client_handle,
    const NodeVector & weak_nodes)
  {
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto & client : group->get_client_ptrs()) {
          if (client->get_client_handle()->data == client_handle) {
            return client;
          }
        }
      }
    }
    return rclcpp::client::ClientBase::SharedPtr();
  }

  static rclcpp::node::Node::SharedPtr
  get_node_by_group(rclcpp::callback_group::CallbackGroup::SharedPtr group,
    const NodeVector & weak_nodes)
  {
    if (!group) {
      return rclcpp::node::Node::SharedPtr();
    }
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto callback_group = weak_group.lock();
        if (callback_group == group) {
          return node;
        }
      }
    }
    return rclcpp::node::Node::SharedPtr();
  }

  static rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_subscription(
    rclcpp::subscription::SubscriptionBase::SharedPtr subscription,
    const NodeVector & weak_nodes)
  {
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto & weak_sub : group->get_subscription_ptrs()) {
          auto sub = weak_sub.lock();
          if (sub == subscription) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }



  virtual void
  get_next_subscription(executor::AnyExecutable::SharedPtr any_exec,
    const NodeVector & weak_nodes)
  {
    auto it = subscriber_handles_.begin();
    while (it != subscriber_handles_.end()) {
      auto subscription = get_subscription_by_handle(*it, weak_nodes);
      if (subscription) {
        // Figure out if this is for intra-process or not.
        bool is_intra_process = false;
        if (subscription->get_intra_process_subscription_handle()) {
          is_intra_process = subscription->get_intra_process_subscription_handle()->data == *it;
        }
        // Find the group for this handle and see if it can be serviced
        auto group = get_group_by_subscription(subscription, weak_nodes);
        if (!group) {
          // Group was not found, meaning the subscription is not valid...
          // Remove it from the ready list and continue looking
          subscriber_handles_.erase(it++);
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
        any_exec->node = get_node_by_group(group, weak_nodes);
        subscriber_handles_.erase(it++);
        return;
      }
      // Else, the subscription is no longer valid, remove it and continue
      subscriber_handles_.erase(it++);
    }
  }

  static rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_service(
    rclcpp::service::ServiceBase::SharedPtr service,
    const NodeVector & weak_nodes)
  {
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto & serv : group->get_service_ptrs()) {
          if (serv == service) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  virtual void
  get_next_service(executor::AnyExecutable::SharedPtr any_exec,
    const NodeVector & weak_nodes)
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
          service_handles_.erase(it++);
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
        any_exec->node = get_node_by_group(group, weak_nodes);
        service_handles_.erase(it++);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      service_handles_.erase(it++);
    }
  }

  static rclcpp::callback_group::CallbackGroup::SharedPtr
  get_group_by_client(
    rclcpp::client::ClientBase::SharedPtr client,
    const NodeVector & weak_nodes)
  {
    for (auto & weak_node : weak_nodes) {
      auto node = weak_node.lock();
      if (!node) {
        continue;
      }
      for (auto & weak_group : node->get_callback_groups()) {
        auto group = weak_group.lock();
        if (!group) {
          continue;
        }
        for (auto & cli : group->get_client_ptrs()) {
          if (cli == client) {
            return group;
          }
        }
      }
    }
    return rclcpp::callback_group::CallbackGroup::SharedPtr();
  }

  virtual void
  get_next_client(executor::AnyExecutable::SharedPtr any_exec,
    const NodeVector & weak_nodes)
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
          client_handles_.erase(it++);
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
        any_exec->node = get_node_by_group(group, weak_nodes);
        client_handles_.erase(it++);
        return;
      }
      // Else, the service is no longer valid, remove it and continue
      client_handles_.erase(it++);
    }
  }



private:
  template<typename U>
  using VectorRebind = typename std::allocator_traits<Alloc>::template rebind_alloc<U>;

  std::vector<rclcpp::subscription::SubscriptionBase::SharedPtr,
  VectorRebind<rclcpp::subscription::SubscriptionBase::SharedPtr>> subscriptions_;
  std::vector<rclcpp::service::ServiceBase::SharedPtr,
  VectorRebind<rclcpp::service::ServiceBase::SharedPtr>> services_;
  std::vector<rclcpp::client::ClientBase::SharedPtr,
  VectorRebind<rclcpp::client::ClientBase::SharedPtr>> clients_;

  std::vector<void *, VoidAlloc> subscriber_handles_;
  std::vector<void *, VoidAlloc> service_handles_;
  std::vector<void *, VoidAlloc> client_handles_;

  std::shared_ptr<ExecAlloc> executable_allocator_;
  std::shared_ptr<VoidAlloc> allocator_;
};

}  /* allocator_memory_strategy */
}  /* memory_strategies */

}  /* rclcpp */

#endif
