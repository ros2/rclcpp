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
#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/client.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_set_policies/detail/storage_policy_common.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace wait_set_policies
{

/// WaitSet policy that provides dynamically sized storage.
class DynamicStorage : public rclcpp::wait_set_policies::detail::StoragePolicyCommon<false>
{
protected:
  using is_mutable = std::true_type;

  class SubscriptionEntry
  {
    // (wjwwood): indent of 'public:' is weird, I know. uncrustify is dumb.

public:
    std::shared_ptr<rclcpp::SubscriptionBase> subscription;
    rclcpp::SubscriptionWaitSetMask mask;

    /// Conversion constructor, which is intentionally not marked explicit.
    SubscriptionEntry(
      std::shared_ptr<rclcpp::SubscriptionBase> subscription_in = nullptr,
      const rclcpp::SubscriptionWaitSetMask & mask_in = {})
    : subscription(std::move(subscription_in)),
      mask(mask_in)
    {}

    void
    reset() noexcept
    {
      subscription.reset();
    }
  };
  class WeakSubscriptionEntry
  {
public:
    std::weak_ptr<rclcpp::SubscriptionBase> subscription;
    rclcpp::SubscriptionWaitSetMask mask;

    explicit WeakSubscriptionEntry(
      const std::shared_ptr<rclcpp::SubscriptionBase> & subscription_in,
      const rclcpp::SubscriptionWaitSetMask & mask_in) noexcept
    : subscription(subscription_in),
      mask(mask_in)
    {}

    explicit WeakSubscriptionEntry(const SubscriptionEntry & other)
    : subscription(other.subscription),
      mask(other.mask)
    {}

    std::shared_ptr<rclcpp::SubscriptionBase>
    lock() const
    {
      return subscription.lock();
    }

    bool
    expired() const noexcept
    {
      return subscription.expired();
    }
  };
  using SequenceOfWeakSubscriptions = std::vector<WeakSubscriptionEntry>;
  using SubscriptionsIterable = std::vector<SubscriptionEntry>;

  using SequenceOfWeakGuardConditions = std::vector<std::weak_ptr<rclcpp::GuardCondition>>;
  using GuardConditionsIterable = std::vector<std::shared_ptr<rclcpp::GuardCondition>>;

  using SequenceOfWeakTimers = std::vector<std::weak_ptr<rclcpp::TimerBase>>;
  using TimersIterable = std::vector<std::shared_ptr<rclcpp::TimerBase>>;

  using SequenceOfWeakClients = std::vector<std::weak_ptr<rclcpp::ClientBase>>;
  using ClientsIterable = std::vector<std::shared_ptr<rclcpp::ClientBase>>;

  using SequenceOfWeakServices = std::vector<std::weak_ptr<rclcpp::ServiceBase>>;
  using ServicesIterable = std::vector<std::shared_ptr<rclcpp::ServiceBase>>;

  class WaitableEntry
  {
public:
    std::shared_ptr<rclcpp::Waitable> waitable;
    std::shared_ptr<void> associated_entity;

    /// Conversion constructor, which is intentionally not marked explicit.
    WaitableEntry(
      std::shared_ptr<rclcpp::Waitable> waitable_in = nullptr,
      std::shared_ptr<void> associated_entity_in = nullptr) noexcept
    : waitable(std::move(waitable_in)),
      associated_entity(std::move(associated_entity_in))
    {}

    void
    reset() noexcept
    {
      waitable.reset();
      associated_entity.reset();
    }
  };
  class WeakWaitableEntry
  {
public:
    std::weak_ptr<rclcpp::Waitable> waitable;
    std::weak_ptr<void> associated_entity;

    explicit WeakWaitableEntry(
      const std::shared_ptr<rclcpp::Waitable> & waitable_in,
      const std::shared_ptr<void> & associated_entity_in) noexcept
    : waitable(waitable_in),
      associated_entity(associated_entity_in)
    {}

    explicit WeakWaitableEntry(const WaitableEntry & other)
    : waitable(other.waitable),
      associated_entity(other.associated_entity)
    {}

    std::shared_ptr<rclcpp::Waitable>
    lock() const
    {
      return waitable.lock();
    }

    bool
    expired() const noexcept
    {
      return waitable.expired();
    }
  };
  using SequenceOfWeakWaitables = std::vector<WeakWaitableEntry>;
  using WaitablesIterable = std::vector<WaitableEntry>;

  template<class ArrayOfExtraGuardConditions>
  explicit
  DynamicStorage(
    const SubscriptionsIterable & subscriptions,
    const GuardConditionsIterable & guard_conditions,
    const ArrayOfExtraGuardConditions & extra_guard_conditions,
    const TimersIterable & timers,
    const ClientsIterable & clients,
    const ServicesIterable & services,
    const WaitablesIterable & waitables,
    rclcpp::Context::SharedPtr context
  )
  : StoragePolicyCommon(
      subscriptions,
      guard_conditions,
      extra_guard_conditions,
      timers,
      clients,
      services,
      waitables,
      context),
    subscriptions_(subscriptions.cbegin(), subscriptions.cend()),
    shared_subscriptions_(subscriptions_.size()),
    guard_conditions_(guard_conditions.cbegin(), guard_conditions.cend()),
    shared_guard_conditions_(guard_conditions_.size()),
    timers_(timers.cbegin(), timers.cend()),
    shared_timers_(timers_.size()),
    clients_(clients.cbegin(), clients.cend()),
    shared_clients_(clients_.size()),
    services_(services.cbegin(), services.cend()),
    shared_services_(services_.size()),
    waitables_(waitables.cbegin(), waitables.cend()),
    shared_waitables_(waitables_.size())
  {}

  ~DynamicStorage() = default;

  template<class ArrayOfExtraGuardConditions>
  void
  storage_rebuild_rcl_wait_set(const ArrayOfExtraGuardConditions & extra_guard_conditions)
  {
    this->storage_rebuild_rcl_wait_set_with_sets(
      subscriptions_,
      guard_conditions_,
      extra_guard_conditions,
      timers_,
      clients_,
      services_,
      waitables_
    );
  }

  template<class EntityT, class SequenceOfEntitiesT>
  static
  bool
  storage_has_entity(const EntityT & entity, const SequenceOfEntitiesT & entities)
  {
    return std::any_of(
      entities.cbegin(),
      entities.cend(),
      [&entity](const auto & inner) {return &entity == inner.lock().get();});
  }

  template<class EntityT, class SequenceOfEntitiesT>
  static
  auto
  storage_find_entity(const EntityT & entity, const SequenceOfEntitiesT & entities)
  {
    return std::find_if(
      entities.cbegin(),
      entities.cend(),
      [&entity](const auto & inner) {return &entity == inner.lock().get();});
  }

  void
  storage_add_subscription(std::shared_ptr<rclcpp::SubscriptionBase> && subscription)
  {
    if (this->storage_has_entity(*subscription, subscriptions_)) {
      throw std::runtime_error("subscription already in wait set");
    }
    WeakSubscriptionEntry weak_entry{std::move(subscription), {}};
    subscriptions_.push_back(std::move(weak_entry));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_subscription(std::shared_ptr<rclcpp::SubscriptionBase> && subscription)
  {
    auto it = this->storage_find_entity(*subscription, subscriptions_);
    if (subscriptions_.cend() == it) {
      throw std::runtime_error("subscription not in wait set");
    }
    subscriptions_.erase(it);
    this->storage_flag_for_resize();
  }

  void
  storage_add_guard_condition(std::shared_ptr<rclcpp::GuardCondition> && guard_condition)
  {
    if (this->storage_has_entity(*guard_condition, guard_conditions_)) {
      throw std::runtime_error("guard_condition already in wait set");
    }
    guard_conditions_.push_back(std::move(guard_condition));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_guard_condition(std::shared_ptr<rclcpp::GuardCondition> && guard_condition)
  {
    auto it = this->storage_find_entity(*guard_condition, guard_conditions_);
    if (guard_conditions_.cend() == it) {
      throw std::runtime_error("guard_condition not in wait set");
    }
    guard_conditions_.erase(it);
    this->storage_flag_for_resize();
  }

  void
  storage_add_timer(std::shared_ptr<rclcpp::TimerBase> && timer)
  {
    if (this->storage_has_entity(*timer, timers_)) {
      throw std::runtime_error("timer already in wait set");
    }
    timers_.push_back(std::move(timer));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_timer(std::shared_ptr<rclcpp::TimerBase> && timer)
  {
    auto it = this->storage_find_entity(*timer, timers_);
    if (timers_.cend() == it) {
      throw std::runtime_error("timer not in wait set");
    }
    timers_.erase(it);
    this->storage_flag_for_resize();
  }

  void
  storage_add_client(std::shared_ptr<rclcpp::ClientBase> && client)
  {
    if (this->storage_has_entity(*client, clients_)) {
      throw std::runtime_error("client already in wait set");
    }
    clients_.push_back(std::move(client));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_client(std::shared_ptr<rclcpp::ClientBase> && client)
  {
    auto it = this->storage_find_entity(*client, clients_);
    if (clients_.cend() == it) {
      throw std::runtime_error("client not in wait set");
    }
    clients_.erase(it);
    this->storage_flag_for_resize();
  }

  void
  storage_add_service(std::shared_ptr<rclcpp::ServiceBase> && service)
  {
    if (this->storage_has_entity(*service, services_)) {
      throw std::runtime_error("service already in wait set");
    }
    services_.push_back(std::move(service));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_service(std::shared_ptr<rclcpp::ServiceBase> && service)
  {
    auto it = this->storage_find_entity(*service, services_);
    if (services_.cend() == it) {
      throw std::runtime_error("service not in wait set");
    }
    services_.erase(it);
    this->storage_flag_for_resize();
  }

  void
  storage_add_waitable(
    std::shared_ptr<rclcpp::Waitable> && waitable,
    std::shared_ptr<void> && associated_entity)
  {
    if (this->storage_has_entity(*waitable, waitables_)) {
      throw std::runtime_error("waitable already in wait set");
    }
    WeakWaitableEntry weak_entry(std::move(waitable), std::move(associated_entity));
    waitables_.push_back(std::move(weak_entry));
    this->storage_flag_for_resize();
  }

  void
  storage_remove_waitable(std::shared_ptr<rclcpp::Waitable> && waitable)
  {
    auto it = this->storage_find_entity(*waitable, waitables_);
    if (waitables_.cend() == it) {
      throw std::runtime_error("waitable not in wait set");
    }
    waitables_.erase(it);
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
    timers_.erase(std::remove_if(timers_.begin(), timers_.end(), p));
    clients_.erase(std::remove_if(clients_.begin(), clients_.end(), p));
    services_.erase(std::remove_if(services_.begin(), services_.end(), p));
    waitables_.erase(std::remove_if(waitables_.begin(), waitables_.end(), p));
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
    lock_all(timers_, shared_timers_);
    lock_all(clients_, shared_clients_);
    lock_all(services_, shared_services_);

    // We need a specialized version of this for waitables.
    auto lock_all_waitables = [](const auto & weak_ptrs, auto & shared_ptrs) {
        shared_ptrs.resize(weak_ptrs.size());
        size_t index = 0;
        for (const auto & weak_ptr : weak_ptrs) {
          shared_ptrs[index++] = WaitableEntry{
            weak_ptr.waitable.lock(),
            weak_ptr.associated_entity.lock()};
        }
      };
    lock_all_waitables(waitables_, shared_waitables_);
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
    reset_all(shared_timers_);
    reset_all(shared_clients_);
    reset_all(shared_services_);
    reset_all(shared_waitables_);
  }

  size_t ownership_reference_counter_ = 0;

  SequenceOfWeakSubscriptions subscriptions_;
  SubscriptionsIterable shared_subscriptions_;

  SequenceOfWeakGuardConditions guard_conditions_;
  GuardConditionsIterable shared_guard_conditions_;

  SequenceOfWeakTimers timers_;
  TimersIterable shared_timers_;

  SequenceOfWeakClients clients_;
  ClientsIterable shared_clients_;

  SequenceOfWeakServices services_;
  ServicesIterable shared_services_;

  SequenceOfWeakWaitables waitables_;
  WaitablesIterable shared_waitables_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DYNAMIC_STORAGE_HPP_
