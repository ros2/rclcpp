// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__SUBSCRIPTION_ENTRY_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__SUBSCRIPTION_ENTRY_HPP_

#include <memory>

#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/wait_set_policies/detail/entity_entry.hpp"
#include "rclcpp/wait_set_policies/detail/waitable_entry.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// Specialization for Subscriptions.
template<>
class EntityEntryTemplate<rclcpp::SubscriptionBase>
{
  using EntityT = rclcpp::SubscriptionBase;
  std::shared_ptr<EntityT> entity_;
  rclcpp::SubscriptionWaitSetMask mask_;

public:
  EntityEntryTemplate(
    std::shared_ptr<EntityT> entity_in = nullptr,
    const rclcpp::SubscriptionWaitSetMask & mask_in = {})
  : entity_(entity_in),
    mask_(mask_in)
  {}

  ~EntityEntryTemplate()
  {
    if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
      EntityEntryTemplate<rclcpp::SubscriptionBase>::cleanup(entity_, mask_);
    }
  }

  /// See EntityEntryTemplate::get_entity().
  std::shared_ptr<EntityT>
  get_entity() const
  {
    return entity_;
  }

  /// Return a const reference to the subscrption mask.
  const rclcpp::SubscriptionWaitSetMask &
  get_mask() const
  {
    return mask_;
  }

  /// See EntityEntryTemplate::manage().
  std::vector<WaitableEntry>
  manage()
  {
    if (nullptr == entity_) {
      throw std::runtime_error("manage() called on EntityEntry with null entity");
    }
    auto associate = [this](void * entity_part) {
      bool already_in_use = entity_->exchange_in_use_by_wait_set_state(entity_part, true);
      if (already_in_use) {
        throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity_);
      }
    };

    if (mask_.include_subscription) {
      associate(entity_.get());
    }

    std::vector<WaitableEntry> decomposed_waitables;

    if (mask_.include_events) {
      for (auto event : entity_->get_event_handlers()) {
        // TODO(wjwwood): do we need to use the unmanage_function argument
        //   to utilize the exchange_in_use_by_wait_set_state of SubscriptionBase?
        decomposed_waitables.emplace_back(event, entity_);
        decomposed_waitables.back().manage();
      }
      // mask_.include_events = false;
    }
    if (mask_.include_intra_process_waitable) {
      auto waitable = entity_->get_intra_process_waitable();
      if (nullptr != waitable) {
        // TODO(wjwwood): do we need to use the unmanage_function argument
        //   to utilize the exchange_in_use_by_wait_set_state of SubscriptionBase?
        decomposed_waitables.emplace_back(waitable, entity_);
        decomposed_waitables.back().manage();
        // mask_.include_intra_process_waitable = false;
      }
    }

    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;

    return decomposed_waitables;
  }

  /// See EntityEntryTemplate::reset().
  void
  reset() noexcept
  {
    entity_.reset();
  }

protected:
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

  static
  void
  cleanup(std::shared_ptr<EntityT> subscription, rclcpp::SubscriptionWaitSetMask mask)
  {
    if (subscription != nullptr) {
      auto dissociate = [&subscription](void * entity_part) {
        bool was_in_use = subscription->exchange_in_use_by_wait_set_state(entity_part, false);
        assert(was_in_use);
      };
      if (mask.include_subscription) {
        dissociate(subscription.get());
      }
      if (mask.include_events) {
        for (auto event : subscription->get_event_handlers()) {
          dissociate(event.get());
        }
      }
      if (mask.include_intra_process_waitable) {
        auto waitable = subscription->get_intra_process_waitable();
        if (nullptr != waitable) {
          dissociate(waitable.get());
        }
      }
    }
  }

  friend WeakEntityEntryTemplate<EntityT>;

};

/// Specialization for Subscriptions.
template<>
class WeakEntityEntryTemplate<rclcpp::SubscriptionBase>
{
  using EntityT = rclcpp::SubscriptionBase;
  std::weak_ptr<EntityT> weak_entity_;
  rclcpp::SubscriptionWaitSetMask mask_;

public:
  explicit WeakEntityEntryTemplate(EntityEntryTemplate<EntityT> && moved_entity_entry)
  : weak_entity_(moved_entity_entry.get_entity()),
    mask_(moved_entity_entry.mask_),
    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_(
      moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_
    )
  {
    moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = false;
  }

  ~WeakEntityEntryTemplate()
  {
    auto entity = weak_entity_.lock();
    if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
      EntityEntryTemplate<rclcpp::SubscriptionBase>::cleanup(entity, mask_);
    }
  }

  /// See WeakEntityEntryTemplate::get_weak_entity().
  std::weak_ptr<EntityT>
  get_weak_entity() const
  {
    return weak_entity_;
  }

  /// Return a const reference to the subscrption mask.
  const rclcpp::SubscriptionWaitSetMask &
  get_mask() const
  {
    return mask_;
  }

  /// See WeakEntityEntryTemplate::lock().
  std::shared_ptr<EntityT>
  lock() const
  {
    return weak_entity_.lock();
  }

  /// See WeakEntityEntryTemplate::expired().
  bool
  expired() const noexcept
  {
    return weak_entity_.expired();
  }

protected:
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

};

using SubscriptionEntry = EntityEntryTemplate<rclcpp::SubscriptionBase>;
using WeakSubscriptionEntry = WeakEntityEntryTemplate<rclcpp::SubscriptionBase>;

// /// Partial Specialization of EntityEntryTemplate for Subscriptions.
// template<typename SmartPointerT>
// class EntityEntryTemplate<rclcpp::SubscriptionBase, SmartPointerT>
// {
//   using EntityT = rclcpp::SubscriptionBase;
//   static constexpr bool is_shared_ptr = std::is_same_v<SmartPointerT, std::shared_ptr<EntityT>>;

// public:
//   SmartPointerT entity;
//   rclcpp::SubscriptionWaitSetMask mask;

//   EntityEntryTemplate(
//     std::shared_ptr<EntityT> entity_in = nullptr,
//     const rclcpp::SubscriptionWaitSetMask & mask_in = {})
//   : entity(entity_in),
//     mask(mask_in)
//   {}

//   ~EntityEntryTemplate()
//   {
//     auto dissociate = [this](void * entity_part) {
//       if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
//         bool was_in_use = entity->exchange_in_use_by_wait_set_state(entity_part, false);
//         assert(was_in_use);
//       }
//     };
//     if (mask.include_subscription) {
//       dissociate(entity.get());
//     }
//     if (mask.include_events) {
//       for (auto event : entity->get_event_handlers()) {
//         dissociate(event.get());
//       }
//     }
//     if (mask.include_intra_process_waitable) {
//       auto waitable = entity->get_intra_process_waitable();
//       if (nullptr != waitable) {
//         dissociate(waitable.get());
//       }
//     }
//   }

//   /// See EntityEntryTemplate::manage(), specialized for subscriptions.
//   void
//   manage()
//   {
//     auto associate = [this](void * entity_part) {
//       bool already_in_use = entity->exchange_in_use_by_wait_set_state(entity_part, true);
//       if (already_in_use) {
//         throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity);
//       }
//     };

//     if (mask.include_subscription) {
//       associate(entity.get());
//     }
//     if (mask.include_events) {
//       for (auto event : entity->get_event_handlers()) {
//         associate(event.get());
//       }
//     }
//     if (mask.include_intra_process_waitable) {
//       auto waitable = entity->get_intra_process_waitable();
//       if (nullptr != waitable) {
//         associate(waitable.get());
//       }
//     }

//     should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;
//   }

//   /// See EntityEntryTemplate::reset().
//   std::enable_if_t<is_shared_ptr>
//   reset() noexcept
//   {
//     entity.reset();
//   }

//   /// See EntityEntryTemplate::lock().
//   std::enable_if_t<!is_shared_ptr, std::shared_ptr<EntityT>>
//   lock() const
//   {
//     return entity.lock();
//   }

//   /// See EntityEntryTemplate::expired().
//   std::enable_if_t<!is_shared_ptr, bool>
//   expired() const noexcept
//   {
//     return entity.expired();
//   }

// protected:
//   bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

// };

// using SubscriptionEntry =
//   EntityEntryTemplate<rclcpp::SubscriptionBase, std::shared_ptr<rclcpp::SubscriptionBase>>;
// using WeakSubscriptionEntry =
//   EntityEntryTemplate<rclcpp::SubscriptionBase, std::weak_ptr<rclcpp::SubscriptionBase>>;

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__SUBSCRIPTION_ENTRY_HPP_
