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
#include <optional>
#include <vector>

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

/// See rclcpp::wait_set_policies::detail::EntityEntryTemplate.
template<>
class EntityEntryTemplate<rclcpp::SubscriptionBase>
{
  using EntityT = rclcpp::SubscriptionBase;

public:
  EntityEntryTemplate(
    std::shared_ptr<EntityT> entity_in = nullptr,
    const rclcpp::SubscriptionWaitSetMask & mask_in = {})
  : entity_(entity_in),
    mask_(mask_in)
  {}

  /// Create ManagedEntityEntryTemplate instances for the subscription and waitables if needed.
  /**
   * This method uses the SubscriptionWaitSetMask to determine how to decompose
   * the various entities within the subscription into managed entries.
   *
   * It optionally returns a managed entry for the subscription, if the mask
   * indicates it should, and then optionally any waitables for the
   * intra-process communication and QoS events.
   */
  std::pair<
    std::optional<ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>>,
    std::vector<ManagedEntityEntryTemplate<rclcpp::Waitable>>
  >
  manage();
  // defined below ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>.

private:
  std::shared_ptr<EntityT> entity_;
  rclcpp::SubscriptionWaitSetMask mask_;
};

/// See rclcpp::wait_set_policies::detail::ManagedEntityEntryTemplate.
template<>
class ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>
{
  using EntityT = rclcpp::SubscriptionBase;

  /// Only constructed by EntityEntryTemplate<rclcpp::SubscriptionBase>::manage().
  explicit ManagedEntityEntryTemplate(std::shared_ptr<EntityT> subscription)
  : entity_(subscription)
  {
    if (nullptr == entity_) {
      throw std::invalid_argument("entity cannot be nullptr for a managed entry");
    }
    bool already_in_use = entity_->exchange_in_use_by_wait_set_state(entity_.get(), true);
    if (already_in_use) {
      throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity_);
    }
  }

public:
  // ManagedEntityEntryTemplate(const ManagedEntityEntryTemplate<EntityT> & other)
  // {
  //   if (other.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
  //     throw std::runtime_error("")
  //   }
  // }

  ~ManagedEntityEntryTemplate()
  {
    if ((nullptr != entity_)) {
      bool was_in_use = entity_->exchange_in_use_by_wait_set_state(entity_.get(), false);
      assert(was_in_use);
    }
  }

  /// Return the interal entity shared pointer.
  std::shared_ptr<EntityT>
  get_entity() const
  {
    return entity_;
  }

  /// Reset the entity.
  /**
   * Specializations of this class may reset more than one item.
   * Having this method in all instantiations of this class provides uniform access.
   */
  // void
  // reset() noexcept
  // {
  //   entity_.reset();
  // }

private:
  std::shared_ptr<EntityT> entity_;

  friend EntityEntryTemplate<EntityT>;

};

std::pair<
  std::optional<ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>>,
  std::vector<ManagedEntityEntryTemplate<rclcpp::Waitable>>
>
EntityEntryTemplate<rclcpp::SubscriptionBase>::manage()
{
  std::optional<ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>> managed_subscription_entry;
  std::vector<ManagedEntityEntryTemplate<rclcpp::Waitable>> waitables;

  if (mask_.include_subscription) {
    managed_subscription_entry = ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>(entity_);
  }
  if (mask_.include_events) {
    for (const auto & event_waitable : entity_->get_event_handlers()) {
      waitables.emplace_back(EntityEntryTemplate<rclcpp::Waitable>(event_waitable, entity_));
    }
  }
  if (mask_.include_intra_process_waitable) {
    waitables.emplace_back(
      EntityEntryTemplate<rclcpp::Waitable>(entity_->get_intra_process_waitable(), entity_)
    );
  }

  return {managed_subscription_entry, waitables};
}

/// See rclcpp::wait_set_policies::detail::WeakManagedEntityEntryTemplate.
template<>
class WeakManagedEntityEntryTemplate<rclcpp::SubscriptionBase>
{
  using EntityT = rclcpp::SubscriptionBase;

public:
  /// Can only be constructed from a moved ManagedEntityEntryTemplate.
  explicit WeakManagedEntityEntryTemplate(ManagedEntityEntryTemplate<EntityT> && moved_entity_entry)
  : weak_entity_(moved_entity_entry.get_entity())
  {}

  ~WeakManagedEntityEntryTemplate()
  {
    auto entity = weak_entity_.lock();
    if (nullptr != entity) {
      bool was_in_use = entity->exchange_in_use_by_wait_set_state(entity.get(), false);
      assert(was_in_use);
    }
  }

  /// Return the interal entity weak pointer.
  std::weak_ptr<EntityT>
  get_weak_entity() const
  {
    return weak_entity_;
  }

  // /// Lock the entity.
  // /**
  //  * Specializations of this class may select from more than one item to lock.
  //  * Having this method in all instantiations of this class provides uniform access.
  //  */
  // std::shared_ptr<EntityT>
  // lock() const
  // {
  //   return weak_entity_.lock();
  // }

  // /// Return true if the entity has expired, otherwise false.
  // /**
  //  * Specializations of this class may select from more than one item to check.
  //  * Having this method in all instantiations of this class provides uniform access.
  //  */
  // bool
  // expired() const noexcept
  // {
  //   return weak_entity_.expired();
  // }
private:
  std::weak_ptr<EntityT> weak_entity_;
  std::weak_ptr<void> weak_associated_entity_;

};

using SubscriptionEntry = EntityEntryTemplate<rclcpp::SubscriptionBase>;
using ManagedSubscriptionEntry = ManagedEntityEntryTemplate<rclcpp::SubscriptionBase>;
using WeakManagedSubscriptionEntry = WeakManagedEntityEntryTemplate<rclcpp::SubscriptionBase>;

// /// Specialization for Subscriptions.
// template<>
// class EntityEntryTemplate<rclcpp::SubscriptionBase>
// {
//   using EntityT = rclcpp::SubscriptionBase;
//   std::shared_ptr<EntityT> entity_;
//   rclcpp::SubscriptionWaitSetMask mask_;

// public:
//   EntityEntryTemplate(
//     std::shared_ptr<EntityT> entity_in = nullptr,
//     const rclcpp::SubscriptionWaitSetMask & mask_in = {})
//   : entity_(entity_in),
//     mask_(mask_in)
//   {}

//   ~EntityEntryTemplate()
//   {
//     if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
//       EntityEntryTemplate<rclcpp::SubscriptionBase>::cleanup(entity_, mask_);
//     }
//   }

//   /// See EntityEntryTemplate::get_entity().
//   std::shared_ptr<EntityT>
//   get_entity() const
//   {
//     return entity_;
//   }

//   /// Return a const reference to the subscrption mask.
//   const rclcpp::SubscriptionWaitSetMask &
//   get_mask() const
//   {
//     return mask_;
//   }

//   /// See EntityEntryTemplate::manage().
//   std::vector<WaitableEntry>
//   manage()
//   {
//     if (nullptr == entity_) {
//       throw std::runtime_error("manage() called on EntityEntry with null entity");
//     }
//     auto associate = [this](void * entity_part) {
//       bool already_in_use = entity_->exchange_in_use_by_wait_set_state(entity_part, true);
//       if (already_in_use) {
//         throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity_);
//       }
//     };

//     if (mask_.include_subscription) {
//       associate(entity_.get());
//     }

//     std::vector<WaitableEntry> decomposed_waitables;

//     if (mask_.include_events) {
//       for (auto event : entity_->get_event_handlers()) {
//         // TODO(wjwwood): do we need to use the unmanage_function argument
//         //   to utilize the exchange_in_use_by_wait_set_state of SubscriptionBase?
//         decomposed_waitables.emplace_back(event, entity_);
//         decomposed_waitables.back().manage();
//       }
//       // mask_.include_events = false;
//     }
//     if (mask_.include_intra_process_waitable) {
//       auto waitable = entity_->get_intra_process_waitable();
//       if (nullptr != waitable) {
//         // TODO(wjwwood): do we need to use the unmanage_function argument
//         //   to utilize the exchange_in_use_by_wait_set_state of SubscriptionBase?
//         decomposed_waitables.emplace_back(waitable, entity_);
//         decomposed_waitables.back().manage();
//         // mask_.include_intra_process_waitable = false;
//       }
//     }

//     should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;

//     return decomposed_waitables;
//   }

//   /// See EntityEntryTemplate::reset().
//   void
//   reset() noexcept
//   {
//     entity_.reset();
//   }

// protected:
//   bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

//   static
//   void
//   cleanup(std::shared_ptr<EntityT> subscription, rclcpp::SubscriptionWaitSetMask mask)
//   {
//     if (subscription != nullptr) {
//       auto dissociate = [&subscription](void * entity_part) {
//         bool was_in_use = subscription->exchange_in_use_by_wait_set_state(entity_part, false);
//         assert(was_in_use);
//       };
//       if (mask.include_subscription) {
//         dissociate(subscription.get());
//       }
//       if (mask.include_events) {
//         for (auto event : subscription->get_event_handlers()) {
//           dissociate(event.get());
//         }
//       }
//       if (mask.include_intra_process_waitable) {
//         auto waitable = subscription->get_intra_process_waitable();
//         if (nullptr != waitable) {
//           dissociate(waitable.get());
//         }
//       }
//     }
//   }

//   friend WeakEntityEntryTemplate<EntityT>;

// };

// /// Specialization for Subscriptions.
// template<>
// class WeakEntityEntryTemplate<rclcpp::SubscriptionBase>
// {
//   using EntityT = rclcpp::SubscriptionBase;
//   std::weak_ptr<EntityT> weak_entity_;
//   rclcpp::SubscriptionWaitSetMask mask_;

// public:
//   explicit WeakEntityEntryTemplate(EntityEntryTemplate<EntityT> && moved_entity_entry)
//   : weak_entity_(moved_entity_entry.get_entity()),
//     mask_(moved_entity_entry.mask_),
//     should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_(
//       moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_
//     )
//   {
//     moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = false;
//   }

//   ~WeakEntityEntryTemplate()
//   {
//     auto entity = weak_entity_.lock();
//     if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
//       EntityEntryTemplate<rclcpp::SubscriptionBase>::cleanup(entity, mask_);
//     }
//   }

//   /// See WeakEntityEntryTemplate::get_weak_entity().
//   std::weak_ptr<EntityT>
//   get_weak_entity() const
//   {
//     return weak_entity_;
//   }

//   /// Return a const reference to the subscrption mask.
//   const rclcpp::SubscriptionWaitSetMask &
//   get_mask() const
//   {
//     return mask_;
//   }

//   /// See WeakEntityEntryTemplate::lock().
//   std::shared_ptr<EntityT>
//   lock() const
//   {
//     return weak_entity_.lock();
//   }

//   /// See WeakEntityEntryTemplate::expired().
//   bool
//   expired() const noexcept
//   {
//     return weak_entity_.expired();
//   }

// protected:
//   bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

// };

// using SubscriptionEntry = EntityEntryTemplate<rclcpp::SubscriptionBase>;
// using WeakSubscriptionEntry = WeakEntityEntryTemplate<rclcpp::SubscriptionBase>;

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__SUBSCRIPTION_ENTRY_HPP_
