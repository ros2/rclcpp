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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__WAITABLE_ENTRY_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__WAITABLE_ENTRY_HPP_

#include <functional>
#include <memory>
#include <type_traits>

#include "rclcpp/waitable.hpp"
#include "rclcpp/wait_set_policies/detail/entity_entry.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// Specialization for Waitables.
template<>
class EntityEntryTemplate<rclcpp::Waitable>
{
  using EntityT = rclcpp::Waitable;
  std::shared_ptr<EntityT> entity_;
  std::shared_ptr<void> associated_entity_;

public:
  EntityEntryTemplate(
    std::shared_ptr<EntityT> entity_in = nullptr,
    std::shared_ptr<void> associated_entity_in = nullptr,
    std::function<void (std::shared_ptr<EntityT>, std::shared_ptr<void>)> unmanage_function =
      EntityEntryTemplate<rclcpp::Waitable>::unmanage)
  : entity_(entity_in),
    associated_entity_(associated_entity_in),
    unmanage_function_(unmanage_function)
  {}

  ~EntityEntryTemplate()
  {
    if ((nullptr != entity_) && should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
      this->unmanage_function_(entity_, associated_entity_);
    }
  }

  /// See EntityEntryTemplate::get_entity().
  std::shared_ptr<EntityT>
  get_entity() const
  {
    return entity_;
  }

  /// See EntityEntryTemplate::manage().
  void
  manage()
  {
    if (nullptr == entity_) {
      throw std::runtime_error("manage() called on EntityEntry with null entity");
    }
    bool already_in_use = entity_->exchange_in_use_by_wait_set_state(true);
    if (already_in_use) {
      throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity_);
    }
    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;
  }

  /// See EntityEntryTemplate::reset().
  void
  reset() noexcept
  {
    entity_.reset();
    associated_entity_.reset();
  }

protected:
  static
  void
  unmanage(std::shared_ptr<EntityT> entity, std::shared_ptr<void>)
  {
    bool was_in_use = entity->exchange_in_use_by_wait_set_state(false);
    assert(was_in_use);
  }

  std::function<void (std::shared_ptr<EntityT>, std::shared_ptr<void>)> unmanage_function_;
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

  friend WeakEntityEntryTemplate<EntityT>;

};

/// Specialization for Waitables.
template<>
class WeakEntityEntryTemplate<rclcpp::Waitable>
{
  using EntityT = rclcpp::Waitable;
  std::weak_ptr<EntityT> weak_entity_;
  std::weak_ptr<void> weak_associated_entity_;

public:
  explicit WeakEntityEntryTemplate(EntityEntryTemplate<EntityT> && moved_entity_entry)
  : weak_entity_(moved_entity_entry.get_entity()),
    weak_associated_entity_(moved_entity_entry.associated_entity_),
    unmanage_function_(moved_entity_entry.unmanage_function_),
    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_(
      moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_
    )
  {
    moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = false;
  }

  ~WeakEntityEntryTemplate()
  {
    auto associated_entity = weak_associated_entity_.lock();
    auto entity = weak_entity_.lock();
    if (
      (nullptr != associated_entity) &&
      (nullptr != entity) &&
      should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_)
    {
      unmanage_function_(entity, associated_entity);
    }
  }

  /// See WeakEntityEntryTemplate::get_weak_entity().
  std::weak_ptr<EntityT>
  get_weak_entity() const
  {
    return weak_entity_;
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
  std::function<void (std::shared_ptr<EntityT>, std::shared_ptr<void>)> unmanage_function_;
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

};

using WaitableEntry = EntityEntryTemplate<rclcpp::Waitable>;
using WeakWaitableEntry = WeakEntityEntryTemplate<rclcpp::Waitable>;

// template<bool>
// class WaitableEntryUtilityFunctionHelper
// {};

// template<>
// class WaitableEntryUtilityFunctionHelper<true>
// {};

// template<>
// class WaitableEntryUtilityFunctionHelper<false>
// {};

// /// Partial Specialization of EntityEntryTemplate for Waitables.
// template<typename SmartPointerT>
// class EntityEntryTemplate<rclcpp::Waitable, SmartPointerT>
// {
//   using EntityT = rclcpp::Waitable;
//   static constexpr bool is_shared_ptr = std::is_same_v<SmartPointerT, std::shared_ptr<EntityT>>;
//   using VoidSmartPointerT =
//    std::conditional<is_shared_ptr, std::shared_ptr<void>, std::weak_ptr<void>>;

// public:
//   SmartPointerT entity;
//   VoidSmartPointerT associated_entity;

//   EntityEntryTemplate(
//     std::shared_ptr<EntityT> entity_in = nullptr,
//     std::shared_ptr<void> associated_entity_in = nullptr)
//   : entity(entity_in),
//     associated_entity(associated_entity_in)
//   {}

//   ~EntityEntryTemplate()
//   {
//     if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
//       bool was_in_use = entity->exchange_in_use_by_wait_set_state(false);
//       assert(was_in_use);
//     }
//   }

//   /// See EntityEntryTemplate::manage(), specialized for subscriptions.
//   void
//   manage()
//   {
//     bool already_in_use = entity->exchange_in_use_by_wait_set_state(true);
//     if (already_in_use) {
//       throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity);
//     }
//     should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;
//   }

//   /// See EntityEntryTemplate::reset().
//   std::enable_if_t<is_shared_ptr>
//   reset() noexcept
//   {
//     entity.reset();
//     associated_entity.reset();
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

// using WaitableEntry =
//   EntityEntryTemplate<rclcpp::Waitable, std::shared_ptr<rclcpp::Waitable>>;
// using WeakWaitableEntry =
//   EntityEntryTemplate<rclcpp::Waitable, std::weak_ptr<rclcpp::Waitable>>;

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__WAITABLE_ENTRY_HPP_
