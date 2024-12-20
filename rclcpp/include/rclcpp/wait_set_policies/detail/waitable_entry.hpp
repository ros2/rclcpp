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

/// See rclcpp::wait_set_policies::detail::EntityEntryTemplate.
template<>
class EntityEntryTemplate<rclcpp::Waitable>
{
  using EntityT = rclcpp::Waitable;

public:
  EntityEntryTemplate(
    std::shared_ptr<EntityT> entity_in = nullptr,
    std::shared_ptr<void> associated_entity_in = nullptr)
  : entity_(entity_in),
    associated_entity_(associated_entity_in)
  {}
private:
  std::shared_ptr<EntityT> entity_;
  std::shared_ptr<void> associated_entity_;

  friend ManagedEntityEntryTemplate<EntityT>;
};

/// See rclcpp::wait_set_policies::detail::ManagedEntityEntryTemplate.
template<>
class ManagedEntityEntryTemplate<rclcpp::Waitable>
{
  using EntityT = rclcpp::Waitable;

public:
  /// The only valid way to construct this is with an unmanaged entity entry.
  explicit ManagedEntityEntryTemplate(const EntityEntryTemplate<EntityT> & unmanaged_entity_entry)
  : entity_(unmanaged_entity_entry.entity_),
    associated_entity_(unmanaged_entity_entry.associated_entity_)
  {
    if (nullptr == entity_) {
      throw std::invalid_argument("entity cannot be nullptr for a managed entry");
    }
    bool already_in_use = entity_->exchange_in_use_by_wait_set_state(true);
    if (already_in_use) {
      throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity_);
    }
  }

  // ManagedEntityEntryTemplate(const ManagedEntityEntryTemplate<EntityT> & other)
  // {
  //   if (other.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
  //     throw std::runtime_error("")
  //   }
  // }

  ~ManagedEntityEntryTemplate()
  {
    if ((nullptr != entity_)) {
      bool was_in_use = entity_->exchange_in_use_by_wait_set_state(false);
      assert(was_in_use);
    }
  }

  /// Return the interal entity shared pointer.
  std::shared_ptr<EntityT>
  get_entity() const
  {
    return entity_;
  }

  /// Return the interal associated entity shared pointer.
  std::shared_ptr<void>
  get_associated_entity() const
  {
    return associated_entity_;
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

protected:
  std::shared_ptr<EntityT> entity_;
  std::shared_ptr<void> associated_entity_;

};

/// See rclcpp::wait_set_policies::detail::WeakManagedEntityEntryTemplate.
template<>
class WeakManagedEntityEntryTemplate<rclcpp::Waitable>
{
  using EntityT = rclcpp::Waitable;

public:
  /// Can only be constructed from a moved ManagedEntityEntryTemplate.
  explicit WeakManagedEntityEntryTemplate(ManagedEntityEntryTemplate<EntityT> && moved_entity_entry)
  : weak_entity_(moved_entity_entry.get_entity()),
    weak_associated_entity_(moved_entity_entry.get_associated_entity())
  {}

  ~WeakManagedEntityEntryTemplate()
  {
    auto entity = weak_entity_.lock();
    if (nullptr != entity) {
      bool was_in_use = entity->exchange_in_use_by_wait_set_state(false);
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

using WaitableEntry = EntityEntryTemplate<rclcpp::Waitable>;
using ManagedWaitableEntry = ManagedEntityEntryTemplate<rclcpp::Waitable>;
using WeakManagedWaitableEntry = WeakManagedEntityEntryTemplate<rclcpp::Waitable>;

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__WAITABLE_ENTRY_HPP_
