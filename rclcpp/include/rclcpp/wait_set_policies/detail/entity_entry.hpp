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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__ENTITY_ENTRY_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__ENTITY_ENTRY_HPP_

#include <cassert>
#include <memory>

#include "rclcpp/wait_set_policies/already_associated_with_wait_set_exception.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

// Forward declaration for use in friend statement.
template<typename EntityT>
class WeakEntityEntryTemplate;

/// RAII-style class to acquire/release "use of" an entity for the wait set automatically.
/**
 * The entity is stored as a std::shared_ptr, but a "weak" version of this class
 * can be created which stores the entity as a std::weak_ptr instead along with
 * whether or not it is associated and should unassociated when destructed.
 *
 * This is actually quasi-RAII, because the "resource acquisition" only occurs
 * when manage() is called, and is only released if manage was called before
 * destruction.
 */
template<typename EntityT>
class EntityEntryTemplate
{
  std::shared_ptr<EntityT> entity_;

public:
  EntityEntryTemplate(std::shared_ptr<EntityT> entity_in = nullptr)
  : entity_(entity_in)
  {}

  ~EntityEntryTemplate()
  {
    if ((nullptr != entity_) && should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
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

  /// Bring the entity under "management" by setting it to "in use" by a wait set.
  /**
   * This is reversed in the destruction of this class.
   *
   * \throws std::runtime_error if entity is nullptr
   */
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

  /// Reset the entity.
  /**
   * Specializations of this class may reset more than one item.
   * Having this method in all instantiations of this class provides uniform access.
   */
  void
  reset() noexcept
  {
    entity_.reset();
  }

protected:
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

  friend WeakEntityEntryTemplate<EntityT>;

};

/// Weak version of EntityEntryTemplate, which can only be created from an EntityEntryTemplate.
/**
 * The entity is stored as a std::weak_ptr, but on destruction, the entity is
 * locked, and if not null and should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_
 * is true, then it will be marked as not in use by a wait set.
 */
template<typename EntityT>
class WeakEntityEntryTemplate
{
  std::weak_ptr<EntityT> weak_entity_;

public:
  explicit WeakEntityEntryTemplate(EntityEntryTemplate<EntityT> && moved_entity_entry)
  : weak_entity_(moved_entity_entry.get_entity()),
    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_(
      moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_
    )
  {
    moved_entity_entry.should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = false;
  }

  ~WeakEntityEntryTemplate()
  {
    auto entity = weak_entity_.lock();
    if ((nullptr != entity) && should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
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

  /// Lock the entity.
  /**
   * Specializations of this class may select from more than one item to lock.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::shared_ptr<EntityT>
  lock() const
  {
    return weak_entity_.lock();
  }

  /// Return true if the entity has expired, otherwise false.
  /**
   * Specializations of this class may select from more than one item to check.
   * Having this method in all instantiations of this class provides uniform access.
   */
  bool
  expired() const noexcept
  {
    return weak_entity_.expired();
  }

protected:
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#if 0
namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// RAII-style class to acquire/release "use of" an entity for the wait set automatically.
/**
 * The entity can be stored as either a std::shared_ptr or std::weak_ptr, based
 * on the SmartPointerT template argument.
 *
 * This is actually quasi-RAII, because the "resource acquisition" only occurs
 * when manage() is called, and is only released if manage was called before
 * destruction.
 */
template<typename EntityT, typename SmartPointerT>
class EntityEntryTemplate
{
  static constexpr bool is_shared_ptr = std::is_same_v<SmartPointerT, std::shared_ptr<EntityT>>;

public:
  SmartPointerT entity;

  EntityEntryTemplate(std::shared_ptr<EntityT> entity_in = nullptr)
  : entity(entity_in)
  {}

  ~EntityEntryTemplate()
  {
    if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
      bool was_in_use = entity->exchange_in_use_by_wait_set_state(false);
      assert(was_in_use);
    }
  }

  /// Bring the entity under "management" by setting it to "in use" by a wait set.
  /**
   * This is reversed in the destruction of this class.
   */
  void
  manage()
  {
    bool already_in_use = entity->exchange_in_use_by_wait_set_state(true);
    if (already_in_use) {
      throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity);
    }
    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;
  }

  /// Reset the entity.
  /**
   * Specializations of this class may reset more than one item.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::enable_if_t<is_shared_ptr>
  reset() noexcept
  {
    entity.reset();
  }

  /// Lock the entity.
  /**
   * Specializations of this class may select from more than one item to lock.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::enable_if_t<!is_shared_ptr, std::shared_ptr<EntityT>>
  lock() const
  {
    return entity.lock();
  }

  /// Return true if the entity has expired, otherwise false.
  /**
   * Specializations of this class may select from more than one item to check.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::enable_if_t<!is_shared_ptr, bool>
  expired() const noexcept
  {
    return entity.expired();
  }

protected:
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp
#endif


#if 0
namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

/// Template declaration for a class that adds utility methods based on the smart pointer type.
template<typename EntityT, typename SmartPointerT>
class EntityEntryUtilityMethodsHelper
{};

/// Partial specialization for shared_ptr.
template<typename EntityT>
class EntityEntryUtilityMethodsHelper<EntityT, std::shared_ptr<EntityT>>
{
protected:
  static constexpr bool is_shared_ptr = true;

public:
  std::shared_ptr<EntityT> entity;

  /// Reset the entity.
  /**
   * Specializations of this class may reset more than one item.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::enable_if_t<is_shared_ptr>
  reset() noexcept
  {
    entity.reset();
  }
};

/// Partial specialization for weak_ptr.
template<typename EntityT>
class EntityEntryUtilityMethodsHelper<EntityT, std::weak_ptr<EntityT>>
{
protected:
  static constexpr bool is_shared_ptr = false;

public:
  std::weak_ptr<EntityT> entity;

  /// Lock the entity.
  /**
   * Specializations of this class may select from more than one item to lock.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::enable_if_t<!is_shared_ptr, std::shared_ptr<EntityT>>
  lock() const
  {
    return entity.lock();
  }

  /// Return true if the entity has expired, otherwise false.
  /**
   * Specializations of this class may select from more than one item to check.
   * Having this method in all instantiations of this class provides uniform access.
   */
  std::enable_if_t<!is_shared_ptr, bool>
  expired() const noexcept
  {
    return entity.expired();
  }
};

/// RAII-style class to acquire/release "use of" an entity for the wait set automatically.
/**
 * The entity can be stored as either a std::shared_ptr or std::weak_ptr, based
 * on the SmartPointerT template argument.
 *
 * This is actually quasi-RAII, because the "resource acquisition" only occurs
 * when manage() is called, and is only released if manage was called before
 * destruction.
 */
template<
  typename EntityT,
  typename SmartPointerT,
  typename EntityEntryUtilityMethodsT = EntityEntryUtilityMethodsHelper<EntityT, SmartPointerT>>
class EntityEntryTemplate : public EntityEntryUtilityMethodsT
{
  static constexpr bool is_shared_ptr = EntityEntryUtilityMethodsT::is_shared_ptr;

public:
  EntityEntryTemplate(std::shared_ptr<EntityT> entity_in = nullptr)
  : EntityEntryUtilityMethodsT(entity_in)
  {}

  ~EntityEntryTemplate()
  {
    if (should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_) {
      bool was_in_use = entity->exchange_in_use_by_wait_set_state(false);
      assert(was_in_use);
    }
  }

  /// Bring the entity under "management" by setting it to "in use" by a wait set.
  /**
   * This is reversed in the destruction of this class.
   */
  void
  manage()
  {
    bool already_in_use = entity->exchange_in_use_by_wait_set_state(true);
    if (already_in_use) {
      throw rclcpp::wait_set_policies::AlreadyAssociatedWithWaitSetException(*entity);
    }
    should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_ = true;
  }

protected:
  bool should_set_in_use_by_wait_set_of_entity_to_false_on_destruction_{false};

};

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp
#endif

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__ENTITY_ENTRY_HPP_
