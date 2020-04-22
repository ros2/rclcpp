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

#ifndef RCLCPP__WAIT_SET_POLICIES__DETAIL__ENTRY_TYPES_HPP_
#define RCLCPP__WAIT_SET_POLICIES__DETAIL__ENTRY_TYPES_HPP_

#include <limits>
#include <memory>

#include "rclcpp/client.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace wait_set_policies
{
namespace detail
{

constexpr const size_t not_in_wait_set = (std::numeric_limits<std::size_t>::max)();

/// Templated ownership entries used for storage.
template<class PointerT>
class EntryTemplate : public PointerT
{
public:
  size_t rcl_wait_set_index;
  constexpr static const size_t not_in_wait_set = not_in_wait_set;

  /// Conversion constructor, which is intentionally not marked explicit.
  EntryTemplate(
    const PointerT & entity_in = nullptr,
    size_t rcl_wait_set_index_in = not_in_wait_set) noexcept
  : PointerT(entity_in),
    rcl_wait_set_index(rcl_wait_set_index_in)
  {}
};

/// Templated ownership subscription entries used for storage.
template<class PointerT>
class SubscriptionEntryTemplate : public EntryTemplate<PointerT>
{
public:
  rclcpp::SubscriptionWaitSetMask mask;

  /// Conversion constructor, which is intentionally not marked explicit.
  SubscriptionEntryTemplate(
    const PointerT & entity_in = nullptr,
    const rclcpp::SubscriptionWaitSetMask & mask_in = {},
    size_t rcl_wait_set_index_in = not_in_wait_set) noexcept
  : EntryTemplate<PointerT>(entity_in, rcl_wait_set_index_in),
    mask(mask_in)
  {}
};

/// Templated ownership entries with associated entity used for storage.
template<class PointerT>
class EntryWithAssociatedEntityTemplate : public PointerT
{
public:
  std::shared_ptr<void> associated_entity;

  /// Conversion constructor, which is intentionally not marked explicit.
  EntryWithAssociatedEntityTemplate(
    const PointerT & entity_in = nullptr,
    const std::shared_ptr<void> & associated_entity_in = nullptr) noexcept
  : PointerT(entity_in),
    associated_entity(associated_entity_in)
  {}
};

/// Shared ownership SubscriptionBase entry for storage.
using SubscriptionEntry = SubscriptionEntryTemplate<std::shared_ptr<rclcpp::SubscriptionBase>>;
/// Weak ownership SubscriptionBase entry for storage.
using WeakSubscriptionEntry = SubscriptionEntryTemplate<std::weak_ptr<rclcpp::SubscriptionBase>>;

/// Shared ownership GuardCondition entry for storage.
using GuardConditionEntry = EntryTemplate<std::shared_ptr<rclcpp::GuardCondition>>;
/// Weak ownership GuardCondition entry for storage.
using WeakGuardConditionEntry = EntryTemplate<std::weak_ptr<rclcpp::GuardCondition>>;

/// Shared ownership Timer entry for storage.
using TimerEntry = EntryTemplate<std::shared_ptr<rclcpp::TimerBase>>;
/// Weak ownership Timer entry for storage.
using WeakTimerEntry = EntryTemplate<std::weak_ptr<rclcpp::TimerBase>>;

/// Shared ownership Client entry for storage.
using ClientEntry = EntryTemplate<std::shared_ptr<rclcpp::ClientBase>>;
/// Weak ownership Client entry for storage.
using WeakClientEntry = EntryTemplate<std::weak_ptr<rclcpp::ClientBase>>;

/// Shared ownership Service entry for storage.
using ServiceEntry = EntryTemplate<std::shared_ptr<rclcpp::ServiceBase>>;
/// Weak ownership Service entry for storage.
using WeakServiceEntry = EntryTemplate<std::weak_ptr<rclcpp::ServiceBase>>;

/// Shared ownership Waitable entry for storage.
using WaitableEntry = EntryWithAssociatedEntityTemplate<std::shared_ptr<rclcpp::Waitable>>;
/// Weak ownership Waitable entry for storage.
using WeakWaitableEntry = EntryWithAssociatedEntityTemplate<std::weak_ptr<rclcpp::Waitable>>;

}  // namespace detail
}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__DETAIL__ENTRY_TYPES_HPP_
