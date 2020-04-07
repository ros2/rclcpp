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

#ifndef RCLCPP__WAIT_SET_POLICIES__STATIC_STORAGE_HPP_
#define RCLCPP__WAIT_SET_POLICIES__STATIC_STORAGE_HPP_

#include <array>
#include <memory>

#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
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

/// WaitSet policy that explicitly provides fixed sized storage only.
/**
 * Note the underlying rcl_wait_set_t is still dynamically allocated, but only
 * once during construction, and deallocated once during destruction.
 */
template<
  std::size_t NumberOfSubscriptions,
  std::size_t NumberOfGuardCondtions,
  std::size_t NumberOfTimers,
  // std::size_t NumberOfClients,
  // std::size_t NumberOfServices,
  std::size_t NumberOfWaitables
>
class StaticStorage : public rclcpp::wait_set_policies::detail::StoragePolicyCommon<true>
{
protected:
  using is_mutable = std::false_type;

  class SubscriptionEntry
  {
  public:
    std::shared_ptr<rclcpp::SubscriptionBase> subscription;
    rclcpp::SubscriptionWaitSetMask mask;

    /// Conversion constructor, which is intentionally not marked explicit.
    SubscriptionEntry(
      const std::shared_ptr<rclcpp::SubscriptionBase> & subscription_in = nullptr,
      const rclcpp::SubscriptionWaitSetMask & mask_in = {})
    : subscription(subscription_in),
      mask(mask_in)
    {}
  };
  using ArrayOfSubscriptions = std::array<
    SubscriptionEntry,
    NumberOfSubscriptions
  >;
  using SubscriptionsIterable = ArrayOfSubscriptions;

  using ArrayOfGuardConditions = std::array<
    std::shared_ptr<rclcpp::GuardCondition>,
    NumberOfGuardCondtions
  >;
  using GuardConditionsIterable = ArrayOfGuardConditions;

  using ArrayOfTimers = std::array<
    std::shared_ptr<rclcpp::TimerBase>,
    NumberOfTimers
  >;
  using TimersIterable = ArrayOfTimers;

  struct WaitableEntry
  {
    /// Conversion constructor, which is intentionally not marked explicit.
    WaitableEntry(
      const std::shared_ptr<rclcpp::Waitable> & waitable_in = nullptr,
      const std::shared_ptr<void> & associated_entity_in = nullptr) noexcept
    : waitable(waitable_in),
      associated_entity(associated_entity_in)
    {}

    std::shared_ptr<rclcpp::Waitable> waitable;
    std::shared_ptr<void> associated_entity;
  };
  using ArrayOfWaitables = std::array<
    WaitableEntry,
    NumberOfWaitables
  >;
  using WaitablesIterable = ArrayOfWaitables;

  explicit
  StaticStorage(
    const ArrayOfSubscriptions & subscriptions,
    const ArrayOfGuardConditions & guard_conditions,
    const ArrayOfTimers & timers,
    const ArrayOfWaitables & waitables,
    rclcpp::Context::SharedPtr context
  )
  : StoragePolicyCommon(subscriptions, guard_conditions, timers, waitables, context),
    subscriptions_(subscriptions),
    guard_conditions_(guard_conditions),
    timers_(timers),
    waitables_(waitables)
  {}

  ~StaticStorage() = default;

  void
  storage_rebuild_rcl_wait_set()
  {
    this->storage_rebuild_rcl_wait_set_with_sets(
      subscriptions_,
      guard_conditions_,
      timers_,
      waitables_
    );
  }

  // storage_add_subscription() explicitly not declared here
  // storage_remove_subscription() explicitly not declared here
  // storage_add_guard_condition() explicitly not declared here
  // storage_remove_guard_condition() explicitly not declared here
  // storage_add_timer() explicitly not declared here
  // storage_remove_timer() explicitly not declared here
  // storage_add_waitable() explicitly not declared here
  // storage_remove_waitable() explicitly not declared here
  // storage_prune_deleted_entities() explicitly not declared here

  void
  storage_acquire_ownerships()
  {
    // Explicitly do nothing.
  }

  void
  storage_release_ownerships()
  {
    // Explicitly do nothing.
  }

  const ArrayOfSubscriptions subscriptions_;
  const ArrayOfGuardConditions guard_conditions_;
  const ArrayOfTimers timers_;
  const ArrayOfWaitables waitables_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__STATIC_STORAGE_HPP_
