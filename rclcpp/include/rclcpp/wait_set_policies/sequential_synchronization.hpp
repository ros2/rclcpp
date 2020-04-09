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

#ifndef RCLCPP__WAIT_SET_POLICIES__SEQUENTIAL_SYNCHRONIZATION_HPP_
#define RCLCPP__WAIT_SET_POLICIES__SEQUENTIAL_SYNCHRONIZATION_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <utility>

#include "rclcpp/client.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_result.hpp"
#include "rclcpp/wait_result_kind.hpp"
#include "rclcpp/wait_set_policies/detail/synchronization_policy_common.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace wait_set_policies
{

/// WaitSet policy that explicitly provides no thread synchronization.
class SequentialSynchronization : public detail::SynchronizationPolicyCommon
{
protected:
  explicit SequentialSynchronization(rclcpp::Context::SharedPtr) {}
  ~SequentialSynchronization() = default;

  /// Return any "extra" guard conditions needed to implement the synchronization policy.
  /**
   * Since this policy provides no thread-safety, it also needs no extra guard
   * conditions to implement it.
   */
  const std::array<std::shared_ptr<rclcpp::GuardCondition>, 0> &
  get_extra_guard_conditions()
  {
    static const std::array<std::shared_ptr<rclcpp::GuardCondition>, 0> empty{};
    return empty;
  }

  /// Add subscription without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_add_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> && subscription,
    const rclcpp::SubscriptionWaitSetMask & mask,
    std::function<
      void(std::shared_ptr<rclcpp::SubscriptionBase>&&, const rclcpp::SubscriptionWaitSetMask &)
    > add_subscription_function)
  {
    // Explicitly no thread synchronization.
    add_subscription_function(std::move(subscription), mask);
  }

  /// Remove guard condition without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_remove_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> && subscription,
    const rclcpp::SubscriptionWaitSetMask & mask,
    std::function<
      void(std::shared_ptr<rclcpp::SubscriptionBase>&&, const rclcpp::SubscriptionWaitSetMask &)
    > remove_subscription_function)
  {
    // Explicitly no thread synchronization.
    remove_subscription_function(std::move(subscription), mask);
  }

  /// Add guard condition without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_add_guard_condition(
    std::shared_ptr<rclcpp::GuardCondition> && guard_condition,
    std::function<void(std::shared_ptr<rclcpp::GuardCondition>&&)> add_guard_condition_function)
  {
    // Explicitly no thread synchronization.
    add_guard_condition_function(std::move(guard_condition));
  }

  /// Remove guard condition without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_remove_guard_condition(
    std::shared_ptr<rclcpp::GuardCondition> && guard_condition,
    std::function<void(std::shared_ptr<rclcpp::GuardCondition>&&)> remove_guard_condition_function)
  {
    // Explicitly no thread synchronization.
    remove_guard_condition_function(std::move(guard_condition));
  }

  /// Add timer without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_add_timer(
    std::shared_ptr<rclcpp::TimerBase> && timer,
    std::function<void(std::shared_ptr<rclcpp::TimerBase>&&)> add_timer_function)
  {
    // Explicitly no thread synchronization.
    add_timer_function(std::move(timer));
  }

  /// Remove timer without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_remove_timer(
    std::shared_ptr<rclcpp::TimerBase> && timer,
    std::function<void(std::shared_ptr<rclcpp::TimerBase>&&)> remove_timer_function)
  {
    // Explicitly no thread synchronization.
    remove_timer_function(std::move(timer));
  }

  /// Add client without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_add_client(
    std::shared_ptr<rclcpp::ClientBase> && client,
    std::function<void(std::shared_ptr<rclcpp::ClientBase>&&)> add_client_function)
  {
    // Explicitly no thread synchronization.
    add_client_function(std::move(client));
  }

  /// Remove client without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_remove_client(
    std::shared_ptr<rclcpp::ClientBase> && client,
    std::function<void(std::shared_ptr<rclcpp::ClientBase>&&)> remove_client_function)
  {
    // Explicitly no thread synchronization.
    remove_client_function(std::move(client));
  }

  /// Add service without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_add_service(
    std::shared_ptr<rclcpp::ServiceBase> && service,
    std::function<void(std::shared_ptr<rclcpp::ServiceBase>&&)> add_service_function)
  {
    // Explicitly no thread synchronization.
    add_service_function(std::move(service));
  }

  /// Remove service without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_remove_service(
    std::shared_ptr<rclcpp::ServiceBase> && service,
    std::function<void(std::shared_ptr<rclcpp::ServiceBase>&&)> remove_service_function)
  {
    // Explicitly no thread synchronization.
    remove_service_function(std::move(service));
  }

  /// Add waitable without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_add_waitable(
    std::shared_ptr<rclcpp::Waitable> && waitable,
    std::shared_ptr<void> && associated_entity,
    std::function<
      void(std::shared_ptr<rclcpp::Waitable>&&, std::shared_ptr<void>&&)
    > add_waitable_function)
  {
    // Explicitly no thread synchronization.
    add_waitable_function(std::move(waitable), std::move(associated_entity));
  }

  /// Remove waitable without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_remove_waitable(
    std::shared_ptr<rclcpp::Waitable> && waitable,
    std::function<void(std::shared_ptr<rclcpp::Waitable>&&)> remove_waitable_function)
  {
    // Explicitly no thread synchronization.
    remove_waitable_function(std::move(waitable));
  }

  /// Prune deleted entities without thread-safety.
  /**
   * Does not throw, but storage function may throw.
   */
  void
  sync_prune_deleted_entities(std::function<void()> prune_deleted_entities_function)
  {
    // Explicitly no thread synchronization.
    prune_deleted_entities_function();
  }

  /// Implements wait without any thread-safety.
  template<class WaitResultT>
  WaitResultT
  sync_wait(
    std::chrono::nanoseconds time_to_wait_ns,
    std::function<void()> rebuild_rcl_wait_set,
    std::function<rcl_wait_set_t & ()> get_rcl_wait_set,
    std::function<WaitResultT(WaitResultKind wait_result_kind)> create_wait_result)
  {
    // Assumption: this function assumes that some measure has been taken to
    // ensure none of the entities being waited on by the wait set are allowed
    // to go out of scope and therefore be deleted.
    // In the case of the StaticStorage policy, this is ensured because it
    // retains shared ownership of all entites for the duration of its own life.
    // In the case of the DynamicStorage policy, this is ensured by the function
    // which calls this function, by acquiring shared ownership of the entites
    // for the duration of this function.

    // Setup looping predicate.
    auto start = std::chrono::steady_clock::now();
    std::function<bool()> should_loop = this->create_loop_predicate(time_to_wait_ns, start);

    // Wait until exit condition is met.
    do {
      // Rebuild the wait set.
      // This will resize the wait set if needed, due to e.g. adding or removing
      // entities since the last wait, but this should never occur in static
      // storage wait sets since they cannot be changed after construction.
      // This will also clear the wait set and re-add all the entities, which
      // prepares it to be waited on again.
      rebuild_rcl_wait_set();

      rcl_wait_set_t & rcl_wait_set = get_rcl_wait_set();

      // Wait unconditionally until timeout condition occurs since we assume
      // there are no conditions that would require the wait to stop and reset,
      // like asynchronously adding or removing an entity, i.e. explicitly
      // providing no thread-safety.

      // Calculate how much time there is left to wait, unless blocking indefinitely.
      auto time_left_to_wait_ns = this->calculate_time_left_to_wait(time_to_wait_ns, start);

      // Then wait for entities to become ready.
      rcl_ret_t ret = rcl_wait(&rcl_wait_set, time_left_to_wait_ns.count());
      if (RCL_RET_OK == ret) {
        // Something has become ready in the wait set, and since this class
        // did not add anything to it, it is a user entity that is ready.
        return create_wait_result(WaitResultKind::Ready);
      } else if (RCL_RET_TIMEOUT == ret) {
        // The wait set timed out, exit the loop.
        break;
      } else if (RCL_RET_WAIT_SET_EMPTY == ret) {
        // Wait set was empty, return Empty.
        return create_wait_result(WaitResultKind::Empty);
      } else {
        // Some other error case, throw.
        rclcpp::exceptions::throw_from_rcl_error(ret);
      }
    } while (should_loop());

    // Wait did not result in ready items, return timeout.
    return create_wait_result(WaitResultKind::Timeout);
  }

  void
  sync_wait_result_acquire()
  {
    // Explicitly do nothing.
  }

  void
  sync_wait_result_release()
  {
    // Explicitly do nothing.
  }
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__SEQUENTIAL_SYNCHRONIZATION_HPP_
