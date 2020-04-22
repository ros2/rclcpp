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

#ifndef RCLCPP__WAIT_SET_POLICIES__THREAD_SAFE_SYNCHRONIZATION_HPP_
#define RCLCPP__WAIT_SET_POLICIES__THREAD_SAFE_SYNCHRONIZATION_HPP_

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
#include "rclcpp/wait_set_policies/detail/write_preferring_read_write_lock.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace wait_set_policies
{

/// WaitSet policy that provides thread-safe synchronization for the wait set.
/**
 * This class uses a "write-preferring RW lock" so that adding items to, and
 * removing items from, the wait set will take priority over reading, i.e.
 * waiting.
 * This is done since add and remove calls will interrupt the wait set anyways
 * so it is wasteful to do "fair" locking when there are many add/remove
 * operations queued up.
 *
 * There are some things to consider about the thread-safety provided by this
 * policy.
 * There are two categories of activities, reading and writing activities.
 * The writing activities include all of the add and remove methods, as well as
 * the prune_deleted_entities() method.
 * The reading methods include the wait() method and keeping a WaitResult in
 * scope.
 * The reading and writing activities will not be run at the same time, and one
 * will block the other.
 * Therefore, if you are holding a WaitResult in scope, and try to add or
 * remove an entity at the same time, they will block each other.
 * The write activities will try to interrupt the wait() method by triggering
 * a guard condition, but they have no way of causing the WaitResult to release
 * its lock.
 */
class ThreadSafeSynchronization : public detail::SynchronizationPolicyCommon
{
protected:
  explicit ThreadSafeSynchronization(rclcpp::Context::SharedPtr context)
  : extra_guard_conditions_{{std::make_shared<rclcpp::GuardCondition>(context)}},
    wprw_lock_([this]() {this->interrupt_waiting_wait_set();})
  {}
  ~ThreadSafeSynchronization() = default;

  /// Return any "extra" guard conditions needed to implement the synchronization policy.
  /**
   * This policy has one guard condition which is used to interrupt the wait
   * set when adding and removing entities.
   */
  const std::array<std::shared_ptr<rclcpp::GuardCondition>, 1> &
  get_extra_guard_conditions()
  {
    return extra_guard_conditions_;
  }

  /// Interrupt any waiting wait set.
  /**
   * Used to interrupt the wait set when adding or removing items.
   */
  void
  interrupt_waiting_wait_set()
  {
    extra_guard_conditions_[0]->trigger();
  }

  /// Add subscription.
  void
  sync_add_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> && subscription,
    const rclcpp::SubscriptionWaitSetMask & mask,
    std::function<
      void(std::shared_ptr<rclcpp::SubscriptionBase>&&, const rclcpp::SubscriptionWaitSetMask &)
    > add_subscription_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    add_subscription_function(std::move(subscription), mask);
  }

  /// Remove guard condition.
  void
  sync_remove_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> && subscription,
    const rclcpp::SubscriptionWaitSetMask & mask,
    std::function<
      void(std::shared_ptr<rclcpp::SubscriptionBase>&&, const rclcpp::SubscriptionWaitSetMask &)
    > remove_subscription_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    remove_subscription_function(std::move(subscription), mask);
  }

  /// Add guard condition.
  void
  sync_add_guard_condition(
    std::shared_ptr<rclcpp::GuardCondition> && guard_condition,
    std::function<void(std::shared_ptr<rclcpp::GuardCondition>&&)> add_guard_condition_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    add_guard_condition_function(std::move(guard_condition));
  }

  /// Remove guard condition.
  void
  sync_remove_guard_condition(
    std::shared_ptr<rclcpp::GuardCondition> && guard_condition,
    std::function<void(std::shared_ptr<rclcpp::GuardCondition>&&)> remove_guard_condition_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    remove_guard_condition_function(std::move(guard_condition));
  }

  /// Add timer.
  void
  sync_add_timer(
    std::shared_ptr<rclcpp::TimerBase> && timer,
    std::function<void(std::shared_ptr<rclcpp::TimerBase>&&)> add_timer_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    add_timer_function(std::move(timer));
  }

  /// Remove timer.
  void
  sync_remove_timer(
    std::shared_ptr<rclcpp::TimerBase> && timer,
    std::function<void(std::shared_ptr<rclcpp::TimerBase>&&)> remove_timer_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    remove_timer_function(std::move(timer));
  }

  /// Add client.
  void
  sync_add_client(
    std::shared_ptr<rclcpp::ClientBase> && client,
    std::function<void(std::shared_ptr<rclcpp::ClientBase>&&)> add_client_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    add_client_function(std::move(client));
  }

  /// Remove client.
  void
  sync_remove_client(
    std::shared_ptr<rclcpp::ClientBase> && client,
    std::function<void(std::shared_ptr<rclcpp::ClientBase>&&)> remove_client_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    remove_client_function(std::move(client));
  }

  /// Add service.
  void
  sync_add_service(
    std::shared_ptr<rclcpp::ServiceBase> && service,
    std::function<void(std::shared_ptr<rclcpp::ServiceBase>&&)> add_service_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    add_service_function(std::move(service));
  }

  /// Remove service.
  void
  sync_remove_service(
    std::shared_ptr<rclcpp::ServiceBase> && service,
    std::function<void(std::shared_ptr<rclcpp::ServiceBase>&&)> remove_service_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    remove_service_function(std::move(service));
  }

  /// Add waitable.
  void
  sync_add_waitable(
    std::shared_ptr<rclcpp::Waitable> && waitable,
    std::shared_ptr<void> && associated_entity,
    std::function<
      void(std::shared_ptr<rclcpp::Waitable>&&, std::shared_ptr<void>&&)
    > add_waitable_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    add_waitable_function(std::move(waitable), std::move(associated_entity));
  }

  /// Remove waitable.
  void
  sync_remove_waitable(
    std::shared_ptr<rclcpp::Waitable> && waitable,
    std::function<void(std::shared_ptr<rclcpp::Waitable>&&)> remove_waitable_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    remove_waitable_function(std::move(waitable));
  }

  /// Prune deleted entities.
  void
  sync_prune_deleted_entities(std::function<void()> prune_deleted_entities_function)
  {
    using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
    std::lock_guard<WritePreferringReadWriteLock::WriteMutex> lock(wprw_lock_.get_write_mutex());
    prune_deleted_entities_function();
  }

  /// Implements wait.
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
      {
        // We have to prevent the entity sets from being mutated while building
        // the rcl wait set.
        using rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock;
        std::lock_guard<WritePreferringReadWriteLock::ReadMutex> lock(wprw_lock_.get_read_mutex());
        // Rebuild the wait set.
        // This will resize the wait set if needed, due to e.g. adding or removing
        // entities since the last wait, but this should never occur in static
        // storage wait sets since they cannot be changed after construction.
        // This will also clear the wait set and re-add all the entities, which
        // prepares it to be waited on again.
        rebuild_rcl_wait_set();
      }

      rcl_wait_set_t & rcl_wait_set = get_rcl_wait_set();

      // Wait unconditionally until timeout condition occurs since we assume
      // there are no conditions that would require the wait to stop and reset,
      // like asynchronously adding or removing an entity, i.e. explicitly
      // providing no thread-safety.

      // Calculate how much time there is left to wait, unless blocking indefinitely.
      auto time_left_to_wait_ns = this->calculate_time_left_to_wait(time_to_wait_ns, start);

      // Then wait for entities to become ready.

      // It is ok to wait while not having the lock acquired, because the state
      // in the rcl wait set will not be updated until this method calls
      // rebuild_rcl_wait_set().
      rcl_ret_t ret = rcl_wait(&rcl_wait_set, time_left_to_wait_ns.count());
      if (RCL_RET_OK == ret) {
        // Something has become ready in the wait set, first check if it was
        // the guard condition added by this class and/or a user defined guard condition.
        const rcl_guard_condition_t * interrupt_guard_condition_ptr =
          &(extra_guard_conditions_[0]->get_rcl_guard_condition());
        bool was_interrupted_by_this_class = false;
        bool any_user_guard_conditions_triggered = false;
        for (size_t index = 0; index < rcl_wait_set.size_of_guard_conditions; ++index) {
          const rcl_guard_condition_t * current = rcl_wait_set.guard_conditions[index];
          if (nullptr != current) {
            // Something is ready.
            if (rcl_wait_set.guard_conditions[index] == interrupt_guard_condition_ptr) {
              // This means that this class triggered a guard condition to interrupt this wait.
              was_interrupted_by_this_class = true;
            } else {
              // This means it was a user guard condition.
              any_user_guard_conditions_triggered = true;
            }
          }
        }

        if (!was_interrupted_by_this_class || any_user_guard_conditions_triggered) {
          // In this case we know:
          //   - something was ready
          //   - it was either:
          //     - not interrupted by this class, or
          //     - maybe it was, but there were also user defined guard conditions.
          //
          // We cannot ignore user defined guard conditions, but we can ignore
          // other kinds of user defined entities, because they will still be
          // ready next time we wait, whereas guard conditions are cleared.
          // Therefore we need to create a WaitResult and return it.

          // The WaitResult will call sync_wait_result_acquire() and
          // sync_wait_result_release() to ensure thread-safety by preventing
          // the mutation of the entity sets while introspecting after waiting.
          return create_wait_result(WaitResultKind::Ready);
        }
        // If we get here the we interrupted the wait set and there were no user
        // guard conditions that needed to be handled.
        // So we will loop and it will re-acquire the lock and rebuild the
        // rcl wait set.
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
    wprw_lock_.get_read_mutex().lock();
  }

  void
  sync_wait_result_release()
  {
    wprw_lock_.get_read_mutex().unlock();
  }

protected:
  std::array<std::shared_ptr<rclcpp::GuardCondition>, 1> extra_guard_conditions_;
  rclcpp::wait_set_policies::detail::WritePreferringReadWriteLock wprw_lock_;
};

}  // namespace wait_set_policies
}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_POLICIES__THREAD_SAFE_SYNCHRONIZATION_HPP_
