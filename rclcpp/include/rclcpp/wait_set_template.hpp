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

#ifndef RCLCPP__WAIT_SET_TEMPLATE_HPP_
#define RCLCPP__WAIT_SET_TEMPLATE_HPP_

#include <chrono>
#include <memory>
#include <utility>

#include "rcl/wait.h"

#include "rclcpp/client.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/guard_condition.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/scope_exit.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/subscription_wait_set_mask.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/wait_result.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{

/// Encapsulates sets of waitable items which can be waited on as a group.
/**
 * This class uses the rcl_wait_set_t as storage, but it also helps manage the
 * ownership of associated rclcpp types.
 */
template<class SynchronizationPolicy, class StoragePolicy>
class WaitSetTemplate final : private SynchronizationPolicy, private StoragePolicy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(WaitSetTemplate)

  using typename StoragePolicy::SubscriptionEntry;
  using typename StoragePolicy::WaitableEntry;

  /// Construct a wait set with optional initial waitable entities and optional custom context.
  /**
   * For the waitables, they have additionally an "associated" entity, which
   * you can read more about in the add and remove functions for those types
   * in this class.
   *
   * \param[in] subscriptions Vector of subscriptions to be added.
   * \param[in] guard_conditions Vector of guard conditions to be added.
   * \param[in] timers Vector of timers to be added.
   * \param[in] clients Vector of clients and their associated entity to be added.
   * \param[in] services Vector of services and their associated entity to be added.
   * \param[in] waitables Vector of waitables and their associated entity to be added.
   * \param[in] context Custom context to be used, defaults to global default.
   * \throws std::invalid_argument If context is nullptr.
   */
  explicit
  WaitSetTemplate(
    const typename StoragePolicy::SubscriptionsIterable & subscriptions = {},
    const typename StoragePolicy::GuardConditionsIterable & guard_conditions = {},
    const typename StoragePolicy::TimersIterable & timers = {},
    const typename StoragePolicy::ClientsIterable & clients = {},
    const typename StoragePolicy::ServicesIterable & services = {},
    const typename StoragePolicy::WaitablesIterable & waitables = {},
    rclcpp::Context::SharedPtr context =
    rclcpp::contexts::get_global_default_context())
  : SynchronizationPolicy(context),
    StoragePolicy(
      subscriptions,
      guard_conditions,
      // this method comes from the SynchronizationPolicy
      this->get_extra_guard_conditions(),
      timers,
      clients,
      services,
      waitables,
      context)
  {}

  /// Return the internal rcl wait set object.
  /**
   * This method provides no thread-safety when accessing this structure.
   * The state of this structure can be updated at anytime by methods like
   * wait(), add_*(), remove_*(), etc.
   */
  const rcl_wait_set_t &
  get_rcl_wait_set() const
  {
    // this method comes from the StoragePolicy
    return this->storage_get_rcl_wait_set();
  }

  /// Add a subscription to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * Additionally to the documentation for add_guard_condition, this method
   * has a mask parameter which allows you to control which parts of the
   * subscription is added to the wait set with this call.
   * For example, you might want to include the actual subscription to this
   * wait set, but add the intra-process waitable to another wait set.
   * If intra-process is disabled, no error will occur, it will just be skipped.
   *
   * When introspecting after waiting, this subscription's shared pointer will
   * be the Waitable's (intra-process or the QoS Events) "associated entity"
   * pointer, for more easily figuring out which subscription which waitable
   * goes with afterwards.
   *
   * \param[in] subscription Subscription to be added.
   * \param[in] mask A class which controls which parts of the subscription to add.
   * \throws std::invalid_argument if subscription is nullptr.
   * \throws std::runtime_error if subscription has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void
  add_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> subscription,
    rclcpp::SubscriptionWaitSetMask mask = {})
  {
    if (nullptr == subscription) {
      throw std::invalid_argument("subscription is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_add_subscription(
      std::move(subscription),
      mask,
      [this](
        std::shared_ptr<rclcpp::SubscriptionBase> && inner_subscription,
        const rclcpp::SubscriptionWaitSetMask & mask)
      {
        // These methods comes from the StoragePolicy, and may not exist for
        // fixed sized storage policies.
        // It will throw if the subscription has already been added.
        if (mask.include_subscription) {
          auto local_subscription = inner_subscription;
          bool already_in_use =
          local_subscription->exchange_in_use_by_wait_set_state(local_subscription.get(), true);
          if (already_in_use) {
            throw std::runtime_error("subscription already associated with a wait set");
          }
          this->storage_add_subscription(std::move(local_subscription));
        }
        if (mask.include_events) {
          for (auto event : inner_subscription->get_event_handlers()) {
            auto local_subscription = inner_subscription;
            bool already_in_use =
            local_subscription->exchange_in_use_by_wait_set_state(event.get(), true);
            if (already_in_use) {
              throw std::runtime_error("subscription event already associated with a wait set");
            }
            this->storage_add_waitable(std::move(event), std::move(local_subscription));
          }
        }
        if (mask.include_intra_process_waitable) {
          auto local_subscription = inner_subscription;
          auto waitable = inner_subscription->get_intra_process_waitable();
          if (nullptr != waitable) {
            bool already_in_use = local_subscription->exchange_in_use_by_wait_set_state(
              waitable.get(),
              true);
            if (already_in_use) {
              throw std::runtime_error(
                "subscription intra-process waitable already associated with a wait set");
            }
            this->storage_add_waitable(
              std::move(inner_subscription->get_intra_process_waitable()),
              std::move(local_subscription));
          }
        }
      });
  }

  /// Remove a subscription from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * Additionally to the documentation for add_guard_condition, this method
   * has a mask parameter which allows you to control which parts of the
   * subscription is added to the wait set with this call.
   * You may remove items selectively from the wait set in a different order
   * than they were added.
   *
   * \param[in] subscription Subscription to be removed.
   * \param[in] mask A class which controls which parts of the subscription to remove.
   * \throws std::invalid_argument if subscription is nullptr.
   * \throws std::runtime_error if subscription is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void
  remove_subscription(
    std::shared_ptr<rclcpp::SubscriptionBase> subscription,
    rclcpp::SubscriptionWaitSetMask mask = {})
  {
    if (nullptr == subscription) {
      throw std::invalid_argument("subscription is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_remove_subscription(
      std::move(subscription),
      mask,
      [this](
        std::shared_ptr<rclcpp::SubscriptionBase> && inner_subscription,
        const rclcpp::SubscriptionWaitSetMask & mask)
      {
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the subscription is not in the wait set.
        if (mask.include_subscription) {
          auto local_subscription = inner_subscription;
          local_subscription->exchange_in_use_by_wait_set_state(local_subscription.get(), false);
          this->storage_remove_subscription(std::move(local_subscription));
        }
        if (mask.include_events) {
          for (auto event : inner_subscription->get_event_handlers()) {
            auto local_subscription = inner_subscription;
            local_subscription->exchange_in_use_by_wait_set_state(event.get(), false);
            this->storage_remove_waitable(std::move(event));
          }
        }
        if (mask.include_intra_process_waitable) {
          auto local_waitable = inner_subscription->get_intra_process_waitable();
          if (nullptr != local_waitable) {
            // This is the case when intra process is enabled for the subscription.
            inner_subscription->exchange_in_use_by_wait_set_state(local_waitable.get(), false);
            this->storage_remove_waitable(std::move(local_waitable));
          }
        }
      });
  }

  /// Add a guard condition to this wait set.
  /**
   * Guard condition is added to the wait set, and shared ownership is held
   * while waiting.
   * However, if between calls to wait() the guard condition's reference count
   * goes to zero, it will be implicitly removed on the next call to wait().
   *
   * Except in the case of a fixed sized storage, where changes to the wait set
   * cannot occur after construction, in which case it holds shared ownership
   * at all times until the wait set is destroy, but this method also does not
   * exist on a fixed sized wait set.
   *
   * This function may be thread-safe depending on the SynchronizationPolicy
   * used with this class.
   * Using the ThreadSafeWaitSetPolicy will ensure that wait() is interrupted
   * and returns before this function adds the guard condition.
   * Otherwise, it is not safe to call this function concurrently with wait().
   *
   * This function will not be enabled (will not be available) if the
   * StoragePolicy does not allow editing of the wait set after initialization.
   *
   * \param[in] guard_condition Guard condition to be added.
   * \throws std::invalid_argument if guard_condition is nullptr.
   * \throws std::runtime_error if guard_condition has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void
  add_guard_condition(std::shared_ptr<rclcpp::GuardCondition> guard_condition)
  {
    if (nullptr == guard_condition) {
      throw std::invalid_argument("guard_condition is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_add_guard_condition(
      std::move(guard_condition),
      [this](std::shared_ptr<rclcpp::GuardCondition> && inner_guard_condition) {
        bool already_in_use = inner_guard_condition->exchange_in_use_by_wait_set_state(true);
        if (already_in_use) {
          throw std::runtime_error("guard condition already in use by another wait set");
        }
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the guard condition has already been added.
        this->storage_add_guard_condition(std::move(inner_guard_condition));
      });
  }

  /// Remove a guard condition from this wait set.
  /**
   * Guard condition is removed from the wait set, and if needed the shared
   * ownership is released.
   *
   * This function may be thread-safe depending on the SynchronizationPolicy
   * used with this class.
   * Using the ThreadSafeWaitSetPolicy will ensure that wait() is interrupted
   * and returns before this function removes the guard condition.
   * Otherwise, it is not safe to call this function concurrently with wait().
   *
   * This function will not be enabled (will not be available) if the
   * StoragePolicy does not allow editing of the wait set after initialization.
   *
   * \param[in] guard_condition Guard condition to be removed.
   * \throws std::invalid_argument if guard_condition is nullptr.
   * \throws std::runtime_error if guard_condition is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void
  remove_guard_condition(std::shared_ptr<rclcpp::GuardCondition> guard_condition)
  {
    if (nullptr == guard_condition) {
      throw std::invalid_argument("guard_condition is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_remove_guard_condition(
      std::move(guard_condition),
      [this](std::shared_ptr<rclcpp::GuardCondition> && inner_guard_condition) {
        inner_guard_condition->exchange_in_use_by_wait_set_state(false);
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the guard condition is not in the wait set.
        this->storage_remove_guard_condition(std::move(inner_guard_condition));
      });
  }

  /// Add a timer to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * \param[in] timer Timer to be added.
   * \throws std::invalid_argument if timer is nullptr.
   * \throws std::runtime_error if timer has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void
  add_timer(std::shared_ptr<rclcpp::TimerBase> timer)
  {
    if (nullptr == timer) {
      throw std::invalid_argument("timer is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_add_timer(
      std::move(timer),
      [this](std::shared_ptr<rclcpp::TimerBase> && inner_timer) {
        bool already_in_use = inner_timer->exchange_in_use_by_wait_set_state(true);
        if (already_in_use) {
          throw std::runtime_error("timer already in use by another wait set");
        }
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the timer has already been added.
        this->storage_add_timer(std::move(inner_timer));
      });
  }

  /// Remove a timer from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] timer Timer to be removed.
   * \throws std::invalid_argument if timer is nullptr.
   * \throws std::runtime_error if timer is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void
  remove_timer(std::shared_ptr<rclcpp::TimerBase> timer)
  {
    if (nullptr == timer) {
      throw std::invalid_argument("timer is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_remove_timer(
      std::move(timer),
      [this](std::shared_ptr<rclcpp::TimerBase> && inner_timer) {
        inner_timer->exchange_in_use_by_wait_set_state(false);
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the timer is not in the wait set.
        this->storage_remove_timer(std::move(inner_timer));
      });
  }

  /// Add a client to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * \param[in] client Client to be added.
   * \throws std::invalid_argument if client is nullptr.
   * \throws std::runtime_error if client has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void
  add_client(std::shared_ptr<rclcpp::ClientBase> client)
  {
    if (nullptr == client) {
      throw std::invalid_argument("client is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_add_client(
      std::move(client),
      [this](std::shared_ptr<rclcpp::ClientBase> && inner_client) {
        bool already_in_use = inner_client->exchange_in_use_by_wait_set_state(true);
        if (already_in_use) {
          throw std::runtime_error("client already in use by another wait set");
        }
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the client has already been added.
        this->storage_add_client(std::move(inner_client));
      });
  }

  /// Remove a client from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] client Client to be removed.
   * \throws std::invalid_argument if client is nullptr.
   * \throws std::runtime_error if client is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void
  remove_client(std::shared_ptr<rclcpp::ClientBase> client)
  {
    if (nullptr == client) {
      throw std::invalid_argument("client is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_remove_client(
      std::move(client),
      [this](std::shared_ptr<rclcpp::ClientBase> && inner_client) {
        inner_client->exchange_in_use_by_wait_set_state(false);
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the client is not in the wait set.
        this->storage_remove_client(std::move(inner_client));
      });
  }

  /// Add a service to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * \param[in] service Service to be added.
   * \throws std::invalid_argument if service is nullptr.
   * \throws std::runtime_error if service has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void
  add_service(std::shared_ptr<rclcpp::ServiceBase> service)
  {
    if (nullptr == service) {
      throw std::invalid_argument("service is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_add_service(
      std::move(service),
      [this](std::shared_ptr<rclcpp::ServiceBase> && inner_service) {
        bool already_in_use = inner_service->exchange_in_use_by_wait_set_state(true);
        if (already_in_use) {
          throw std::runtime_error("service already in use by another wait set");
        }
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the service has already been added.
        this->storage_add_service(std::move(inner_service));
      });
  }

  /// Remove a service from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] service Service to be removed.
   * \throws std::invalid_argument if service is nullptr.
   * \throws std::runtime_error if service is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void
  remove_service(std::shared_ptr<rclcpp::ServiceBase> service)
  {
    if (nullptr == service) {
      throw std::invalid_argument("service is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_remove_service(
      std::move(service),
      [this](std::shared_ptr<rclcpp::ServiceBase> && inner_service) {
        inner_service->exchange_in_use_by_wait_set_state(false);
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the service is not in the wait set.
        this->storage_remove_service(std::move(inner_service));
      });
  }

  /// Add a waitable to this wait set.
  /**
   * \sa add_guard_condition() for details of how this method works.
   *
   * Additionally, this function has an optional parameter which can be used to
   * more quickly associate this waitable with an entity when it is ready, and
   * so that the ownership maybe held in order to keep the waitable's parent in
   * scope while waiting.
   * If it is set to nullptr it will be ignored.
   * The destruction of the associated entity's shared pointer will not cause
   * the waitable to be removed, but it will cause the associated entity pointer
   * to be nullptr when introspecting this waitable after waiting.
   *
   * Note that rclcpp::QOSEventHandlerBase are just a special case of
   * rclcpp::Waitable and can be added with this function.
   *
   * \param[in] waitable Waitable to be added.
   * \param[in] associated_entity Type erased shared pointer associated with the waitable.
   *   This may be nullptr.
   * \throws std::invalid_argument if waitable is nullptr.
   * \throws std::runtime_error if waitable has already been added or is
   *   associated with another wait set.
   * \throws exceptions based on the policies used.
   */
  void
  add_waitable(
    std::shared_ptr<rclcpp::Waitable> waitable,
    std::shared_ptr<void> associated_entity = nullptr)
  {
    if (nullptr == waitable) {
      throw std::invalid_argument("waitable is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_add_waitable(
      std::move(waitable),
      std::move(associated_entity),
      [this](
        std::shared_ptr<rclcpp::Waitable> && inner_waitable,
        std::shared_ptr<void> && associated_entity)
      {
        bool already_in_use = inner_waitable->exchange_in_use_by_wait_set_state(true);
        if (already_in_use) {
          throw std::runtime_error("waitable already in use by another wait set");
        }
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the waitable has already been added.
        this->storage_add_waitable(std::move(inner_waitable), std::move(associated_entity));
      });
  }

  /// Remove a waitable from this wait set.
  /**
   * \sa remove_guard_condition() for details of how this method works.
   *
   * \param[in] waitable Waitable to be removed.
   * \throws std::invalid_argument if waitable is nullptr.
   * \throws std::runtime_error if waitable is not part of the wait set.
   * \throws exceptions based on the policies used.
   */
  void
  remove_waitable(std::shared_ptr<rclcpp::Waitable> waitable)
  {
    if (nullptr == waitable) {
      throw std::invalid_argument("waitable is nullptr");
    }
    // this method comes from the SynchronizationPolicy
    this->sync_remove_waitable(
      std::move(waitable),
      [this](std::shared_ptr<rclcpp::Waitable> && inner_waitable) {
        inner_waitable->exchange_in_use_by_wait_set_state(false);
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        // It will throw if the waitable is not in the wait set.
        this->storage_remove_waitable(std::move(inner_waitable));
      });
  }

  /// Remove any destroyed entities from the wait set.
  /**
   * When the storage policy does not maintain shared ownership for the life
   * of the wait set, e.g. the DynamicStorage policy, it is possible for an
   * entity to go out of scope and be deleted without this wait set noticing.
   * Therefore there are weak references in this wait set which need to be
   * periodically cleared.
   * This function performs that clean up.
   *
   * Since this involves removing entities from the wait set, and is only
   * needed if the wait set does not keep ownership of the added entities, the
   * storage policies which are static will not need this function and therefore
   * do not provide this function.
   *
   * \throws exceptions based on the policies used.
   */
  void
  prune_deleted_entities()
  {
    // this method comes from the SynchronizationPolicy
    this->sync_prune_deleted_entities(
      [this]() {
        // This method comes from the StoragePolicy, and it may not exist for
        // fixed sized storage policies.
        this->storage_prune_deleted_entities();
      });
  }

  /// Wait for any of the entities in the wait set to be ready, or a period of time to pass.
  /**
   * This function will return when either one of the entities within this wait
   * set is ready, or a period of time has passed, which ever is first.
   * The term "ready" means different things for different entities, but
   * generally it means some condition is met asynchronously for which this
   * function waits.
   *
   * This function can either wait for a period of time, do no waiting
   * (non-blocking), or wait indefinitely, all based on the value of the
   * time_to_wait parameter.
   * Waiting is always measured against the std::chrono::steady_clock.
   * If waiting indefinitely, the Timeout result is not possible.
   * There is no "cancel wait" function on this class, but if you want to wait
   * indefinitely but have a way to asynchronously interrupt this method, then
   * you can use a dedicated rclcpp::GuardCondition for that purpose.
   *
   * This function will modify the internal rcl_wait_set_t, so introspecting
   * the wait set during a call to wait is never safe.
   * You should always wait, then introspect, and then, only when done
   * introspecting, wait again.
   *
   * It may be thread-safe to add and remove entities to the wait set
   * concurrently with this function, depending on the SynchronizationPolicy
   * that is used.
   * With the rclcpp::wait_set_policies::ThreadSafeSynchronization policy this
   * function will stop waiting to allow add or remove of an entity, and then
   * resume waiting, so long as the timeout has not been reached.
   *
   * \param[in] time_to_wait If > 0, time to wait for entities to be ready,
   *   if == 0, check if anything is ready without blocking, or
   *   if < 0, wait indefinitely until one of the items is ready.
   *   Default is -1, so wait indefinitely.
   * \returns Ready when one of the entities is ready, or
   * \returns Timeout when the given time to wait is exceeded, not possible
   *   when time_to_wait is < 0, or
   * \returns Empty if the wait set is empty, avoiding the possibility of
   *   waiting indefinitely on an empty wait set.
   * \throws rclcpp::exceptions::RCLError on unhandled rcl errors or,
   * \throws std::runtime_error if unknown WaitResultKind
   */
  template<class Rep = int64_t, class Period = std::milli>
  RCUTILS_WARN_UNUSED
  WaitResult<WaitSetTemplate>
  wait(std::chrono::duration<Rep, Period> time_to_wait = std::chrono::duration<Rep, Period>(-1))
  {
    auto time_to_wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(time_to_wait);

    // ensure the ownership of the entities in the wait set is shared for the duration of wait
    this->storage_acquire_ownerships();
    RCLCPP_SCOPE_EXIT({this->storage_release_ownerships();});

    // this method comes from the SynchronizationPolicy
    return this->template sync_wait<WaitResult<WaitSetTemplate>>(
      // pass along the time_to_wait duration as nanoseconds
      time_to_wait_ns,
      // this method provides the ability to rebuild the wait set, if needed
      [this]() {
        // This method comes from the StoragePolicy
        this->storage_rebuild_rcl_wait_set(
          // This method comes from the SynchronizationPolicy
          this->get_extra_guard_conditions()
        );
      },
      // this method provides access to the rcl wait set
      [this]() -> rcl_wait_set_t & {
        // This method comes from the StoragePolicy
        return this->storage_get_rcl_wait_set();
      },
      // this method provides a way to create the WaitResult
      [this](WaitResultKind wait_result_kind) -> WaitResult<WaitSetTemplate> {
        // convert the result into a WaitResult
        switch (wait_result_kind) {
          case WaitResultKind::Ready:
            return WaitResult<WaitSetTemplate>::from_ready_wait_result_kind(*this);
          case WaitResultKind::Timeout:
            return WaitResult<WaitSetTemplate>::from_timeout_wait_result_kind();
          case WaitResultKind::Empty:
            return WaitResult<WaitSetTemplate>::from_empty_wait_result_kind();
          default:
            auto msg = "unknown WaitResultKind with value: " + std::to_string(wait_result_kind);
            throw std::runtime_error(msg);
        }
      }
    );
  }

private:
  // Add WaitResult type as a friend so it can call private methods for
  // acquiring and releasing resources as the WaitResult is initialized and
  // destructed, respectively.
  friend WaitResult<WaitSetTemplate>;

  /// Called by the WaitResult's constructor to place a hold on ownership and thread-safety.
  /**
   * Should only be called in pairs with wait_result_release().
   *
   * \throws std::runtime_error If called twice before wait_result_release().
   */
  void
  wait_result_acquire()
  {
    if (wait_result_holding_) {
      throw std::runtime_error("wait_result_acquire() called while already holding");
    }
    wait_result_holding_ = true;
    // this method comes from the SynchronizationPolicy
    this->sync_wait_result_acquire();
    // this method comes from the StoragePolicy
    this->storage_acquire_ownerships();
  }

  /// Called by the WaitResult's destructor to release resources.
  /**
   * Should only be called if wait_result_acquire() has been called.
   *
   * \throws std::runtime_error If called before wait_result_acquire().
   */
  void
  wait_result_release()
  {
    if (!wait_result_holding_) {
      throw std::runtime_error("wait_result_release() called while not holding");
    }
    wait_result_holding_ = false;
    // this method comes from the StoragePolicy
    this->storage_release_ownerships();
    // this method comes from the SynchronizationPolicy
    this->sync_wait_result_release();
  }

  bool wait_result_holding_ = false;
};

}  // namespace rclcpp

#endif  // RCLCPP__WAIT_SET_TEMPLATE_HPP_
