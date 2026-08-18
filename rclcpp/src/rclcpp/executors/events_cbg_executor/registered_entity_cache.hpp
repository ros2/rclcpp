// Copyright 2024 Cellumation GmbH.
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

#pragma once

#include <unordered_map>
#include <utility>
#include <memory>
#include <vector>

#include "scheduler.hpp"
#include "timer_manager.hpp"
#include <rclcpp/executors/executor_entities_collection.hpp>

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
template<class EntityType_T>
struct WeakEntityPtrWithRemoveFunction
{
  WeakEntityPtrWithRemoveFunction(
    const std::shared_ptr<EntityType_T> & shr_ptr,
    const std::function<void(const std::shared_ptr<EntityType_T> & ptr)> & destruction_callback)
  :executable(shr_ptr),
    ptr(shr_ptr.get()),
    destruction_callback(destruction_callback)
  {
  }

  ~WeakEntityPtrWithRemoveFunction()
  {
    std::shared_ptr<EntityType_T> shr_ptr = executable.lock();
    if(shr_ptr) {
      destruction_callback(shr_ptr);
    }
  }

  std::weak_ptr<EntityType_T> executable;

  // May be used as key, to identify the removed element
  // after the weak pointer went out of scope
  EntityType_T *ptr;

  std::function<void(const std::shared_ptr<EntityType_T> & ptr)> destruction_callback;
};

struct GuardConditionWithFunction
{
  GuardConditionWithFunction(rclcpp::GuardCondition::SharedPtr gc, std::function<void(void)> fun)
  : guard_condition(std::move(gc)), handle_guard_condition_fun(std::move(fun))
  {
  }

  rclcpp::GuardCondition::SharedPtr guard_condition;

  // A function that should be executed if the guard_condition is ready
  std::function<void(void)> handle_guard_condition_fun;
};

template<class EntityType_T>
class EntityCache
{
  using CacheType = WeakEntityPtrWithRemoveFunction<EntityType_T>;

  std::unordered_map<const EntityType_T *, std::unique_ptr<CacheType>> entities;

public:
  void update(
    const std::vector<std::shared_ptr<EntityType_T>> & entityList,
    const std::function<void(const std::shared_ptr<EntityType_T> &)> & on_add,
    const std::function<void(const std::shared_ptr<EntityType_T> &)> & on_remove)
  {
    std::unordered_map<const EntityType_T *, std::unique_ptr<CacheType>> nextEntities;
    for(const std::shared_ptr<EntityType_T> & shr_ptr : entityList) {
      auto it = entities.find(shr_ptr.get());
      if(it != entities.end()) {
        nextEntities.insert(std::move(entities.extract(it)));
      } else {
        // new entry
        nextEntities.emplace(std::make_pair(shr_ptr.get(),
              std::make_unique<CacheType>(shr_ptr, on_remove)));

        on_add(shr_ptr);
      }
    }

    entities.swap(nextEntities);

    // NOTE, at this point the non moved unique_ptrs get destructed, and the
    // remove callbacks are called
  }

  void clear()
  {
    entities.clear();
  }
};

struct RegisteredEntityCache
{
  RegisteredEntityCache(
    CBGScheduler & scheduler, TimerManager & timer_manager,
    const rclcpp::CallbackGroup::SharedPtr & callback_group)
  : callback_group_weak_ptr(callback_group),
    scheduler_cbg_handle(*scheduler.add_callback_group(callback_group)),
    timer_manager(timer_manager)
  {
    auto cbg_gc = callback_group->get_notify_guard_condition();

    if(cbg_gc) {
      // register guard condition for the callback group with the handler
      // this makes sure, that we pick up new entities in case the guard
      // condition is triggered
      add_guard_condition_event(
        cbg_gc, [&scheduler]() {
          scheduler.trigger_sync();
        }
      );
    }
  }

  EntityCache<rclcpp::TimerBase> timers_cache;
  EntityCache<rclcpp::SubscriptionBase> subscribers_cache;
  EntityCache<rclcpp::ClientBase> clients_cache;
  EntityCache<rclcpp::ServiceBase> services_cache;
  EntityCache<rclcpp::Waitable> waitables_cache;

  std::vector<GuardConditionWithFunction> guard_conditions;

  rclcpp::CallbackGroup::WeakPtr callback_group_weak_ptr;
  CBGScheduler::CallbackGroupHandle & scheduler_cbg_handle;

  TimerManager & timer_manager;

  ~RegisteredEntityCache()
  {
    for (const auto & gc_ref : guard_conditions) {
      gc_ref.guard_condition->set_on_trigger_callback(nullptr);
    }

    auto cbg_shr_ptr = callback_group_weak_ptr.lock();
    if(!cbg_shr_ptr) {
      return;
    }

    const auto clear_sub_cb = [](const rclcpp::SubscriptionBase::SharedPtr & s) {
        s->clear_on_new_message_callback();
      };
    const auto clear_timer_cb = [this](const rclcpp::TimerBase::SharedPtr & s) {
        timer_manager.remove_timer(s);
      };
    const auto clear_client_cb = [](const rclcpp::ClientBase::SharedPtr & s) {
        s->clear_on_new_response_callback();
      };
    const auto clear_service_cb = [](const rclcpp::ServiceBase::SharedPtr & s) {
        s->clear_on_new_request_callback();
      };
    const auto clear_waitable_cb = [](const rclcpp::Waitable::SharedPtr & s) {
        s->clear_on_ready_callback();
      };

    // populate all vectors
    cbg_shr_ptr->collect_all_ptrs(clear_sub_cb, clear_service_cb, clear_client_cb, clear_timer_cb,
            clear_waitable_cb);

    if(cbg_shr_ptr->get_notify_guard_condition()) {
      cbg_shr_ptr->get_notify_guard_condition()->set_on_trigger_callback(nullptr);
    }
  }

  void clear_caches()
  {
    timers_cache.clear();
    subscribers_cache.clear();
    clients_cache.clear();
    services_cache.clear();
    waitables_cache.clear();
  }


  bool regenerate_events()
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = callback_group_weak_ptr.lock();

    if(!callback_group) {
      clear_caches();
      return false;
    }

    std::vector<rclcpp::TimerBase::SharedPtr> timers;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers;
    std::vector<rclcpp::ClientBase::SharedPtr> clients;
    std::vector<rclcpp::ServiceBase::SharedPtr> services;
    std::vector<rclcpp::Waitable::SharedPtr> waitables;

    // we reserve to much memory here, but this should be fine
    const size_t max_size = callback_group->size();
    timers.reserve(max_size);
    subscribers.reserve(max_size);
    clients.reserve(max_size);
    services.reserve(max_size);
    waitables.reserve(max_size);

    const auto add_sub = [&subscribers](const rclcpp::SubscriptionBase::SharedPtr & s) {
        subscribers.push_back(s);
      };
    const auto add_timer = [&timers](const rclcpp::TimerBase::SharedPtr & s) {
        timers.push_back(s);
      };
    const auto add_client = [&clients](const rclcpp::ClientBase::SharedPtr & s) {
        clients.push_back(s);
      };
    const auto add_service = [&services](const rclcpp::ServiceBase::SharedPtr & s) {
        services.push_back(s);
      };
    const auto add_waitable = [&waitables](const rclcpp::Waitable::SharedPtr & s) {
        waitables.push_back(s);
      };

    // populate all vectors
    callback_group->collect_all_ptrs(add_sub, add_service, add_client, add_timer, add_waitable);

    timers_cache.update(timers,
      [this](const rclcpp::TimerBase::SharedPtr & s) {
        timer_manager.add_timer(s, scheduler_cbg_handle.get_ready_callback_for_entity(s));
      },
      [this](const rclcpp::TimerBase::SharedPtr & s) {
        timer_manager.remove_timer(s);
    });

    subscribers_cache.update(subscribers,
      [this](const rclcpp::SubscriptionBase::SharedPtr & s) {
        s->set_on_new_message_callback(
          scheduler_cbg_handle.get_ready_callback_for_entity(s));
      },
      [] (const rclcpp::SubscriptionBase::SharedPtr & shr_ptr) {
        shr_ptr->clear_on_new_message_callback();
    });

    clients_cache.update(clients,
      [this](const rclcpp::ClientBase::SharedPtr & s) {
        s->set_on_new_response_callback(
          scheduler_cbg_handle.get_ready_callback_for_entity(s));
      },
      [] (const rclcpp::ClientBase::SharedPtr & shr_ptr) {
        shr_ptr->clear_on_new_response_callback();
    });
    services_cache.update(services,
      [this](const rclcpp::ServiceBase::SharedPtr & s) {
        s->set_on_new_request_callback(
          scheduler_cbg_handle.get_ready_callback_for_entity(s));
      },
      [] (const rclcpp::ServiceBase::SharedPtr & shr_ptr) {
        shr_ptr->clear_on_new_request_callback();
    });
    waitables_cache.update(waitables,
      [this](const rclcpp::Waitable::SharedPtr & s) {
        s->set_on_ready_callback(
          scheduler_cbg_handle.get_ready_callback_for_entity(s));
        for (const auto & t : s->get_timers()) {
          timer_manager.add_timer(t, scheduler_cbg_handle.get_ready_callback_for_entity(t));
        }
      },
      [this] (const rclcpp::Waitable::SharedPtr & s) {
        s->clear_on_ready_callback();
        for (const auto & t : s->get_timers()) {
          timer_manager.remove_timer(t);
        }
    });

    return true;
  }

  /**
   * Register special case guard conditions.
   * These guard conditions can be registered to a callback, that will
   * be executed within the executor worker context on trigger.
   *
   * Note, there is currently no way, to deregister these guard conditions
   */
  void add_guard_condition_event(
    rclcpp::GuardCondition::SharedPtr ptr,
    std::function<void(void)> fun)
  {
    auto & new_entry = guard_conditions.emplace_back(std::move(ptr), std::move(fun));

    if (new_entry.handle_guard_condition_fun) {
      new_entry.guard_condition->set_on_trigger_callback(
        scheduler_cbg_handle.get_ready_callback_for_entity(CBGScheduler::CallbackEventType(
              new_entry.handle_guard_condition_fun)));
    }
  }
};
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
