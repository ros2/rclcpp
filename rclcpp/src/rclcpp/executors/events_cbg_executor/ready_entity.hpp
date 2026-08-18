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

#include <utility>

#include "scheduler.hpp"
#include "global_event_id_provider.hpp"
#include "rclcpp/executors/events_cbg_executor/events_cbg_executor.hpp"

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
struct ReadyEntity
{
  struct ReadyTimerWithExecutedCallback
  {
    rclcpp::TimerBase::WeakPtr timer_ptr;
        // must be called by the after executing the timer callback
    std::function<void()> timer_was_executed;

    bool expired() const
    {
      return timer_ptr.expired();
    }
  };

  std::variant<rclcpp::SubscriptionBase::WeakPtr, ReadyTimerWithExecutedCallback,
    rclcpp::ServiceBase::WeakPtr, rclcpp::ClientBase::WeakPtr, CBGScheduler::WaitableWithEventType,
    CBGScheduler::CallbackEventType> entity;

  explicit ReadyEntity(const rclcpp::SubscriptionBase::WeakPtr ptr)
  : entity(ptr), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const ReadyTimerWithExecutedCallback & timer)
  : entity(timer), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const rclcpp::ServiceBase::WeakPtr ptr)
  : entity(ptr), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const rclcpp::ClientBase::WeakPtr ptr)
  : entity(ptr), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const CBGScheduler::WaitableWithEventType & ev)
  : entity(ev), id(GlobalEventIdProvider::get_next_id()) {}
  explicit ReadyEntity(const CBGScheduler::CallbackEventType & ev)
  : entity(ev), id(GlobalEventIdProvider::get_next_id()) {}

  std::function<void()> get_execute_function() const
  {
    return std::visit([](auto && entity) -> std::function<void()> {
               using T = std::decay_t<decltype(entity)>;
               if constexpr (std::is_same_v<T, rclcpp::SubscriptionBase::WeakPtr>) {
                 rclcpp::SubscriptionBase::SharedPtr shr_ptr = entity.lock();
                 if (!shr_ptr) {
                   return std::function<void()>();
                 }
                 return [shr_ptr = std::move(shr_ptr)]() {
                          rclcpp::executors::EventsCBGExecutor::execute_subscription(shr_ptr);
                        };
               } else if constexpr (std::is_same_v<T, ReadyTimerWithExecutedCallback>) {
                 auto shr_ptr = entity.timer_ptr.lock();
                 if (!shr_ptr) {
                   return std::function<void()>();
                 }

                 return [shr_ptr = std::move(shr_ptr),
                        timer_executed_cb = entity.timer_was_executed]() {
                          auto data = shr_ptr->call();
                          if (!data) {
                              // timer was cancelled, skip it.
                            return;
                          }

                          rclcpp::executors::EventsCBGExecutor::execute_timer(shr_ptr, data);

                          // readd the timer to the timers manager
                          timer_executed_cb();
                        };
               } else if constexpr (std::is_same_v<T, rclcpp::ServiceBase::WeakPtr>) {
                 auto shr_ptr = entity.lock();
                 if (!shr_ptr) {
                   return std::function<void()>();
                 }
                 return [shr_ptr = std::move(shr_ptr)]() {
                          rclcpp::executors::EventsCBGExecutor::execute_service(shr_ptr);
                        };
               } else if constexpr (std::is_same_v<T, rclcpp::ClientBase::WeakPtr>) {
                 auto shr_ptr = entity.lock();
                 if (!shr_ptr) {
                   return std::function<void()>();
                 }
                 return [shr_ptr = std::move(shr_ptr)]() {
                          rclcpp::executors::EventsCBGExecutor::execute_client(shr_ptr);
                        };
               } else if constexpr (std::is_same_v<T, CBGScheduler::WaitableWithEventType>) {
                 auto shr_ptr_in = entity.waitable.lock();
                 if (!shr_ptr_in) {
                   return std::function<void()>();
                 }

                 return [shr_ptr = std::move(shr_ptr_in),
                        event_type = entity.internal_event_type]() {
                          auto data = shr_ptr->take_data_by_entity_id(event_type);
                          shr_ptr->execute(data);
                        };
               } else if constexpr (std::is_same_v<T, CBGScheduler::CallbackEventType>) {
                 return entity.callback;
               }
        }, entity);
  }

  GlobalEventIdProvider::MonotonicId id;

    /**
     * Returns true if the event has expired / does not need to be executed any more
     */
  bool expired() const
  {
    return std::visit([](const auto & entity) {return entity.expired();}, entity);
  }
};
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
