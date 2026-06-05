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
#include "first_in_first_out_scheduler.hpp"
#include <utility>

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
std::function<void(size_t)> FirstInFirstOutCallbackGroupHandle::get_ready_callback_for_entity(
  const rclcpp::SubscriptionBase::WeakPtr & entity)
{
  return [weak_ptr = entity, this](size_t nr_msg) {
           add_ready_entity([&] () {
               for (size_t i = 0; i < nr_msg; i++) {
                 ready_entities.emplace_back(weak_ptr);
               }
            });
         };
}

std::function<void(std::function<void()> executed_callback)> FirstInFirstOutCallbackGroupHandle::
get_ready_callback_for_entity(const rclcpp::TimerBase::WeakPtr & entity)
{
  return [weak_ptr = entity, this](std::function<void()> executed_callback) {
           add_ready_entity([&] () {
               ready_entities.emplace_back(ReadyEntity::ReadyTimerWithExecutedCallback{weak_ptr,
                 executed_callback});
            });
         };
}

std::function<void(size_t)> FirstInFirstOutCallbackGroupHandle::get_ready_callback_for_entity(
  const rclcpp::ClientBase::WeakPtr & entity)
{
  return [weak_ptr = entity, this](size_t nr_msg) {
           add_ready_entity([&] () {
               for (size_t i = 0; i < nr_msg; i++) {
                 ready_entities.emplace_back(weak_ptr);
               }
            });
         };
}

std::function<void(size_t)> FirstInFirstOutCallbackGroupHandle::get_ready_callback_for_entity(
  const rclcpp::ServiceBase::WeakPtr & entity)
{
  return [weak_ptr = entity, this](size_t nr_msg) {
           add_ready_entity([&] () {
               for (size_t i = 0; i < nr_msg; i++) {
                 ready_entities.emplace_back(weak_ptr);
               }
            });
         };
}

std::function<void(size_t,
  int)> FirstInFirstOutCallbackGroupHandle::get_ready_callback_for_entity(
  const rclcpp::Waitable::WeakPtr & entity)
{
  return [weak_ptr = entity, this](size_t nr_msg, int event_type) {
           add_ready_entity([&] () {
               for (size_t i = 0; i < nr_msg; i++) {
                 ready_entities.emplace_back(CBGScheduler::WaitableWithEventType({weak_ptr,
                   event_type}));
               }
            });
         };
}
std::function<void(size_t)> FirstInFirstOutCallbackGroupHandle::get_ready_callback_for_entity(
  const CBGScheduler::CallbackEventType & entity)
{
  return [weak_ptr = entity, this](size_t nr_msg) {
           add_ready_entity([&] () {
               for (size_t i = 0; i < nr_msg; i++) {
                 ready_entities.emplace_back(weak_ptr);
               }
            });
         };
}

std::optional<CBGScheduler::ExecutableEntity> FirstInFirstOutCallbackGroupHandle::
get_next_ready_entity()
{
  std::lock_guard l(ready_mutex);

  while(!ready_entities.empty()) {
    auto & first = ready_entities.front();

    std::function<void()> exec_fun = first.get_execute_function();
    ready_entities.pop_front();
    if(!exec_fun) {
      // was deleted, or in case of timer was canceled
      continue;
    }

    mark_as_executing();

    return CBGScheduler::ExecutableEntity{exec_fun, this};
  }

  mark_as_skipped();

  return std::nullopt;
}

std::optional<CBGScheduler::ExecutableEntity> FirstInFirstOutCallbackGroupHandle::
get_next_ready_entity(GlobalEventIdProvider::MonotonicId max_id)
{
  std::lock_guard l(ready_mutex);

  while(!ready_entities.empty()) {
    auto & first = ready_entities.front();
    if(first.id > max_id) {
      return std::nullopt;
    }

    std::function<void()> exec_fun = first.get_execute_function();
    ready_entities.pop_front();
    if(!exec_fun) {
      // was deleted, or in case of timer was canceled
      continue;
    }

    mark_as_executing();

    return CBGScheduler::ExecutableEntity{exec_fun, this};
  }

  mark_as_skipped();

  return std::nullopt;
}

std::unique_ptr<FirstInFirstOutScheduler::CallbackGroupHandle> FirstInFirstOutScheduler::
get_handle_for_callback_group(const rclcpp::CallbackGroup::SharedPtr &/*callback_group*/)
{
  return std::make_unique<FirstInFirstOutCallbackGroupHandle>(*this);
}

CBGScheduler::ExecutableEntityWithInfo FirstInFirstOutScheduler::get_next_ready_entity_intern()
{
  std::lock_guard l(ready_callback_groups_mutex);

  while(!ready_callback_groups.empty()) {
    FirstInFirstOutCallbackGroupHandle *ready_cbg =
      static_cast<FirstInFirstOutCallbackGroupHandle *>(ready_callback_groups.front());
    ready_callback_groups.pop_front();

    std::optional<FirstInFirstOutScheduler::ExecutableEntity> ret =
      ready_cbg->get_next_ready_entity();
    if(ret) {
      return CBGScheduler::ExecutableEntityWithInfo{.entity = std::move(ret),
        .moreEntitiesReady = !ready_callback_groups.empty()};
    }
  }

  return CBGScheduler::ExecutableEntityWithInfo{.entity = std::nullopt,
    .moreEntitiesReady = false};
}

CBGScheduler::ExecutableEntityWithInfo FirstInFirstOutScheduler::get_next_ready_entity_intern(
  GlobalEventIdProvider::MonotonicId max_id)
{
  std::lock_guard l(ready_callback_groups_mutex);

  // as, we remove an reappend ready callback_groups during execution,
  // the first ready cbg may not contain the lowest id. Therefore we
  // need to search the whole deque
  for(auto it = ready_callback_groups.begin(); it != ready_callback_groups.end(); it++) {
    FirstInFirstOutCallbackGroupHandle *ready_cbg(
      static_cast<FirstInFirstOutCallbackGroupHandle *>(*it));
    std::optional<FirstInFirstOutScheduler::ExecutableEntity> ret =
      ready_cbg->get_next_ready_entity(max_id);
    if(ret) {
      ready_callback_groups.erase(it);
      return CBGScheduler::ExecutableEntityWithInfo{.entity = std::move(ret),
        .moreEntitiesReady = !ready_callback_groups.empty()};
    }
  }

  return CBGScheduler::ExecutableEntityWithInfo{.entity = std::nullopt,
    .moreEntitiesReady = false};
}
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
