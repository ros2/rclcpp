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

#include <deque>
#include <memory>

#include <vector>

#include "ready_entity.hpp"
#include "scheduler.hpp"
#include "global_event_id_provider.hpp"

namespace rclcpp
{
namespace executors
{
namespace cbg_executor
{
struct FirstInFirstOutCallbackGroupHandle final : public CBGScheduler::CallbackGroupHandle
{
public:
  explicit FirstInFirstOutCallbackGroupHandle(CBGScheduler & scheduler)
  : CallbackGroupHandle(scheduler) {}

  std::function<void(size_t)> get_ready_callback_for_entity(
    const rclcpp::SubscriptionBase::WeakPtr & entity) final;
  std::function<void(std::function<void()> executed_callback)> get_ready_callback_for_entity(
    const rclcpp::TimerBase::WeakPtr & entity) final;
  std::function<void(size_t)> get_ready_callback_for_entity(
    const rclcpp::ClientBase::WeakPtr & entity) final;
  std::function<void(size_t)> get_ready_callback_for_entity(
    const rclcpp::ServiceBase::WeakPtr & entity) final;
  std::function<void(size_t,
    int)> get_ready_callback_for_entity(const rclcpp::Waitable::WeakPtr & entity) final;
  std::function<void(size_t)> get_ready_callback_for_entity(
    const CBGScheduler::CallbackEventType & entity) final;

  std::optional<CBGScheduler::ExecutableEntity> get_next_ready_entity();
  std::optional<CBGScheduler::ExecutableEntity> get_next_ready_entity(
    GlobalEventIdProvider::MonotonicId max_id);

  bool has_ready_entities() const final
  {
    return !ready_entities.empty();
  }

private:
  std::deque<ReadyEntity> ready_entities;
};

class FirstInFirstOutScheduler : public CBGScheduler
{
public:
  using CBGScheduler::CBGScheduler;

  ExecutableEntityWithInfo get_next_ready_entity_intern() final;
  ExecutableEntityWithInfo get_next_ready_entity_intern(
    GlobalEventIdProvider::MonotonicId max_id) final;

private:
  std::unique_ptr<CallbackGroupHandle> get_handle_for_callback_group(
    const rclcpp::CallbackGroup::SharedPtr & callback_group) final;

  std::vector<std::unique_ptr<FirstInFirstOutCallbackGroupHandle>> callback_group_handles;
};
}  // namespace cbg_executor
}  // namespace executors
}  // namespace rclcpp
