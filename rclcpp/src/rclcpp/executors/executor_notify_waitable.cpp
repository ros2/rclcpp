// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <iostream>

#include "rclcpp/executors/executor_notify_waitable.hpp"
#include "rclcpp/detail/add_guard_condition_to_rcl_wait_set.hpp"

namespace rclcpp
{
namespace executors
{

ExecutorNotifyWaitable::ExecutorNotifyWaitable(std::function<void(void)> on_execute_callback)
: execute_callback_(on_execute_callback)
{
}

void
ExecutorNotifyWaitable::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  for (auto guard_condition : this->notify_guard_conditions_) {
    detail::add_guard_condition_to_rcl_wait_set(*wait_set, *guard_condition);
  }
}

bool
ExecutorNotifyWaitable::is_ready(rcl_wait_set_t * wait_set)
{
  for (size_t ii = 0; ii < wait_set->size_of_guard_conditions; ++ii) {
    auto rcl_guard_condition = wait_set->guard_conditions[ii];

    if (nullptr == rcl_guard_condition) {
      continue;
    }

    for (auto guard_condition : this->notify_guard_conditions_) {
      if (&guard_condition->get_rcl_guard_condition() == rcl_guard_condition) {
        return true;
      }
    }
  }

  return false;
}

void
ExecutorNotifyWaitable::execute(std::shared_ptr<void> & data)
{
  (void) data;
  this->execute_callback_();
}

std::shared_ptr<void>
ExecutorNotifyWaitable::take_data()
{
  return nullptr;
}

void
ExecutorNotifyWaitable::add_guard_condition(rclcpp::GuardCondition * guard_condition)
{
  if (guard_condition == nullptr) {
    throw std::runtime_error("Attempting to add null guard condition.");
  }

  for (const auto & existing_guard_condition : notify_guard_conditions_) {
    if (existing_guard_condition == guard_condition) {
      return;
    }
  }
  notify_guard_conditions_.push_back(guard_condition);
}

void
ExecutorNotifyWaitable::remove_guard_condition(rclcpp::GuardCondition * guard_condition)
{
  if (guard_condition == nullptr) {
    throw std::runtime_error("Attempting to remove null guard condition.");
  }

  for (auto it = notify_guard_conditions_.begin(); it != notify_guard_conditions_.end(); ++it) {
    if (*it == guard_condition) {
      notify_guard_conditions_.erase(it);
      break;
    }
  }
}

size_t
ExecutorNotifyWaitable::get_number_of_ready_guard_conditions()
{
  return notify_guard_conditions_.size();
}

}  // namespace executors
}  // namespace rclcpp
