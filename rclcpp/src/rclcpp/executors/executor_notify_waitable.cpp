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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/executors/executor_notify_waitable.hpp"

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
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  for (auto guard_condition : this->notify_guard_conditions_) {
    auto rcl_guard_condition = guard_condition->get_rcl_guard_condition();

    rcl_ret_t ret = rcl_wait_set_add_guard_condition(
      wait_set,
      &rcl_guard_condition, NULL);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to add guard condition to wait set");
    }
  }
}

bool
ExecutorNotifyWaitable::is_ready(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
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
ExecutorNotifyWaitable::add_guard_condition(const rclcpp::GuardCondition * guard_condition)
{
  if (guard_condition == nullptr) {
    throw std::runtime_error("Attempting to add null guard condition.");
  }
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  to_add_.push_back(guard_condition);
}

void
ExecutorNotifyWaitable::remove_guard_condition(const rclcpp::GuardCondition * guard_condition)
{
  if (guard_condition == nullptr) {
    throw std::runtime_error("Attempting to remove null guard condition.");
  }
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  to_remove_.push_back(guard_condition);
}

size_t
ExecutorNotifyWaitable::get_number_of_ready_guard_conditions()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  for (auto add_guard_condition : to_add_) {
    auto guard_it = std::find(
      notify_guard_conditions_.begin(),
      notify_guard_conditions_.end(),
      add_guard_condition);
    if (guard_it == notify_guard_conditions_.end()) {
      notify_guard_conditions_.push_back(add_guard_condition);
    }
  }
  to_add_.clear();

  for (auto remove_guard_condition : to_remove_) {
    auto guard_it = std::find(
      notify_guard_conditions_.begin(),
      notify_guard_conditions_.end(),
      remove_guard_condition);
    if (guard_it != notify_guard_conditions_.end()) {
      notify_guard_conditions_.erase(guard_it);
    }
  }
  to_remove_.clear();

  return notify_guard_conditions_.size();
}

}  // namespace executors
}  // namespace rclcpp
