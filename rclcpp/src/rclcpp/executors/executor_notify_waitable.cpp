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

ExecutorNotifyWaitable::ExecutorNotifyWaitable(const ExecutorNotifyWaitable & other)
: ExecutorNotifyWaitable(other.execute_callback_)
{
  this->notify_guard_conditions_ = other.notify_guard_conditions_;
}

ExecutorNotifyWaitable & ExecutorNotifyWaitable::operator=(const ExecutorNotifyWaitable & other)
{
  if (this != &other) {
    this->execute_callback_ = other.execute_callback_;
    this->notify_guard_conditions_ = other.notify_guard_conditions_;
  }
  return *this;
}

void
ExecutorNotifyWaitable::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  for (auto weak_guard_condition : this->notify_guard_conditions_) {
    auto guard_condition = weak_guard_condition.lock();
    if (!guard_condition) {continue;}

    rcl_guard_condition_t * cond = &guard_condition->get_rcl_guard_condition();

    rcl_ret_t ret = rcl_wait_set_add_guard_condition(
      wait_set, cond, NULL);

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

  bool any_ready = false;
  for (size_t ii = 0; ii < wait_set->size_of_guard_conditions; ++ii) {
    auto rcl_guard_condition = wait_set->guard_conditions[ii];

    if (nullptr == rcl_guard_condition) {
      continue;
    }
    for (auto weak_guard_condition : this->notify_guard_conditions_) {
      auto guard_condition = weak_guard_condition.lock();
      if (guard_condition && &guard_condition->get_rcl_guard_condition() == rcl_guard_condition) {
        any_ready = true;
      }
    }
  }
  return any_ready;
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
ExecutorNotifyWaitable::add_guard_condition(rclcpp::GuardCondition::WeakPtr weak_guard_condition)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  auto guard_condition = weak_guard_condition.lock();
  if (guard_condition && notify_guard_conditions_.count(weak_guard_condition) == 0) {
    notify_guard_conditions_.insert(weak_guard_condition);
  }
}

void
ExecutorNotifyWaitable::remove_guard_condition(rclcpp::GuardCondition::WeakPtr weak_guard_condition)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  if (notify_guard_conditions_.count(weak_guard_condition) != 0) {
    notify_guard_conditions_.erase(weak_guard_condition);
  }
}

size_t
ExecutorNotifyWaitable::get_number_of_ready_guard_conditions()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  return notify_guard_conditions_.size();
}

}  // namespace executors
}  // namespace rclcpp
