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

#include "rclcpp/exceptions.hpp"
#include "rclcpp/executors/executor_notify_waitable.hpp"

namespace rclcpp
{
namespace executors
{

ExecutorNotifyWaitable::ExecutorNotifyWaitable(
  std::function<void(void)> on_execute_callback,
  const rclcpp::Context::SharedPtr & context)
: execute_callback_(on_execute_callback),
  guard_condition_(std::make_shared<rclcpp::GuardCondition>(context))
{
}

ExecutorNotifyWaitable::ExecutorNotifyWaitable(ExecutorNotifyWaitable & other)
{
  std::lock_guard<std::mutex> lock(other.guard_condition_mutex_);
  this->execute_callback_ = other.execute_callback_;
  this->notify_guard_conditions_ = other.notify_guard_conditions_;
  this->guard_condition_ = other.guard_condition_;
  this->idxs_of_added_guard_condition_ = other.idxs_of_added_guard_condition_;
  this->needs_processing = other.needs_processing;
}

ExecutorNotifyWaitable & ExecutorNotifyWaitable::operator=(ExecutorNotifyWaitable & other)
{
  if (this != &other) {
    std::lock_guard<std::mutex> lock(other.guard_condition_mutex_);
    this->execute_callback_ = other.execute_callback_;
    this->notify_guard_conditions_ = other.notify_guard_conditions_;
    this->guard_condition_ = other.guard_condition_;
    this->idxs_of_added_guard_condition_ = other.idxs_of_added_guard_condition_;
    this->needs_processing = other.needs_processing;
  }
  return *this;
}

void
ExecutorNotifyWaitable::add_to_wait_set(rcl_wait_set_t & wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  idxs_of_added_guard_condition_.clear();
  idxs_of_added_guard_condition_.reserve(notify_guard_conditions_.size());

  if(needs_processing) {
    rcl_guard_condition_t * cond = &guard_condition_->get_rcl_guard_condition();
    size_t rcl_index;
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(&wait_set, cond, &rcl_index);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to add guard condition to wait set");
    }

    idxs_of_added_guard_condition_.push_back(rcl_index);

    // we want to directly wake up any way, not need to add the other guard conditions
    guard_condition_->trigger();

    return;
  }

  // Note: no guard conditions need to be re-triggered, since the guard
  // conditions in this class are not tracking a stateful condition, but instead
  // only serve to interrupt the wait set when new information is available to
  // consider.
  for (const auto & guard_condition : this->notify_guard_conditions_) {
    rcl_guard_condition_t * cond = &guard_condition->get_rcl_guard_condition();
    size_t rcl_index;
    rcl_ret_t ret = rcl_wait_set_add_guard_condition(&wait_set, cond, &rcl_index);

    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(
        ret, "failed to add guard condition to wait set");
    }

    idxs_of_added_guard_condition_.push_back(rcl_index);
  }
}

bool
ExecutorNotifyWaitable::is_ready(const rcl_wait_set_t & wait_set)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  bool any_ready = false;
  for (size_t rcl_index : idxs_of_added_guard_condition_) {
    if(rcl_index >= wait_set.size_of_guard_conditions) {
      throw std::runtime_error(
            "ExecutorNotifyWaitable::is_ready: Internal error, got index out of range");
    }

    const auto * rcl_guard_condition = wait_set.guard_conditions[rcl_index];

    if (nullptr == rcl_guard_condition) {
      continue;
    }

    any_ready = true;
    needs_processing = true;
    break;
  }

  return any_ready;
}

void
ExecutorNotifyWaitable::execute(const std::shared_ptr<void> & /*data*/)
{
  std::lock_guard<std::mutex> lock(execute_mutex_);

  needs_processing = false;

  this->execute_callback_();
}

std::shared_ptr<void>
ExecutorNotifyWaitable::take_data()
{
  return nullptr;
}

std::shared_ptr<void>
ExecutorNotifyWaitable::take_data_by_entity_id([[maybe_unused]] size_t id)
{
  return nullptr;
}

void
ExecutorNotifyWaitable::set_on_ready_callback(std::function<void(size_t, int)> callback)
{
  // The second argument of the callback could be used to identify which guard condition
  // triggered the event.
  // We could indicate which of the guard conditions was triggered, but the executor
  // is already going to check that.
  auto gc_callback = [callback](size_t count) {
      callback(count, 0);
    };

  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  on_ready_callback_ = gc_callback;
  for (const auto & gc : notify_guard_conditions_) {
    gc->set_on_trigger_callback(on_ready_callback_);
  }
}

RCLCPP_PUBLIC
void
ExecutorNotifyWaitable::clear_on_ready_callback()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);

  on_ready_callback_ = nullptr;
  for (const auto & gc : notify_guard_conditions_) {
    gc->set_on_trigger_callback(nullptr);
  }
}

RCLCPP_PUBLIC
void
ExecutorNotifyWaitable::set_execute_callback(std::function<void(void)> on_execute_callback)
{
  std::lock_guard<std::mutex> lock(execute_mutex_);
  execute_callback_ = on_execute_callback;
}

void
ExecutorNotifyWaitable::add_guard_condition(rclcpp::GuardCondition::WeakPtr weak_guard_condition)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  const auto & guard_condition = weak_guard_condition.lock();
  if (guard_condition && notify_guard_conditions_.count(guard_condition) == 0) {
    notify_guard_conditions_.insert(guard_condition);
    if (on_ready_callback_) {
      guard_condition->set_on_trigger_callback(on_ready_callback_);
    }
  }
}

void
ExecutorNotifyWaitable::remove_guard_condition(rclcpp::GuardCondition::WeakPtr weak_guard_condition)
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  const auto & guard_condition = weak_guard_condition.lock();
  if (!guard_condition) {
    // If the lock did not work, the guard condition can't be
    // saved in the sets, therefore we don't need to remove it
    return;
  }
  auto it = notify_guard_conditions_.find(guard_condition);
  if (it != notify_guard_conditions_.end()) {
    notify_guard_conditions_.erase(it);
    // If this notify waitable doesn't have an on_ready_callback, then there's nothing to unset
    if (on_ready_callback_) {
      guard_condition->set_on_trigger_callback(nullptr);
    }
  }
}

size_t
ExecutorNotifyWaitable::get_number_of_ready_guard_conditions()
{
  std::lock_guard<std::mutex> lock(guard_condition_mutex_);
  return notify_guard_conditions_.size();
}

std::vector<std::shared_ptr<rclcpp::TimerBase>>
ExecutorNotifyWaitable::get_timers() const
{
  return {};
}


}  // namespace executors
}  // namespace rclcpp
