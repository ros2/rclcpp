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

#include <functional>

#include "rclcpp/guard_condition.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

GuardCondition::GuardCondition(
  rclcpp::Context::SharedPtr context,
  rcl_guard_condition_options_t guard_condition_options)
: context_(context), rcl_guard_condition_{rcl_get_zero_initialized_guard_condition()}
{
  if (!context_) {
    throw std::invalid_argument("context argument unexpectedly nullptr");
  }
  rcl_ret_t ret = rcl_guard_condition_init(
    &this->rcl_guard_condition_,
    context_->get_rcl_context().get(),
    guard_condition_options);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret, "failed to create guard condition");
  }
}

GuardCondition::~GuardCondition()
{
  rcl_ret_t ret = rcl_guard_condition_fini(&this->rcl_guard_condition_);
  if (RCL_RET_OK != ret) {
    try {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "failed to finalize guard condition: %s", exception.what());
    }
  }
}

rclcpp::Context::SharedPtr
GuardCondition::get_context() const
{
  return context_;
}

rcl_guard_condition_t &
GuardCondition::get_rcl_guard_condition()
{
  return rcl_guard_condition_;
}

const rcl_guard_condition_t &
GuardCondition::get_rcl_guard_condition() const
{
  return rcl_guard_condition_;
}

void
GuardCondition::trigger()
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  if (on_trigger_callback_) {
    on_trigger_callback_(1);
  } else {
    rcl_ret_t ret = rcl_trigger_guard_condition(&rcl_guard_condition_);
    if (RCL_RET_OK != ret) {
      rclcpp::exceptions::throw_from_rcl_error(ret);
    }
    unread_count_++;
  }
}

bool
GuardCondition::exchange_in_use_by_wait_set_state(bool in_use_state)
{
  return in_use_by_wait_set_.exchange(in_use_state);
}

void
GuardCondition::add_to_wait_set(rcl_wait_set_t * wait_set)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  if (exchange_in_use_by_wait_set_state(true)) {
    if (wait_set != wait_set_) {
      throw std::runtime_error("guard condition has already been added to a wait set.");
    }
  } else {
    wait_set_ = wait_set;
  }

  rcl_ret_t ret = rcl_wait_set_add_guard_condition(wait_set, &this->rcl_guard_condition_, NULL);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(
      ret, "failed to add guard condition to wait set");
  }
}

void
GuardCondition::set_on_trigger_callback(std::function<void(size_t)> callback)
{
  std::lock_guard<std::recursive_mutex> lock(reentrant_mutex_);

  if (callback) {
    on_trigger_callback_ = callback;

    if (unread_count_) {
      callback(unread_count_);
      unread_count_ = 0;
    }
    return;
  }

  on_trigger_callback_ = nullptr;
}

}  // namespace rclcpp
