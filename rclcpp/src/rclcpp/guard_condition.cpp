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

#include "rclcpp/guard_condition.hpp"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logging.hpp"

namespace rclcpp
{

GuardCondition::GuardCondition(rclcpp::Context::SharedPtr context)
: context_(context), rcl_guard_condition_{rcl_get_zero_initialized_guard_condition()}
{
  if (!context_) {
    throw std::invalid_argument("context argument unexpectedly nullptr");
  }
  rcl_guard_condition_options_t guard_condition_options = rcl_guard_condition_get_default_options();
  rcl_ret_t ret = rcl_guard_condition_init(
    &this->rcl_guard_condition_,
    context_->get_rcl_context().get(),
    guard_condition_options);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
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
        "Error in destruction of rcl guard condition: %s", exception.what());
    }
  }
}

rclcpp::Context::SharedPtr
GuardCondition::get_context() const
{
  return context_;
}

const rcl_guard_condition_t &
GuardCondition::get_rcl_guard_condition() const
{
  return rcl_guard_condition_;
}

void
GuardCondition::trigger()
{
  rcl_ret_t ret = rcl_trigger_guard_condition(&rcl_guard_condition_);
  if (RCL_RET_OK != ret) {
    rclcpp::exceptions::throw_from_rcl_error(ret);
  }
}

bool
GuardCondition::exchange_in_use_by_wait_set_state(bool in_use_state)
{
  return in_use_by_wait_set_.exchange(in_use_state);
}

}  // namespace rclcpp
