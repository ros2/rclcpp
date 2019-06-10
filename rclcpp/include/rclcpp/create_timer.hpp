// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__CREATE_TIMER_HPP_
#define RCLCPP__CREATE_TIMER_HPP_

#include <memory>
#include <string>
#include <utility>

#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"
#include "rclcpp/duration.hpp"


namespace rclcpp
{
/// Create a timer with a given type.
template<typename CallbackT>
typename rclcpp::TimerBase::SharedPtr
create_timer(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeTimersInterface> node_timers,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Duration period,
  CallbackT && callback,
  rclcpp::callback_group::CallbackGroup::SharedPtr group = nullptr)
{
  auto timer = rclcpp::GenericTimer<CallbackT>::make_shared(
    clock,
    period.to_chrono<std::chrono::nanoseconds>(),
    std::forward<CallbackT &&>(callback),
    node_base->get_context());

  node_timers->add_timer(timer, group);
  return timer;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_TIMER_HPP_
