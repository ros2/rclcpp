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

#include <chrono>
#include <exception>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/duration.hpp"
#include "rclcpp/node_interfaces/get_node_base_interface.hpp"
#include "rclcpp/node_interfaces/get_node_timers_interface.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/node_interfaces/node_timers_interface.hpp"

namespace rclcpp
{
/// Create a timer with a given clock
/// \internal
template<typename CallbackT>
typename rclcpp::TimerBase::SharedPtr
create_timer(
  std::shared_ptr<node_interfaces::NodeBaseInterface> node_base,
  std::shared_ptr<node_interfaces::NodeTimersInterface> node_timers,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Duration period,
  CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  auto timer = rclcpp::GenericTimer<CallbackT>::make_shared(
    clock,
    period.to_chrono<std::chrono::nanoseconds>(),
    std::forward<CallbackT>(callback),
    node_base->get_context());

  node_timers->add_timer(timer, group);
  return timer;
}

/// Create a timer with a given clock
template<typename NodeT, typename CallbackT>
typename rclcpp::TimerBase::SharedPtr
create_timer(
  NodeT node,
  rclcpp::Clock::SharedPtr clock,
  rclcpp::Duration period,
  CallbackT && callback,
  rclcpp::CallbackGroup::SharedPtr group = nullptr)
{
  return create_timer(
    rclcpp::node_interfaces::get_node_base_interface(node),
    rclcpp::node_interfaces::get_node_timers_interface(node),
    clock,
    period,
    std::forward<CallbackT>(callback),
    group);
}

/// Convenience method to create a timer with node resources.
/**
 *
 * \tparam DurationRepT
 * \tparam DurationT
 * \tparam CallbackT
 * \param period period to exectute callback
 * \param callback callback to execute via the timer period
 * \param group
 * \param node_base
 * \param node_timers
 * \return
 * \throws std::invalid argument if either node_base or node_timers
 * are null
 */
template<typename DurationRepT, typename DurationT, typename CallbackT>
typename rclcpp::WallTimer<CallbackT>::SharedPtr
create_wall_timer(
  std::chrono::duration<DurationRepT, DurationT> period,
  CallbackT callback,
  rclcpp::CallbackGroup::SharedPtr group,
  node_interfaces::NodeBaseInterface * node_base,
  node_interfaces::NodeTimersInterface * node_timers)
{
  if (node_base == nullptr) {
    throw std::invalid_argument{"input node_base cannot be null"};
  }

  if (node_timers == nullptr) {
    throw std::invalid_argument{"input node_timers cannot be null"};
  }

  auto timer = rclcpp::WallTimer<CallbackT>::make_shared(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::move(callback),
    node_base->get_context());
  node_timers->add_timer(timer, group);
  return timer;
}

}  // namespace rclcpp

#endif  // RCLCPP__CREATE_TIMER_HPP_
