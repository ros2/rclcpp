// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#ifndef RCLCPP__MEMORY_STRATEGY_HPP_
#define RCLCPP__MEMORY_STRATEGY_HPP_

#include <list>
#include <memory>

#include "rcl/allocator.h"
#include "rcl/wait.h"

#include "rclcpp/any_executable.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node_interfaces/node_base_interface.hpp"
#include "rclcpp/visibility_control.hpp"
#include "rclcpp/waitable.hpp"

namespace rclcpp
{
namespace memory_strategy
{

/// Delegate for handling memory allocations while the Executor is executing.
/**
 * By default, the memory strategy dynamically allocates memory for structures that come in from
 * the rmw implementation after the executor waits for work, based on the number of entities that
 * come through.
 */
class RCLCPP_PUBLIC MemoryStrategy
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS_NOT_COPYABLE(MemoryStrategy)
  using WeakNodeList = std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr>;

  virtual ~MemoryStrategy() = default;

  virtual bool collect_entities(const WeakNodeList & weak_nodes) = 0;

  virtual size_t number_of_ready_subscriptions() const = 0;
  virtual size_t number_of_ready_services() const = 0;
  virtual size_t number_of_ready_clients() const = 0;
  virtual size_t number_of_ready_events() const = 0;
  virtual size_t number_of_ready_timers() const = 0;
  virtual size_t number_of_guard_conditions() const = 0;
  virtual size_t number_of_waitables() const = 0;

  virtual void add_waitable_handle(const rclcpp::Waitable::SharedPtr & waitable) = 0;
  virtual bool add_handles_to_wait_set(rcl_wait_set_t * wait_set) = 0;
  virtual void clear_handles() = 0;
  virtual void remove_null_handles(rcl_wait_set_t * wait_set) = 0;

  virtual void add_guard_condition(const rcl_guard_condition_t * guard_condition) = 0;

  virtual void remove_guard_condition(const rcl_guard_condition_t * guard_condition) = 0;

  virtual void
  get_next_subscription(
    rclcpp::AnyExecutable & any_exec,
    const WeakNodeList & weak_nodes) = 0;

  virtual void
  get_next_service(
    rclcpp::AnyExecutable & any_exec,
    const WeakNodeList & weak_nodes) = 0;

  virtual void
  get_next_client(
    rclcpp::AnyExecutable & any_exec,
    const WeakNodeList & weak_nodes) = 0;

  virtual void
  get_next_timer(
    rclcpp::AnyExecutable & any_exec,
    const WeakNodeList & weak_nodes) = 0;

  virtual void
  get_next_waitable(
    rclcpp::AnyExecutable & any_exec,
    const WeakNodeList & weak_nodes) = 0;

  virtual rcl_allocator_t
  get_allocator() = 0;

  static rclcpp::SubscriptionBase::SharedPtr
  get_subscription_by_handle(
    std::shared_ptr<const rcl_subscription_t> subscriber_handle,
    const WeakNodeList & weak_nodes);

  static rclcpp::ServiceBase::SharedPtr
  get_service_by_handle(
    std::shared_ptr<const rcl_service_t> service_handle,
    const WeakNodeList & weak_nodes);

  static rclcpp::ClientBase::SharedPtr
  get_client_by_handle(
    std::shared_ptr<const rcl_client_t> client_handle,
    const WeakNodeList & weak_nodes);

  static rclcpp::TimerBase::SharedPtr
  get_timer_by_handle(
    std::shared_ptr<const rcl_timer_t> timer_handle,
    const WeakNodeList & weak_nodes);

  static rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_by_group(
    rclcpp::CallbackGroup::SharedPtr group,
    const WeakNodeList & weak_nodes);

  static rclcpp::CallbackGroup::SharedPtr
  get_group_by_subscription(
    rclcpp::SubscriptionBase::SharedPtr subscription,
    const WeakNodeList & weak_nodes);

  static rclcpp::CallbackGroup::SharedPtr
  get_group_by_service(
    rclcpp::ServiceBase::SharedPtr service,
    const WeakNodeList & weak_nodes);

  static rclcpp::CallbackGroup::SharedPtr
  get_group_by_client(
    rclcpp::ClientBase::SharedPtr client,
    const WeakNodeList & weak_nodes);

  static rclcpp::CallbackGroup::SharedPtr
  get_group_by_timer(
    rclcpp::TimerBase::SharedPtr timer,
    const WeakNodeList & weak_nodes);

  static rclcpp::CallbackGroup::SharedPtr
  get_group_by_waitable(
    rclcpp::Waitable::SharedPtr waitable,
    const WeakNodeList & weak_nodes);
};

}  // namespace memory_strategy
}  // namespace rclcpp

#endif  // RCLCPP__MEMORY_STRATEGY_HPP_
