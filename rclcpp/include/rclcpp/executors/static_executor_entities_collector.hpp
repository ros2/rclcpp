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

#ifndef RCLCPP__EXECUTORS__STATIC_EXECUTOR_ENTITIES_COLLECTOR_HPP_
#define RCLCPP__EXECUTORS__STATIC_EXECUTOR_ENTITIES_COLLECTOR_HPP_

#include <memory>

#include <rcl/guard_condition.h>
#include <rcl/wait.h>

#include <rclcpp/executable_list.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/memory_strategy.hpp>
#include <rclcpp/visibility_control.hpp>
#include <rclcpp/waitable.hpp>

namespace rclcpp
{
namespace executors
{

class StaticExecutorEntitiesCollector final
  : public rclcpp::Waitable,
  public std::enable_shared_from_this<StaticExecutorEntitiesCollector>
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(StaticExecutorEntitiesCollector)

  // Constructor
  RCLCPP_PUBLIC
  StaticExecutorEntitiesCollector() = default;

  // Destructor
  ~StaticExecutorEntitiesCollector();

  RCLCPP_PUBLIC
  void
  init(
    rcl_wait_set_t * p_wait_set,
    rclcpp::memory_strategy::MemoryStrategy::SharedPtr & memory_strategy,
    rcl_guard_condition_t * executor_guard_condition);

  RCLCPP_PUBLIC
  void
  execute() override;

  RCLCPP_PUBLIC
  void
  fill_memory_strategy();

  RCLCPP_PUBLIC
  void
  fill_executable_list();

  /// Function to reallocate space for entities in the wait set.
  RCLCPP_PUBLIC
  void
  prepare_wait_set();

  /// Function to add_handles_to_wait_set and wait for work and
  // block until the wait set is ready or until the timeout has been exceeded.
  RCLCPP_PUBLIC
  void
  refresh_wait_set(std::chrono::nanoseconds timeout = std::chrono::nanoseconds(-1));

  RCLCPP_PUBLIC
  bool
  add_to_wait_set(rcl_wait_set_t * wait_set) override;

  RCLCPP_PUBLIC
  size_t
  get_number_of_ready_guard_conditions() override;

  RCLCPP_PUBLIC
  void
  add_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  RCLCPP_PUBLIC
  bool
  remove_node(
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr);

  /// Complete all available queued work without blocking.
  /**
   * This function checks if after the guard condition was triggered
   * (or a spurious wakeup happened) we are really ready to execute
   * i.e. re-collect entities
   */
  RCLCPP_PUBLIC
  bool
  is_ready(rcl_wait_set_t * wait_set) override;

  RCLCPP_PUBLIC
  size_t
  get_number_of_timers() {return exec_list_.number_of_timers;}

  RCLCPP_PUBLIC
  size_t
  get_number_of_subscriptions() {return exec_list_.number_of_subscriptions;}

  RCLCPP_PUBLIC
  size_t
  get_number_of_services() {return exec_list_.number_of_services;}

  RCLCPP_PUBLIC
  size_t
  get_number_of_clients() {return exec_list_.number_of_clients;}

  RCLCPP_PUBLIC
  size_t
  get_number_of_waitables() {return exec_list_.number_of_waitables;}

  RCLCPP_PUBLIC
  rclcpp::SubscriptionBase::SharedPtr
  get_subscription(size_t i) {return exec_list_.subscription[i];}

  RCLCPP_PUBLIC
  rclcpp::TimerBase::SharedPtr
  get_timer(size_t i) {return exec_list_.timer[i];}

  RCLCPP_PUBLIC
  rclcpp::ServiceBase::SharedPtr
  get_service(size_t i) {return exec_list_.service[i];}

  RCLCPP_PUBLIC
  rclcpp::ClientBase::SharedPtr
  get_client(size_t i) {return exec_list_.client[i];}

  RCLCPP_PUBLIC
  rclcpp::Waitable::SharedPtr
  get_waitable(size_t i) {return exec_list_.waitable[i];}

private:
  /// Nodes guard conditions which trigger this waitable
  std::list<const rcl_guard_condition_t *> guard_conditions_;

  /// Memory strategy: an interface for handling user-defined memory allocation strategies.
  memory_strategy::MemoryStrategy::SharedPtr memory_strategy_;

  /// List of weak nodes registered in the static executor
  std::list<rclcpp::node_interfaces::NodeBaseInterface::WeakPtr> weak_nodes_;

  /// Wait set for managing entities that the rmw layer waits on.
  rcl_wait_set_t * p_wait_set_ = nullptr;

  /// Executable list: timers, subscribers, clients, services and waitables
  rclcpp::executor::ExecutableList exec_list_;
};

}  // namespace executors
}  // namespace rclcpp

#endif  // RCLCPP__EXECUTORS__STATIC_EXECUTOR_ENTITIES_COLLECTOR_HPP_
