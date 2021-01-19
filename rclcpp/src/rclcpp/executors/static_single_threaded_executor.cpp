// Copyright 2019 Nobleo Technology
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

#include "rclcpp/executors/static_single_threaded_executor.hpp"

#include <memory>
#include <vector>

#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::StaticSingleThreadedExecutor;
using rclcpp::experimental::ExecutableList;

StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(
  const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{
  entities_collector_ = std::make_shared<StaticExecutorEntitiesCollector>();
}

StaticSingleThreadedExecutor::~StaticSingleThreadedExecutor() {}

void
StaticSingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );

  // Set memory_strategy_ and exec_list_ based on weak_nodes_
  // Prepare wait_set_ based on memory_strategy_
  entities_collector_->init(&wait_set_, memory_strategy_, &interrupt_guard_condition_);
  RCLCPP_SCOPE_EXIT(entities_collector_->fini());

  while (rclcpp::ok(this->context_) && spinning.load()) {
    // Refresh wait set and wait for work
    entities_collector_->refresh_wait_set();
    execute_ready_executables();
  }
}

void
StaticSingleThreadedExecutor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  bool notify)
{
  bool is_new_node = entities_collector_->add_callback_group(group_ptr, node_ptr);
  if (is_new_node && notify) {
    // Interrupt waiting to handle new node
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }
}

void
StaticSingleThreadedExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  bool is_new_node = entities_collector_->add_node(node_ptr);
  if (is_new_node && notify) {
    // Interrupt waiting to handle new node
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }
}

void
StaticSingleThreadedExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
StaticSingleThreadedExecutor::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify)
{
  bool node_removed = entities_collector_->remove_callback_group(group_ptr);
  // If the node was matched and removed, interrupt waiting
  if (node_removed && notify) {
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }
}

void
StaticSingleThreadedExecutor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  bool node_removed = entities_collector_->remove_node(node_ptr);
  if (!node_removed) {
    throw std::runtime_error("Node needs to be associated with this executor.");
  }
  // If the node was matched and removed, interrupt waiting
  if (notify) {
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticSingleThreadedExecutor::get_all_callback_groups()
{
  return entities_collector_->get_all_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticSingleThreadedExecutor::get_manually_added_callback_groups()
{
  return entities_collector_->get_manually_added_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
StaticSingleThreadedExecutor::get_automatically_added_callback_groups_from_nodes()
{
  return entities_collector_->get_automatically_added_callback_groups_from_nodes();
}

void
StaticSingleThreadedExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
StaticSingleThreadedExecutor::execute_ready_executables()
{
  // Execute all the ready subscriptions
  for (size_t i = 0; i < wait_set_.size_of_subscriptions; ++i) {
    if (i < entities_collector_->get_number_of_subscriptions()) {
      if (wait_set_.subscriptions[i]) {
        execute_subscription(entities_collector_->get_subscription(i));
      }
    }
  }
  // Execute all the ready timers
  for (size_t i = 0; i < wait_set_.size_of_timers; ++i) {
    if (i < entities_collector_->get_number_of_timers()) {
      if (wait_set_.timers[i] && entities_collector_->get_timer(i)->is_ready()) {
        execute_timer(entities_collector_->get_timer(i));
      }
    }
  }
  // Execute all the ready services
  for (size_t i = 0; i < wait_set_.size_of_services; ++i) {
    if (i < entities_collector_->get_number_of_services()) {
      if (wait_set_.services[i]) {
        execute_service(entities_collector_->get_service(i));
      }
    }
  }
  // Execute all the ready clients
  for (size_t i = 0; i < wait_set_.size_of_clients; ++i) {
    if (i < entities_collector_->get_number_of_clients()) {
      if (wait_set_.clients[i]) {
        execute_client(entities_collector_->get_client(i));
      }
    }
  }
  // Execute all the ready waitables
  for (size_t i = 0; i < entities_collector_->get_number_of_waitables(); ++i) {
    auto waitable = entities_collector_->get_waitable(i);
    if (waitable->is_ready(&wait_set_)) {
      auto data = waitable->take_data();
      waitable->execute(data);
    }
  }
}
