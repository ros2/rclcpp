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
#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::StaticSingleThreadedExecutor;
using rclcpp::executor::ExecutableList;

StaticSingleThreadedExecutor::StaticSingleThreadedExecutor(
  const rclcpp::executor::ExecutorArgs & args)
: executor::Executor(args) {}

StaticSingleThreadedExecutor::~StaticSingleThreadedExecutor() {}


void
StaticSingleThreadedExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );
  rclcpp::executor::ExecutableList executable_list;
  run_collect_entities();
  get_executable_list(executable_list);
  while (rclcpp::ok(this->context_) && spinning.load()) {
    execute_ready_executables(executable_list);
  }
}

void
StaticSingleThreadedExecutor::get_timer_list(ExecutableList & exec_list)
{
  // Clear the previous timers (if any) from the ExecutableList struct
  exec_list.timer.clear();
  exec_list.number_of_timers = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      group->find_timer_ptrs_if([&exec_list](const rclcpp::TimerBase::SharedPtr & timer) {
          if(timer){
            // If any timer is found, push it in the exec_list struct
            exec_list.timer.push_back(timer);
            exec_list.number_of_timers++;
          }
          return false;
        });
    }
  }
}

void
StaticSingleThreadedExecutor::get_subscription_list(ExecutableList & exec_list)
{
  // Clear the previous subscriptions (if any) from the ExecutableList struct
  exec_list.subscription.clear();
  exec_list.number_of_subscriptions = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      group->find_subscription_ptrs_if([&exec_list](
        const rclcpp::SubscriptionBase::SharedPtr & subscription) {
          if(subscription){
            // If any subscription (intra-process as well) is found, push it in the exec_list struct
            exec_list.subscription.push_back(subscription);
            exec_list.number_of_subscriptions++;
          }
          return false;
        });
    }
  }
}

void
StaticSingleThreadedExecutor::get_service_list(ExecutableList & exec_list)
{
  // Clear the previous services (if any) from the ExecutableList struct
  exec_list.service.clear();
  exec_list.number_of_services = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      group->find_service_ptrs_if([&exec_list](const rclcpp::ServiceBase::SharedPtr & service) {
          if(service){
            // If any service is found, push it in the exec_list struct
            exec_list.service.push_back(service);
            exec_list.number_of_services++;
          }
          return false;
        });
    }
  }
}

void
StaticSingleThreadedExecutor::get_client_list(ExecutableList & exec_list)
{
  // Clear the previous clients (if any) from the ExecutableList struct
  exec_list.client.clear();
  exec_list.number_of_clients = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      group->find_client_ptrs_if([&exec_list](const rclcpp::ClientBase::SharedPtr & client) {
          if(client){
            // If any client is found, push it in the exec_list struct
            exec_list.client.push_back(client);
            exec_list.number_of_clients++;
          }
          return false;
        });
    }
  }
}

void
StaticSingleThreadedExecutor::get_waitable_list(ExecutableList & exec_list)
{
  // Clear the previous waitables (if any) from the ExecutableList struct
  exec_list.waitable.clear();
  exec_list.number_of_waitables = 0;
  for (auto & weak_node : weak_nodes_) {
    auto node = weak_node.lock();
    if (!node) {
      continue;
    }
    // Check in all the callback groups
    for (auto & weak_group : node->get_callback_groups()) {
      auto group = weak_group.lock();
      if (!group || !group->can_be_taken_from().load()) {
        continue;
      }
      group->find_waitable_ptrs_if([&exec_list](const rclcpp::Waitable::SharedPtr & waitable) {
          if(waitable){
            // If any waitable is found, push it in the exec_list struct
            exec_list.waitable.push_back(waitable);
            exec_list.number_of_waitables++;
          }
            return false;
        });
    }
  }
}

void
StaticSingleThreadedExecutor::get_executable_list(
  ExecutableList & executable_list, std::chrono::nanoseconds timeout)
{
  // prepare the wait_set
  prepare_wait_set();

  // add handles to the wait_set and wait for work
  refresh_wait_set(timeout);

  // Get all the timers
  get_timer_list(executable_list);

  // Get all the subscribers
  get_subscription_list(executable_list);

  // Get all the services
  get_service_list(executable_list);

  // Get all the clients
  get_client_list(executable_list);

  // Get all the waitables
  get_waitable_list(executable_list);
}

void
StaticSingleThreadedExecutor::execute_ready_executables(
  ExecutableList & exec_list, std::chrono::nanoseconds timeout)
{
 refresh_wait_set(timeout);
  // Execute all the ready subscriptions
  for (size_t i = 0; i < wait_set_.size_of_subscriptions; ++i) {
    if (i < exec_list.number_of_subscriptions) {
      if (wait_set_.subscriptions[i]) {
        execute_subscription(exec_list.subscription[i]);
      }
    }
  }
  // Execute all the ready timers
  for (size_t i = 0; i < wait_set_.size_of_timers; ++i) {
    if (i < exec_list.number_of_timers) {
      if (wait_set_.timers[i] && exec_list.timer[i]->is_ready()) {
        execute_timer(exec_list.timer[i]);
      }
    }
  }
  // Execute all the ready services
  for (size_t i = 0; i < wait_set_.size_of_services; ++i) {
    if (i < exec_list.number_of_services) {
      if (wait_set_.services[i]) {
        execute_service(exec_list.service[i]);
      }
    }
  }
  // Execute all the ready clients
  for (size_t i = 0; i < wait_set_.size_of_clients; ++i) {
    if (i < exec_list.number_of_clients) {
      if (wait_set_.clients[i]) {
        execute_client(exec_list.client[i]);
      }
    }
  }
  // Execute all the ready waitables
  for (size_t i = 0; i < exec_list.number_of_waitables; ++i) {
    if (exec_list.waitable[i]->is_ready(&wait_set_)) {
      exec_list.waitable[i]->execute();
    }
  }
  // Check the guard_conditions to see if a new entity was added to a node
  for (size_t i = 0; i < wait_set_.size_of_guard_conditions; ++i) {
    if (wait_set_.guard_conditions[i]) {
      // Check if the guard condition triggered belongs to a node
      auto it = std::find(guard_conditions_.begin(), guard_conditions_.end(),
                            wait_set_.guard_conditions[i]);

      // If it does, re-collect entities
      if (it != guard_conditions_.end()) {
        run_collect_entities();
        get_executable_list(exec_list);
        break;
      }
    }
  }
}

void
StaticSingleThreadedExecutor::run_collect_entities()
{
  memory_strategy_->clear_handles();
  bool has_invalid_weak_nodes = memory_strategy_->collect_entities(weak_nodes_);

  // Clean up any invalid nodes, if they were detected
  if (has_invalid_weak_nodes) {
    auto node_it = weak_nodes_.begin();
    auto gc_it = guard_conditions_.begin();
    while (node_it != weak_nodes_.end()) {
      if (node_it->expired()) {
        node_it = weak_nodes_.erase(node_it);
        memory_strategy_->remove_guard_condition(*gc_it);
        gc_it = guard_conditions_.erase(gc_it);
      } else {
        ++node_it;
        ++gc_it;
      }
    }
  }
}

void
StaticSingleThreadedExecutor::prepare_wait_set()
{
  // clear wait set
  if (rcl_wait_set_clear(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }

  // The size of waitables are accounted for in size of the other entities
  rcl_ret_t ret = rcl_wait_set_resize(
    &wait_set_, memory_strategy_->number_of_ready_subscriptions(),
    memory_strategy_->number_of_guard_conditions(), memory_strategy_->number_of_ready_timers(),
    memory_strategy_->number_of_ready_clients(), memory_strategy_->number_of_ready_services(),
    memory_strategy_->number_of_ready_events());

  if (RCL_RET_OK != ret) {
    throw std::runtime_error(
            std::string("Couldn't resize the wait set : ") + rcl_get_error_string().str);
  }
}

void
StaticSingleThreadedExecutor::refresh_wait_set(std::chrono::nanoseconds timeout)
{
  // clear wait set
  if (rcl_wait_set_clear(&wait_set_) != RCL_RET_OK) {
    throw std::runtime_error("Couldn't clear wait set");
  }
  // add handles to the wait_set
  if (!memory_strategy_->add_handles_to_wait_set(&wait_set_)) {
    throw std::runtime_error("Couldn't fill wait set");
  }
  rcl_ret_t status =
    rcl_wait(&wait_set_, std::chrono::duration_cast<std::chrono::nanoseconds>(timeout).count());
  if (status == RCL_RET_WAIT_SET_EMPTY) {
    RCUTILS_LOG_WARN_NAMED(
      "rclcpp",
      "empty wait set received in rcl_wait(). This should never happen.");
  } else if (status != RCL_RET_OK && status != RCL_RET_TIMEOUT) {
    using rclcpp::exceptions::throw_from_rcl_error;
    throw_from_rcl_error(status, "rcl_wait() failed");
  }
}
