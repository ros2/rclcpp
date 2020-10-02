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

#include "rclcpp/executors/events_executor.hpp"

#include <memory>
#include <iomanip>

#include "rclcpp/scope_exit.hpp"

using rclcpp::executors::EventsExecutor;
using rclcpp::experimental::ExecutableList;

EventsExecutor::EventsExecutor(
  const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{
  entities_collector_ = std::make_shared<StaticExecutorEntitiesCollector>();
}

EventsExecutor::~EventsExecutor() {}

void
EventsExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false); );

  // Init entities collector
  entities_collector_->init_events_executor(
    this,
    &EventsExecutor::push_event,
    &exec_list_mutex_);

  std::thread t_exec_timers(&EventsExecutor::execute_timers, this);
  pthread_setname_np(t_exec_timers.native_handle(), "Timers");

  while(spinning.load())
  {
    execute_events();
  }

  t_exec_timers.join();
}

void
EventsExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  bool is_new_node = entities_collector_->add_node(node_ptr);
  if (is_new_node && notify) {
    // Interrupt waiting to handle new node
    if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
      throw std::runtime_error(rcl_get_error_string().str);
    }
  }

  // Add timers to timers heap
  for (auto & weak_group : node_ptr->get_callback_groups()) {
    auto group = weak_group.lock();
    if (!group || !group->can_be_taken_from().load()) {
      continue;
    }
    group->find_timer_ptrs_if(
      [this](const rclcpp::TimerBase::SharedPtr & timer) {
        if (timer) {
        timers.add_timer(timer);
      }
      return false;
    });
  }
}

void
EventsExecutor::add_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->add_node(node_ptr->get_node_base_interface(), notify);
}

void
EventsExecutor::remove_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  bool node_removed = entities_collector_->remove_node(node_ptr);

  if (notify) {
    // If the node was matched and removed, interrupt waiting
    if (node_removed) {
      if (rcl_trigger_guard_condition(&interrupt_guard_condition_) != RCL_RET_OK) {
        throw std::runtime_error(rcl_get_error_string().str);
      }
    }
  }

  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);

  // Remove the timers from the timers heap here?
}

void
EventsExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
EventsExecutor::execute_timers()
{
  while (rclcpp::ok(this->context_) && spinning.load())
  {
    std::this_thread::sleep_for(timers.get_head_timeout());
    timers.execute_ready_timers();
  }
}

void
EventsExecutor::execute_events()
{
  // When condition variable is notified, check this predicate to proceed
  auto predicate = [this]() { return !event_queue.empty(); };

  // Local event queue
  std::queue<EventQ> local_event_queue;

  // Scope block for the mutex
  {
    std::unique_lock<std::mutex> lock(event_queue_mutex_);
    // We wait here until something has been pushed to the event queue
    event_queue_cv.wait(lock, predicate);

    // We got an event! Swap queues and execute events
    swap(local_event_queue, event_queue);
  }

  // Mutex to protect the executable list from being
  // cleared while we still have events to process
  std::unique_lock<std::mutex> lock(exec_list_mutex_);

  // Execute events
  do {
    EventQ event = local_event_queue.front();

    local_event_queue.pop();

    switch(event.type)
    {
    case SUBSCRIPTION_EVENT:
      {
        execute_subscription(std::move(entities_collector_->get_subscription_by_handle(event.entity)));
        break;
      }

    case SERVICE_EVENT:
      {
        execute_service(std::move(entities_collector_->get_service_by_handle(event.entity)));
        break;
      }

    case CLIENT_EVENT:
      {
        execute_client(std::move(entities_collector_->get_client_by_handle(event.entity)));
        break;
      }

     case GUARD_CONDITION_EVENT:
      {
        entities_collector_->get_waitable_by_handle(event.entity)->execute();
        break;
      }

    }
  } while (!local_event_queue.empty());
}
