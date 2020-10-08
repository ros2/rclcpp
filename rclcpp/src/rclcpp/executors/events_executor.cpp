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

using rclcpp::executors::EventsExecutor;

EventsExecutor::EventsExecutor(
  const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{
  entities_collector_ = std::make_shared<EventsExecutorEntitiesCollector>();

  // Set entities collector callbacks
  entities_collector_->init(
    this,
    &EventsExecutor::push_event,
    [this](const rclcpp::TimerBase::SharedPtr & t) {
      timers.add_timer(t);
    },
    [this](const rclcpp::TimerBase::SharedPtr & t) {
      timers.remove_timer(t);
    },
    [this]() {
      timers.clear_all();
    });

}

EventsExecutor::~EventsExecutor() {}

void
EventsExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false););

  std::thread t_spin_timers(&EventsExecutor::spin_timers, this, false);
  pthread_setname_np(t_spin_timers.native_handle(), "Timers");

  while (rclcpp::ok(context_) && spinning.load())
  {
    execute_events();
  }

  t_spin_timers.join();
}

// Before calling spin_some
void
EventsExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  (void)max_duration;

  // Check, are we already spinning?
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }
  RCLCPP_SCOPE_EXIT(this->spinning.store(false););

  std::thread t_spin_timers(&EventsExecutor::spin_timers, this, true);

  // Execute events and leave
  execute_events();

  t_spin_timers.join();
}

void
EventsExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  (void) notify;

  // Add node to entities collector
  entities_collector_->add_node(node_ptr);

  // Get nodes entities, and assign their callbaks
  for (auto & weak_group : node_ptr->get_callback_groups()) {
    auto group = weak_group.lock();
    if (!group || !group->can_be_taken_from().load()) {
      continue;
    }
    // Add timers to timers heap
    group->find_timer_ptrs_if(
      [this](const rclcpp::TimerBase::SharedPtr & timer) {
        if (timer) {
          timers.add_timer(timer);
        }
        return false;
    });
    // Set the callbacks to all the entities
    group->find_subscription_ptrs_if(
      [this](const rclcpp::SubscriptionBase::SharedPtr & subscription) {
        if (subscription) {
          subscription->set_callback(this, &EventsExecutor::push_event);
        }
        return false;
      });
    group->find_service_ptrs_if(
      [this](const rclcpp::ServiceBase::SharedPtr & service) {
        if (service) {
          service->set_callback(this, &EventsExecutor::push_event);
        }
        return false;
      });
    group->find_client_ptrs_if(
      [this](const rclcpp::ClientBase::SharedPtr & client) {
        if (client) {
          client->set_callback(this, &EventsExecutor::push_event);
        }
        return false;
      });
    group->find_waitable_ptrs_if(
      [this](const rclcpp::Waitable::SharedPtr & waitable) {
        if (waitable) {
          waitable->set_guard_condition_callback(this, &EventsExecutor::push_event);
        }
        return false;
      });
  }

  // Set node's guard condition callback, so if new entities are added while
  // spinning we can set their callback.
  rcl_ret_t ret = rcl_guard_condition_set_callback(
                    this,
                    &EventsExecutor::push_event,
                    entities_collector_.get(),
                    node_ptr->get_notify_guard_condition(),
                    false /* Discard previous events */);

  if (ret != RCL_RET_OK) {
    throw std::runtime_error(std::string("Couldn't set guard condition callback"));
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
  (void)notify;
  entities_collector_->remove_node(node_ptr);

  std::atomic_bool & has_executor = node_ptr->get_associated_with_executor_atomic();
  has_executor.store(false);
}

void
EventsExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
EventsExecutor::spin_timers(bool spin_once)
{
  while (rclcpp::ok(context_) && spinning.load())
  {
    std::this_thread::sleep_for(timers.get_head_timeout());
    timers.execute_ready_timers();
    if (spin_once) {
      break;
    }
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

  // Execute events
  do {
    EventQ event = local_event_queue.front();

    local_event_queue.pop();

    switch(event.type)
    {
    case SUBSCRIPTION_EVENT:
      {
        auto subscription = const_cast<rclcpp::SubscriptionBase *>(
                     static_cast<const rclcpp::SubscriptionBase*>(event.entity));
        execute_subscription(subscription);
        break;
      }

    case SERVICE_EVENT:
      {
        auto service = const_cast<rclcpp::ServiceBase*>(
                static_cast<const rclcpp::ServiceBase*>(event.entity));
        execute_service(service);
        break;
      }

    case CLIENT_EVENT:
      {
        auto client = const_cast<rclcpp::ClientBase*>(
               static_cast<const rclcpp::ClientBase*>(event.entity));
        execute_client(client);
        break;
      }

     case GUARD_CONDITION_EVENT:
      {
        auto waitable = const_cast<rclcpp::Waitable*>(
                 static_cast<const rclcpp::Waitable*>(event.entity));
        waitable->execute();
        break;
      }

    }
  } while (!local_event_queue.empty());
}
