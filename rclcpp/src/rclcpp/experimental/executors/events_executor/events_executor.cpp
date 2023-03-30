// Copyright 2023 iRobot Corporation.
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

#include "rclcpp/experimental/executors/events_executor/events_executor.hpp"

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/exceptions/exceptions.hpp"

using namespace std::chrono_literals;

using rclcpp::experimental::executors::EventsExecutor;

EventsExecutor::EventsExecutor(
  rclcpp::experimental::executors::EventsQueue::UniquePtr events_queue,
  bool execute_timers_separate_thread,
  const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options)
{
  // Get ownership of the queue used to store events.
  if (!events_queue) {
    throw std::invalid_argument("events_queue can't be a null pointer");
  }
  events_queue_ = std::move(events_queue);

  // Create timers manager
  std::function<void(void *)> timer_on_ready_cb = nullptr;
  if (execute_timers_separate_thread) {
    timer_on_ready_cb = [this](const void * timer_id) {
        ExecutorEvent event = {timer_id, -1, ExecutorEventType::TIMER_EVENT, 1};
        this->events_queue_->enqueue(event);
      };
  }
  timers_manager_ =
    std::make_shared<rclcpp::experimental::TimersManager>(context_, timer_on_ready_cb);

  notify_waitable_ = std::make_shared<rclcpp::executors::ExecutorNotifyWaitable>(
    [this](){
      // This callback is invoked when:
      // - the interrupt or shutdown guard condition is triggered:
      //    ---> we need to wake up the executor so that it can terminate
      // - a node or callback group guard condition is triggered:
      //    ---> the entities collection is changed, we need to update callbacks
      std::cout<<"EXECUTE NOTIFY WAITABLE!!!!!!!!"<<std::endl;
      this->refresh_current_collection_from_callback_groups();
    });

  std::cout<<"NOTIFY WATIABLE ID IS --------> "<< notify_waitable_.get()<<std::endl;

  notify_waitable_->add_guard_condition(&interrupt_guard_condition_);
  notify_waitable_->add_guard_condition(shutdown_guard_condition_.get());

  notify_waitable_->set_on_ready_callback(
    this->create_waitable_callback(notify_waitable_.get()));

  this->entities_collector_ = std::make_shared<rclcpp::executors::ExecutorEntitiesCollector>(
    notify_waitable_);

  this->current_entities_collection_ = std::make_shared<rclcpp::executors::ExecutorEntitiesCollection>();
}

EventsExecutor::~EventsExecutor()
{
  spinning.store(false);
  notify_waitable_->clear_on_ready_callback();
  this->refresh_current_collection({});
}

void
EventsExecutor::spin()
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }
  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

  timers_manager_->start();
  RCPPUTILS_SCOPE_EXIT(timers_manager_->stop(); );

  while (rclcpp::ok(context_) && spinning.load()) {
    // Wait until we get an event
    ExecutorEvent event;
    bool has_event = events_queue_->dequeue(event);
    if (has_event) {
      std::cout<<"Got event!"<<std::endl;
      this->execute_event(event);
    }
  }
}

void
EventsExecutor::spin_some(std::chrono::nanoseconds max_duration)
{
  return this->spin_some_impl(max_duration, false);
}

void
EventsExecutor::spin_all(std::chrono::nanoseconds max_duration)
{
  if (max_duration <= 0ns) {
    throw std::invalid_argument("max_duration must be positive");
  }
  return this->spin_some_impl(max_duration, true);
}

void
EventsExecutor::spin_some_impl(std::chrono::nanoseconds max_duration, bool exhaustive)
{
  if (spinning.exchange(true)) {
    throw std::runtime_error("spin_some() called while already spinning");
  }

  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );

  auto start = std::chrono::steady_clock::now();

  auto max_duration_not_elapsed = [max_duration, start]() {
      if (std::chrono::nanoseconds(0) == max_duration) {
        // told to spin forever if need be
        return true;
      } else if (std::chrono::steady_clock::now() - start < max_duration) {
        // told to spin only for some maximum amount of time
        return true;
      }
      // spun too long
      return false;
    };

  // Get the number of events and timers ready at start
  const size_t ready_events_at_start = events_queue_->size();
  size_t executed_events = 0;
  const size_t ready_timers_at_start = timers_manager_->get_number_ready_timers();
  size_t executed_timers = 0;

  while (rclcpp::ok(context_) && spinning.load() && max_duration_not_elapsed()) {
    // Execute first ready event from queue if exists
    if (exhaustive || (executed_events < ready_events_at_start)) {
      bool has_event = !events_queue_->empty();

      if (has_event) {
        ExecutorEvent event;
        bool ret = events_queue_->dequeue(event, std::chrono::nanoseconds(0));
        if (ret) {
          this->execute_event(event);
          executed_events++;
          continue;
        }
      }
    }

    // Execute first timer if it is ready
    if (exhaustive || (executed_timers < ready_timers_at_start)) {
      bool timer_executed = timers_manager_->execute_head_timer();
      if (timer_executed) {
        executed_timers++;
        continue;
      }
    }

    // If there's no more work available, exit
    break;
  }
}

void
EventsExecutor::spin_once_impl(std::chrono::nanoseconds timeout)
{
  // In this context a negative input timeout means no timeout
  if (timeout < 0ns) {
    timeout = std::chrono::nanoseconds::max();
  }

  // Select the smallest between input timeout and timer timeout
  bool is_timer_timeout = false;
  auto next_timer_timeout = timers_manager_->get_head_timeout();
  if (next_timer_timeout < timeout) {
    timeout = next_timer_timeout;
    is_timer_timeout = true;
  }

  ExecutorEvent event;
  bool has_event = events_queue_->dequeue(event, timeout);

  // If we wake up from the wait with an event, it means that it
  // arrived before any of the timers expired.
  if (has_event) {
    this->execute_event(event);
  } else if (is_timer_timeout) {
    timers_manager_->execute_head_timer();
  }
}

void
EventsExecutor::add_node(
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr, bool notify)
{
  // This field is unused because we don't have to wake up the executor when a node is added.
  (void) notify;

  // Add node to entities collector
  this->entities_collector_->add_node(node_ptr);

  this->refresh_current_collection_from_callback_groups();
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
  // This field is unused because we don't have to wake up the executor when a node is removed.
  (void)notify;

  // Remove node from entities collector.
  // This will result in un-setting all the event callbacks from its entities.
  // After this function returns, this executor will not receive any more events associated
  // to these entities.
  this->entities_collector_->remove_node(node_ptr);

  this->refresh_current_collection_from_callback_groups();
}

void
EventsExecutor::remove_node(std::shared_ptr<rclcpp::Node> node_ptr, bool notify)
{
  this->remove_node(node_ptr->get_node_base_interface(), notify);
}

void
EventsExecutor::execute_event(const ExecutorEvent & event)
{
  switch (event.type) {
    case ExecutorEventType::CLIENT_EVENT:
      {
        auto client = this->retrieve_entity(
          static_cast<const rcl_client_t *>(event.exec_entity_id),
          current_entities_collection_->clients);

        if (client) {
          for (size_t i = 0; i < event.num_events; i++) {
            execute_client(client);
          }
        }

        break;
      }
    case ExecutorEventType::SUBSCRIPTION_EVENT:
      {
        std::cout<<"Processing subscription event for "<< event.exec_entity_id <<std::endl;
        auto subscription = this->retrieve_entity(
          static_cast<const rcl_subscription_t *>(event.exec_entity_id),
          current_entities_collection_->subscriptions);
        if (subscription) {
          std::cout<<"------ Executing subscription!"<<std::endl;
          for (size_t i = 0; i < event.num_events; i++) {
            execute_subscription(subscription);
          }
        }
        break;
      }
    case ExecutorEventType::SERVICE_EVENT:
      {
        auto service = this->retrieve_entity(
          static_cast<const rcl_service_t *>(event.exec_entity_id),
          current_entities_collection_->services);

        if (service) {
          for (size_t i = 0; i < event.num_events; i++) {
            execute_service(service);
          }
        }

        break;
      }
    case ExecutorEventType::TIMER_EVENT:
      {
        timers_manager_->execute_ready_timer(event.exec_entity_id);
        break;
      }
    case ExecutorEventType::WAITABLE_EVENT:
      {
        std::cout<<"Processing waitable event for "<< event.exec_entity_id <<std::endl;
        auto waitable = this->retrieve_entity(
          static_cast<const rclcpp::Waitable *>(event.exec_entity_id),
          current_entities_collection_->waitables);
        if (waitable) {
          std::cout<<"------ Executing waitable!"<<std::endl;
          for (size_t i = 0; i < event.num_events; i++) {
            auto data = waitable->take_data_by_entity_id(event.gen_entity_id);
            waitable->execute(data);
          }
        }
        break;
      }
  }
}

void
EventsExecutor::add_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr,
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_ptr,
  bool notify)
{
  // This field is unused because we don't have to wake up
  // the executor when a callback group is added.
  (void)notify;
  (void)node_ptr;

  this->entities_collector_->add_callback_group(group_ptr);

  this->refresh_current_collection_from_callback_groups();
}

void
EventsExecutor::remove_callback_group(
  rclcpp::CallbackGroup::SharedPtr group_ptr, bool notify)
{
  // This field is unused because we don't have to wake up
  // the executor when a callback group is removed.
  (void)notify;
  //entities_collector_->remove_callback_group(group_ptr);

  this->entities_collector_->remove_callback_group(group_ptr);

  this->refresh_current_collection_from_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsExecutor::get_all_callback_groups()
{
  return this->entities_collector_->get_all_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsExecutor::get_manually_added_callback_groups()
{
  return this->entities_collector_->get_manually_added_callback_groups();
}

std::vector<rclcpp::CallbackGroup::WeakPtr>
EventsExecutor::get_automatically_added_callback_groups_from_nodes()
{
  return this->entities_collector_->get_automatically_added_callback_groups();
}

void
EventsExecutor::refresh_current_collection_from_callback_groups()
{
  auto callback_groups = this->entities_collector_->get_all_callback_groups();
  rclcpp::executors::ExecutorEntitiesCollection new_collection;
  rclcpp::executors::build_entities_collection(callback_groups, new_collection);

  rclcpp::CallbackGroup::WeakPtr weak_group_ptr;
  new_collection.waitables.insert(
    {
      this->notify_waitable_.get(),
      {this->notify_waitable_, weak_group_ptr}
    });

  this->current_entities_collection_->waitables.insert(
    {
      this->notify_waitable_.get(),
      {this->notify_waitable_, weak_group_ptr}
    });

  this->refresh_current_collection(new_collection);
}

void
EventsExecutor::refresh_current_collection(const rclcpp::executors::ExecutorEntitiesCollection & new_collection)
{
  current_entities_collection_->timers.update(new_collection.timers,
    [this](rclcpp::TimerBase::SharedPtr timer){timers_manager_->add_timer(timer);},
    [this](rclcpp::TimerBase::SharedPtr timer){timers_manager_->remove_timer(timer);});

  current_entities_collection_->subscriptions.update(new_collection.subscriptions,
    [this](auto subscription){
      subscription->set_on_new_message_callback(
        this->create_entity_callback(subscription->get_subscription_handle().get(), ExecutorEventType::SUBSCRIPTION_EVENT));
    },
    [](auto subscription){subscription->clear_on_new_message_callback();});

  current_entities_collection_->clients.update(new_collection.clients,
    [this](auto client){
      client->set_on_new_response_callback(
        this->create_entity_callback(client->get_client_handle().get(), ExecutorEventType::CLIENT_EVENT));
    },
    [](auto client){client->clear_on_new_response_callback();});


  current_entities_collection_->services.update(new_collection.services,
    [this](auto service){
      service->set_on_new_request_callback(
        this->create_entity_callback(service->get_service_handle().get(), ExecutorEventType::SERVICE_EVENT));
    },
    [](auto service){service->clear_on_new_request_callback();});

  // DO WE NEED THIS? WE ARE NOT DOING ANYTHING WITH GUARD CONDITIONS
  /*
  current_entities_collection_->guard_conditions.update(new_collection.guard_conditions,
    [](auto guard_condition){(void)guard_condition;},
    [](auto guard_condition){guard_condition->set_on_trigger_callback(nullptr);});
  */

  //std::cout<<"Refresh waitables collection: "<< current_entities_collection_->waitables.size() << " vs " << new_collection.waitables.size() <<std::endl;

  current_entities_collection_->waitables.update(new_collection.waitables,
    [this](auto waitable){
      waitable->set_on_ready_callback(
        this->create_waitable_callback(waitable.get()));
    },
    [](auto waitable){waitable->clear_on_ready_callback();});

  //std::cout<<"New waitable collection: "<< current_entities_collection_->waitables.size() <<std::endl;

}

std::function<void(size_t)>
EventsExecutor::create_entity_callback(
  void * exec_entity_id, ExecutorEventType event_type)
{
  std::cout<<"Create entity callback for "<< exec_entity_id<< std::endl;
  std::function<void(size_t)>
  callback = [this, exec_entity_id, event_type](size_t num_events) {
      std::cout<<"Pushing event of type "<< event_type << " to entity " << exec_entity_id << std::endl;
      ExecutorEvent event = {exec_entity_id, -1, event_type, num_events};
      this->events_queue_->enqueue(event);
    };
  return callback;
}

std::function<void(size_t, int)>
EventsExecutor::create_waitable_callback(void * exec_entity_id)
{
  std::cout<<"Create waitable entity callback for "<< exec_entity_id<< std::endl;
  std::function<void(size_t, int)>
  callback = [this, exec_entity_id](size_t num_events, int gen_entity_id) {
      ExecutorEvent event =
      {exec_entity_id, gen_entity_id, ExecutorEventType::WAITABLE_EVENT, num_events};
      this->events_queue_->enqueue(event);
    };
  return callback;
}
