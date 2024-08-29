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
#include <utility>
#include <vector>

#include "rcpputils/scope_exit.hpp"

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
  // The timers manager can be used either to only track timers (in this case an expired
  // timer will generate an executor event and then it will be executed by the executor thread)
  // or it can also take care of executing expired timers in its dedicated thread.
  std::function<void(const rclcpp::TimerBase *,
    const std::shared_ptr<void> &)> timer_on_ready_cb = nullptr;
  if (!execute_timers_separate_thread) {
    timer_on_ready_cb =
      [this](const rclcpp::TimerBase * timer_id, const std::shared_ptr<void> & data) {
        ExecutorEvent event = {timer_id, data, -1, ExecutorEventType::TIMER_EVENT, 1};
        this->events_queue_->enqueue(event);
      };
  }
  timers_manager_ =
    std::make_shared<rclcpp::experimental::TimersManager>(context_, timer_on_ready_cb);

  entities_need_rebuild_ = false;

  this->setup_notify_waitable();

  // Ensure that the entities collection is empty (the base class may have added elements
  // that we are not interested in)
  this->current_collection_.clear();

  // Make sure that the notify waitable is immediately added to the collection
  // to avoid missing events
  this->add_notify_waitable_to_collection(current_collection_.waitables);
}

void
EventsExecutor::setup_notify_waitable()
{
  // The base class already created this object but the events-executor
  // needs different callbacks.
  assert(notify_waitable_ && "The notify waitable should have already been constructed");

  notify_waitable_->set_execute_callback(
    [this]() {
      // This callback is invoked when:
      // - the interrupt or shutdown guard condition is triggered:
      //    ---> we need to wake up the executor so that it can terminate
      // - a node or callback group guard condition is triggered:
      //    ---> the entities collection is changed, we need to update callbacks
      this->handle_updated_entities(false);
    });

  auto notify_waitable_entity_id = notify_waitable_.get();
  notify_waitable_->set_on_ready_callback(
    [this, notify_waitable_entity_id](size_t num_events, int waitable_data) {
      // The notify waitable has a special callback.
      // We don't care about how many events as when we wake up the executor we are going to
      // process everything regardless.
      // For the same reason, if an event of this type has already been pushed but it has not been
      // processed yet, we avoid pushing additional events.
      (void)num_events;
      if (entities_need_rebuild_.exchange(true)) {
        return;
      }

      ExecutorEvent event =
      {notify_waitable_entity_id, nullptr, waitable_data, ExecutorEventType::WAITABLE_EVENT, 1};
      this->events_queue_->enqueue(event);
    });
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

  // If this spin is not exhaustive (e.g. spin_some), we need to explicitly check
  // if entities need to be rebuilt here rather than letting the notify waitable event do it.
  // A non-exhaustive spin would not check for work a second time, thus delaying the execution
  // of some entities to the next invocation of spin.
  if (!exhaustive) {
    this->handle_updated_entities(false);
  }

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

  // Select the smallest between input timeout and timer timeout.
  // Cancelled timers are not considered.
  bool is_timer_timeout = false;
  auto next_timer_timeout = timers_manager_->get_head_timeout();
  if (next_timer_timeout.has_value() && next_timer_timeout.value() < timeout) {
    timeout = next_timer_timeout.value();
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
EventsExecutor::execute_event(const ExecutorEvent & event)
{
  switch (event.type) {
    case ExecutorEventType::CLIENT_EVENT:
      {
        rclcpp::ClientBase::SharedPtr client;
        {
          client = this->retrieve_entity(
            static_cast<const rcl_client_t *>(event.entity_key),
            current_collection_.clients);
        }
        if (client) {
          for (size_t i = 0; i < event.num_events; i++) {
            execute_client(client);
          }
        }

        break;
      }
    case ExecutorEventType::SUBSCRIPTION_EVENT:
      {
        rclcpp::SubscriptionBase::SharedPtr subscription;
        {
          subscription = this->retrieve_entity(
            static_cast<const rcl_subscription_t *>(event.entity_key),
            current_collection_.subscriptions);
        }
        if (subscription) {
          for (size_t i = 0; i < event.num_events; i++) {
            execute_subscription(subscription);
          }
        }
        break;
      }
    case ExecutorEventType::SERVICE_EVENT:
      {
        rclcpp::ServiceBase::SharedPtr service;
        {
          service = this->retrieve_entity(
            static_cast<const rcl_service_t *>(event.entity_key),
            current_collection_.services);
        }
        if (service) {
          for (size_t i = 0; i < event.num_events; i++) {
            execute_service(service);
          }
        }

        break;
      }
    case ExecutorEventType::TIMER_EVENT:
      {
        timers_manager_->execute_ready_timer(
          static_cast<const rclcpp::TimerBase *>(event.entity_key), event.data);
        break;
      }
    case ExecutorEventType::WAITABLE_EVENT:
      {
        rclcpp::Waitable::SharedPtr waitable;
        {
          waitable = this->retrieve_entity(
            static_cast<const rclcpp::Waitable *>(event.entity_key),
            current_collection_.waitables);
        }
        if (waitable) {
          for (size_t i = 0; i < event.num_events; i++) {
            const auto data = waitable->take_data_by_entity_id(event.waitable_data);
            waitable->execute(data);
          }
        }
        break;
      }
  }
}

void
EventsExecutor::handle_updated_entities(bool notify)
{
  (void)notify;

  // Do not rebuild if we don't need to.
  // A rebuild event could be generated, but then
  // this function could end up being called from somewhere else
  // before that event gets processed, for example if
  // a node or callback group is manually added to the executor.
  const bool notify_waitable_triggered = entities_need_rebuild_.exchange(false);
  if (!notify_waitable_triggered && !this->collector_.has_pending()) {
    return;
  }

  // Build the new collection
  this->collector_.update_collections();
  auto callback_groups = this->collector_.get_all_callback_groups();
  rclcpp::executors::ExecutorEntitiesCollection new_collection;
  rclcpp::executors::build_entities_collection(callback_groups, new_collection);

  // TODO(alsora): this may be implemented in a better way.
  // We need the notify waitable to be included in the executor "current_collection"
  // because we need to be able to retrieve events for it.
  // We could explicitly check for the notify waitable ID when we receive a waitable event
  // but I think that it's better if the waitable was in the collection and it could be
  // retrieved in the "standard" way.
  // To do it, we need to add the notify waitable as an entry in the new collection
  // such that it's neither added or removed (it should have already been added
  // to the current collection in the constructor)
  this->add_notify_waitable_to_collection(new_collection.waitables);

  this->refresh_current_collection(new_collection);
}

void
EventsExecutor::refresh_current_collection(
  const rclcpp::executors::ExecutorEntitiesCollection & new_collection)
{
  // Acquire lock before modifying the current collection
  std::lock_guard<std::mutex> guard(mutex_);

  // Remove expired entities to ensure re-initialized objects
  // are updated. This fixes issues with stale state entities.
  // See: https://github.com/ros2/rclcpp/pull/2586
  current_collection_.remove_expired_entities();

  current_collection_.timers.update(
    new_collection.timers,
    [this](rclcpp::TimerBase::SharedPtr timer) {timers_manager_->add_timer(timer);},
    [this](rclcpp::TimerBase::SharedPtr timer) {timers_manager_->remove_timer(timer);});

  current_collection_.subscriptions.update(
    new_collection.subscriptions,
    [this](auto subscription) {
      subscription->set_on_new_message_callback(
        this->create_entity_callback(
          subscription->get_subscription_handle().get(), ExecutorEventType::SUBSCRIPTION_EVENT));
    },
    [](auto subscription) {subscription->clear_on_new_message_callback();});

  current_collection_.clients.update(
    new_collection.clients,
    [this](auto client) {
      client->set_on_new_response_callback(
        this->create_entity_callback(
          client->get_client_handle().get(), ExecutorEventType::CLIENT_EVENT));
    },
    [](auto client) {client->clear_on_new_response_callback();});

  current_collection_.services.update(
    new_collection.services,
    [this](auto service) {
      service->set_on_new_request_callback(
        this->create_entity_callback(
          service->get_service_handle().get(), ExecutorEventType::SERVICE_EVENT));
    },
    [](auto service) {service->clear_on_new_request_callback();});

  // DO WE NEED THIS? WE ARE NOT DOING ANYTHING WITH GUARD CONDITIONS
  /*
  current_collection_.guard_conditions.update(new_collection.guard_conditions,
    [](auto guard_condition) {(void)guard_condition;},
    [](auto guard_condition) {guard_condition->set_on_trigger_callback(nullptr);});
  */

  current_collection_.waitables.update(
    new_collection.waitables,
    [this](auto waitable) {
      waitable->set_on_ready_callback(
        this->create_waitable_callback(waitable.get()));
    },
    [](auto waitable) {waitable->clear_on_ready_callback();});
}

std::function<void(size_t)>
EventsExecutor::create_entity_callback(
  void * entity_key, ExecutorEventType event_type)
{
  std::function<void(size_t)>
  callback = [this, entity_key, event_type](size_t num_events) {
      ExecutorEvent event = {entity_key, nullptr, -1, event_type, num_events};
      this->events_queue_->enqueue(event);
    };
  return callback;
}

std::function<void(size_t, int)>
EventsExecutor::create_waitable_callback(const rclcpp::Waitable * entity_key)
{
  std::function<void(size_t, int)>
  callback = [this, entity_key](size_t num_events, int waitable_data) {
      ExecutorEvent event =
      {entity_key, nullptr, waitable_data, ExecutorEventType::WAITABLE_EVENT, num_events};
      this->events_queue_->enqueue(event);
    };
  return callback;
}

void
EventsExecutor::add_notify_waitable_to_collection(
  rclcpp::executors::ExecutorEntitiesCollection::WaitableCollection & collection)
{
  // The notify waitable is not associated to any group, so use an invalid one
  rclcpp::CallbackGroup::WeakPtr weak_group_ptr;
  collection.insert(
  {
    this->notify_waitable_.get(),
    {this->notify_waitable_, weak_group_ptr}
  });
}
