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
        //execute_subscription(std::move(entities_collector_->get_subscription_by_handle(event.entity)));
        auto subscription = const_cast<rclcpp::SubscriptionBase *>(
                     static_cast<const rclcpp::SubscriptionBase*>(event.entity));
        execute_subscription(subscription);
        break;
      }

    case SERVICE_EVENT:
      {
        //execute_service(std::move(entities_collector_->get_service_by_handle(event.entity)));
        auto service = const_cast<rclcpp::ServiceBase*>(
                static_cast<const rclcpp::ServiceBase*>(event.entity));
        execute_service(service);
        break;
      }

    case CLIENT_EVENT:
      {
        //execute_client(std::move(entities_collector_->get_client_by_handle(event.entity)));
        auto client = const_cast<rclcpp::ClientBase*>(
               static_cast<const rclcpp::ClientBase*>(event.entity));
        execute_client(client);
        break;
      }

     case GUARD_CONDITION_EVENT:
      {
        //entities_collector_->get_waitable_by_handle(event.entity)->execute();
        auto waitable = const_cast<rclcpp::Waitable*>(
                 static_cast<const rclcpp::Waitable*>(event.entity));
        waitable->execute();
        break;
      }

    }
  } while (!local_event_queue.empty());
}


// Raw pointer versions of Executor::execute_<ENTITY>
void
EventsExecutor::execute_subscription(rclcpp::SubscriptionBase* subscription)
{
  rclcpp::MessageInfo message_info;
  message_info.get_rmw_message_info().from_intra_process = false;

  if (subscription->is_serialized()) {
    // This is the case where a copy of the serialized message is taken from
    // the middleware via inter-process communication.
    std::shared_ptr<SerializedMessage> serialized_msg = subscription->create_serialized_message();
    take_and_do_error_handling(
      "taking a serialized message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_serialized(*serialized_msg.get(), message_info);},
      [&]()
      {
        auto void_serialized_msg = std::static_pointer_cast<void>(serialized_msg);
        subscription->handle_message(void_serialized_msg, message_info);
      });
    subscription->return_serialized_message(serialized_msg);
  } else if (subscription->can_loan_messages()) {
    // This is the case where a loaned message is taken from the middleware via
    // inter-process communication, given to the user for their callback,
    // and then returned.
    void * loaned_msg = nullptr;
    // TODO(wjwwood): refactor this into methods on subscription when LoanedMessage
    //   is extened to support subscriptions as well.
    take_and_do_error_handling(
      "taking a loaned message from topic",
      subscription->get_topic_name(),
      [&]()
      {
        rcl_ret_t ret = rcl_take_loaned_message(
          subscription->get_subscription_handle().get(),
          &loaned_msg,
          &message_info.get_rmw_message_info(),
          nullptr);
        if (RCL_RET_SUBSCRIPTION_TAKE_FAILED == ret) {
          return false;
        } else if (RCL_RET_OK != ret) {
          rclcpp::exceptions::throw_from_rcl_error(ret);
        }
        return true;
      },
      [&]() {subscription->handle_loaned_message(loaned_msg, message_info);});
    rcl_ret_t ret = rcl_return_loaned_message_from_subscription(
      subscription->get_subscription_handle().get(),
      loaned_msg);
    if (RCL_RET_OK != ret) {
      RCLCPP_ERROR(
        rclcpp::get_logger("rclcpp"),
        "rcl_return_loaned_message_from_subscription() failed for subscription on topic '%s': %s",
        subscription->get_topic_name(), rcl_get_error_string().str);
    }
    loaned_msg = nullptr;
  } else {
    // This case is taking a copy of the message data from the middleware via
    // inter-process communication.
    std::shared_ptr<void> message = subscription->create_message();
    take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() {subscription->handle_message(message, message_info);});
    subscription->return_message(message);
  }
}

void
EventsExecutor::execute_service(rclcpp::ServiceBase* service)
{
  auto request_header = service->create_request_header();
  std::shared_ptr<void> request = service->create_request();
  take_and_do_error_handling(
    "taking a service server request from service",
    service->get_service_name(),
    [&]() {return service->take_type_erased_request(request.get(), *request_header);},
    [&]() {service->handle_request(request_header, request);});
}

void
EventsExecutor::execute_client(rclcpp::ClientBase* client)
{
  auto request_header = client->create_request_header();
  std::shared_ptr<void> response = client->create_response();
  take_and_do_error_handling(
    "taking a service client response from service",
    client->get_service_name(),
    [&]() {return client->take_type_erased_response(response.get(), *request_header);},
    [&]() {client->handle_response(request_header, response);});
}
